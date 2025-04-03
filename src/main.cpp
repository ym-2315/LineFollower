#include <Arduino.h>

// IR Sensor
constexpr uint8_t NumPins = 8;
constexpr uint8_t IRPins[NumPins] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Motor A (RIGHT)
constexpr uint8_t SpeedPinA = 5;
constexpr uint8_t ForWordPinA = 6;
constexpr uint8_t ReversePinA = 7;

// Motor B (LEFT)
constexpr uint8_t SpeedPinB = 10;
constexpr uint8_t ForWordPinB = 8;
constexpr uint8_t ReversePinB = 9;

constexpr uint8_t NumRaws = 5;

typedef enum {
    FORWARD,
    BACKWARD,
    STOP
} Direction;

class Sensor {
public:
    uint8_t sensorValue = 0b00000000;
    uint8_t previousValue[NumRaws] = {0b00000000};
    uint16_t threshold[NumPins] = {512};
    uint16_t maxRead[NumPins] = {0};
    uint16_t minRead[NumPins] = {1023};

    explicit Sensor() : sensorValue(), threshold{}, maxRead{}, minRead{} {}

    void calibrate(const uint32_t CalibrationTime) {
        const uint32_t CalibrationStart = millis();
        while ((millis() - CalibrationStart) < CalibrationTime) {
            for (uint8_t i = 0; i < NumPins; i++) {
                uint16_t value = analogRead(IRPins[i]);
                maxRead[i] = max(maxRead[i], value);
                minRead[i] = min(minRead[i], value);
            }
        }

        for (uint8_t i = 0; i < NumPins; i++) {
            threshold[i] = (2 * maxRead[i] + minRead[i]) / 3;
        }

    }

    void read(const uint8_t n) {
        sensorValue = 0b00000000;
        uint8_t voteCount[NumPins] = {0};

        for (uint8_t j = 0; j < n; j++) {
            for (uint8_t i = 0; i < NumPins; i++) {
                if (static_cast<uint16_t>(analogRead(IRPins[i])) > threshold[i]) {
                    voteCount[i]++;
                }
            }
        }

        for (uint8_t i = 0; i < NumPins; i++) {
            if (voteCount[i] > (n / 2)) {
                sensorValue |= (1 << i);
            }
        }
    }

    void update(const uint8_t n) {
        if (n > 0) {
            read(n);
        } else {
            for (uint8_t i = 0; i < NumPins; i++) {
                if (static_cast<uint16_t>(analogRead(IRPins[i])) > threshold[i]) {
                    sensorValue |= (1 << i);
                }
            }
        }

        if (sensorValue != previousValue[0]) {
            for (uint8_t i = NumRaws-1; i > 0; i--) {
                previousValue[i] = previousValue[i - 1];
            }
            previousValue[0] = sensorValue;
        }
    }

    uint8_t getLinePosition() const {

        uint8_t countOnes = 0;
        for (uint8_t i = 0; i < 8; i++) {
            if (sensorValue & (1 << i)) countOnes++;
        }

        switch (countOnes) {
            case 1:
                for (uint8_t i = 0; i < 8; i++) {
                    if (sensorValue == (1 << i)) return (2 * i);
                }
            case 2:
                for (uint8_t i = 0; i < 7; i++) {
                    if (sensorValue == (3 << i)) return (2 * i + 1);
                }
            default: return static_cast<uint8_t>(-1); // Error: two line detected
        }
    }
};

class Motor {
public:
    const uint8_t speedPin, forWordPin, reversePin;

    Motor(const uint8_t speed, const uint8_t forward, const uint8_t reverse) : speedPin(speed), forWordPin(forward), reversePin(reverse) {
        pinMode(speedPin, OUTPUT);
        pinMode(forWordPin, OUTPUT);
        pinMode(reversePin, OUTPUT);
    }

    void setDirection(const Direction dir) const {
        switch (dir) {
            case FORWARD:
                digitalWrite(forWordPin, HIGH);
                digitalWrite(reversePin, LOW);
                break;
            case BACKWARD:
                digitalWrite(forWordPin, LOW);
                digitalWrite(reversePin, HIGH);
                break;
            case STOP:
                digitalWrite(forWordPin, LOW);
                digitalWrite(reversePin, LOW);
                break;
        }
    }

    void setSpeed(const uint8_t speed) const {
        analogWrite(speedPin, speed);
    }
};

constexpr int8_t Ki = 5;
constexpr int8_t Kp = 5;
constexpr int8_t Kd = 5;

constexpr uint8_t TargetPosition = 7;
constexpr uint8_t Speed = 200;
constexpr uint8_t MaxSpeed = 255;
constexpr uint8_t MinSpeed = 150;


constexpr uint32_t CalibrationTime = 5000;

int8_t SpeedControl = 0;

int8_t Error = 0;
int8_t PreviousError = 0;
int8_t IntegralError = 0;
int8_t DerivativeError = 0;

uint8_t MotorASpeed = Speed;
uint8_t MotorBSpeed = Speed;

Motor motorA(SpeedPinA, ForWordPinA, ReversePinA);
Motor motorB(SpeedPinB, ForWordPinB, ReversePinB);
Sensor irSensor;

void setup() {
    Serial.begin(9600);
    pinMode(LED_BUILTIN, OUTPUT);

    for(const uint8_t IRPin : IRPins) {
        pinMode(IRPin, INPUT);
    }

    motorA.setDirection(STOP);
    motorB.setDirection(STOP);
    motorA.setSpeed(Speed);
    motorB.setSpeed(Speed);

    digitalWrite(LED_BUILTIN, HIGH);
    irSensor.calibrate(CalibrationTime);
    digitalWrite(LED_BUILTIN, LOW);

    Serial.print("Calibrated threshold: ");
    for (const uint16_t threshold: irSensor.threshold) {
        Serial.print(threshold);
        Serial.print(" ");
    }
    Serial.println("");

}

void loop() {
    const uint32_t time = millis();
    irSensor.update(1);
    motorA.setDirection(FORWARD);
    motorB.setDirection(FORWARD);

    const uint8_t pos = irSensor.getLinePosition();

    if (pos == 0xFF) {
        motorA.setDirection(FORWARD);
        motorB.setDirection(FORWARD);
    } else {
        Error = static_cast<int8_t>(TargetPosition - pos);
        IntegralError += Error;
        DerivativeError = Error - PreviousError;

        SpeedControl = Kp * Error + Ki * IntegralError + Kd * DerivativeError;
        PreviousError = Error;
    }

    Serial.print(" Speed: ");
    Serial.println(SpeedControl);

    MotorASpeed = constrain(Speed + SpeedControl, MinSpeed, MaxSpeed);
    MotorBSpeed = constrain(Speed - SpeedControl, MinSpeed, MaxSpeed);

    motorA.setSpeed(MotorASpeed);
    motorB.setSpeed(MotorBSpeed);
    const uint32_t end = millis() - time;

    Serial.print("IR: ");
    Serial.print((uint32_t) irSensor.sensorValue + 0x0100, BIN);
    Serial.print(" Time (ms): ");
    Serial.print(end);

    Serial.print(" Error: ");
    Serial.print(Error);

}
