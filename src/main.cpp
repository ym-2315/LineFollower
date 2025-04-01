#include <Arduino.h>

// IR Sensor
constexpr uint8_t NumPins = 8;
constexpr uint8_t IRPins[NumPins] = {A0, A1, A2, A3, A4, A5, A6, A7};

// Motor A (RIGHT)
constexpr uint8_t SpeedPinA = PD5;
constexpr uint8_t ForWordPinA = PD6;
constexpr uint8_t ReversePinA = PD7;

// Motor B (LEFT)
constexpr uint8_t SpeedPinB = PB2;
constexpr uint8_t ForWordPinB = PB1;
constexpr uint8_t ReversePinB = PB0;

typedef enum {
    FORWARD,
    BACKWARD,
    STOP
} Direction;

class Sensor {
public:
    uint8_t sensorValue;
    uint16_t threshold;
    uint16_t maxRead = 0;
    uint16_t minRead = UINT16_MAX;

    explicit Sensor(const uint16_t initialThreshold = 250) : sensorValue(0), threshold(initialThreshold) {
    }

    static void calibrate() {
        uint16_t maxRead = 0, minRead = 1023;
        for (const uint8_t Pin : IRPins) {
            uint16_t value = analogRead(Pin);
            maxRead = max(maxRead, value);
            minRead = min(minRead, value);
        }
    }

    void read(const uint8_t n) {
        sensorValue = 0;
        uint8_t voteCount[NumPins] = {0};

        for (uint8_t j = 0; j < n; j++) {
            for (uint8_t i = 0; i < NumPins; i++) {
                if (analogRead(IRPins[i]) > threshold) {
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
    const  uint8_t speedPin, forWordPin, reversePin;

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

constexpr int8_t Ki = 20;
constexpr int8_t Kp = 20;
constexpr int8_t Kd = 0;

constexpr uint8_t TargetPosition = 7;
constexpr uint8_t Speed = 200;
constexpr uint8_t MaxSpeed = 255;
constexpr uint8_t MinSpeed = 150;
constexpr uint32_t LostLineLimit = 5000;
constexpr uint32_t CalibrationTime = 10000;

uint32_t lostLineTime = 0;
uint32_t SpeedControl = 0;

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

    motorA.setDirection(STOP);
    motorB.setDirection(STOP);
    motorA.setSpeed(Speed);
    motorB.setSpeed(Speed);

    const uint64_t StartTime = millis();
    while (millis() - StartTime < CalibrationTime) {
        Sensor::calibrate();
        if (millis() - StartTime % 500 == 0)
            digitalWrite(LED_BUILTIN, HIGH);
        else
            digitalWrite(LED_BUILTIN, LOW);
    }

    irSensor.threshold = (irSensor.minRead  + irSensor.maxRead) / 2;

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Calibrated threshold: ");
    Serial.println(irSensor.threshold);
}

void loop() {
    irSensor.read(3);
    motorA.setDirection(FORWARD);
    motorB.setDirection(FORWARD);

    Error = static_cast<int8_t>(TargetPosition - irSensor.getLinePosition());

    IntegralError += Error;
    DerivativeError = Error - PreviousError;

    SpeedControl = Kp * Error + Ki * IntegralError + Kd * DerivativeError;
    PreviousError = Error;

    MotorASpeed = constrain(Speed + SpeedControl, MinSpeed, MaxSpeed);
    MotorBSpeed = constrain(Speed - SpeedControl, MinSpeed, MaxSpeed);

    motorA.setSpeed(MotorASpeed);
    motorB.setSpeed(MotorBSpeed);
}
