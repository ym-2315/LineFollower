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

typedef enum {
    BLANC,
    LEFT,
    RIGHT,
    CENTER,
    EXTREME_LEFT,
    EXTREME_RIGHT,
    END
} LinePosition;

class Sensor {
public:
    uint8_t sensorValue;
    uint16_t threshold;

    explicit Sensor(const uint16_t initialThreshold = 250) : sensorValue(0), threshold(initialThreshold) {
    }

    void calibrate() {
        uint16_t maxRead = 0, minRead = 1023;
        for (const uint8_t Pin : IRPins) {
            uint16_t value = analogRead(Pin);
            maxRead = max(maxRead, value);
            minRead = min(minRead, value);
        }
        threshold = (maxRead + minRead + 2 * threshold) / 4;
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

    LinePosition getLinePosition() const {
        constexpr uint8_t centerMask = 0b00011000;
        constexpr uint8_t extremeMask = 0b10000001;
        constexpr uint8_t leftMask = 0b00001110;
        constexpr uint8_t rightMask = 0b01110000;

        uint8_t countOnes = 0;
        for (uint8_t i = 0; i < 8; i++) {
            if (sensorValue & (1 << i)) countOnes++;
        }

        if (countOnes >= 6) return END;
        if (countOnes == 0) return BLANC;
        if ((sensorValue & centerMask) && !(sensorValue & ~centerMask & ~extremeMask)) return CENTER;
        if ((sensorValue & leftMask) && (sensorValue & 0b10000000)) return EXTREME_LEFT;
        if ((sensorValue & rightMask) && (sensorValue & 0b00000001)) return EXTREME_RIGHT;
        if (sensorValue & leftMask) return LEFT;
        if (sensorValue & rightMask) return RIGHT;

        return BLANC;
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

constexpr uint16_t Speed = 200;
constexpr uint8_t LostLineLimit = 100;
uint16_t lostLineCounter = 0;

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

    for (uint16_t i = 0; i < 25000; i++) {
        irSensor.calibrate();
        if (i % 512 == 0) {
            digitalWrite(LED_BUILTIN, HIGH);
        } else {
            digitalWrite(LED_BUILTIN, LOW);
        }
    }

    digitalWrite(LED_BUILTIN, LOW);
    Serial.print("Calibrated threshold: ");
    Serial.println(irSensor.threshold);
}

void loop() {
    irSensor.read(3);
    motorA.setDirection(FORWARD);
    motorB.setDirection(FORWARD);

    switch (irSensor.getLinePosition()) {
        case END:
            motorA.setDirection(STOP);
            motorB.setDirection(STOP);
            break;
        case CENTER:
            motorA.setSpeed(Speed);
            motorB.setSpeed(Speed);
            break;
        case LEFT:
            motorA.setSpeed(Speed * 0.8);
            motorB.setSpeed(Speed * 1.1);
            break;
        case RIGHT:
            motorA.setSpeed(Speed * 1.1);
            motorB.setSpeed(Speed * 0.8);
            break;
        case EXTREME_LEFT:
            motorA.setSpeed(Speed * 0.5);
            motorB.setSpeed(Speed * 0.5);
            motorA.setDirection(BACKWARD);
            motorB.setDirection(FORWARD);
            break;
        case EXTREME_RIGHT:
            motorA.setSpeed(Speed * 0.5);
            motorB.setSpeed(Speed * 0.5);
            motorA.setDirection(FORWARD);
            motorB.setDirection(BACKWARD);
            break;
        case BLANC:
            motorA.setSpeed(Speed * 0.5);
            motorB.setSpeed(Speed * 0.5);
            lostLineCounter++;
            if (lostLineCounter > LostLineLimit) {
                motorA.setDirection(STOP);
                motorB.setDirection(STOP);
            }
    }
}
