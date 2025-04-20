#include <Arduino.h>

// IR Sensor
constexpr uint8_t NumPins = 8;
constexpr uint8_t IRPins[NumPins] = {A7, A6, A5, A4, A3, A2, A1, A0};

// Motor A (RIGHT)
constexpr uint8_t ForWordPinA = 9;
constexpr uint8_t ReversePinA = 10;

// Motor B (LEFT)
constexpr uint8_t ForWordPinB =  5;
constexpr uint8_t ReversePinB =  6;

//PID K's
constexpr int8_t Ki = 0;
constexpr int8_t Kp = 10;
constexpr int8_t Kd = 2;

//Motor regulation
constexpr uint8_t TargetPosition = 7;
constexpr uint8_t Speed = 140;
constexpr uint8_t MaxSpeed = 255;
constexpr uint8_t MinSpeed = 140;


constexpr uint32_t CalibrationTime = 5000;

uint8_t LastPosition = 0;
int8_t SpeedControl = 0;

int8_t Error = 0;
int8_t PreviousError = 0;
int8_t IntegralError = 0;
int8_t DerivativeError = 0;

uint8_t MotorASpeed = Speed;
uint8_t MotorBSpeed = Speed;

bool ErrorDetected = false;

typedef enum {
    FORWARD,
    BACKWARD,
    STOP
} Direction;

typedef enum {
    LEFT_TURN     = 0b010110000,
    RIGHT_TURN    = 0b010011000,
    LEFT_RIGHT    = 0b010111000,
    UP_LEFT       = 0b010110010,
    UP_RIGHT      = 0b010011010,
    UP_LEFT_RIGHT = 0b010111010,
} Intersection;

class Sensor {
public:

    uint8_t sensorValue = 0b00000000;
    uint16_t threshold[NumPins] = {512};
    uint16_t interMatrix = 0b000000000 ;

    Sensor(){
      pinMode(LED_BUILTIN, OUTPUT);

      for(const uint8_t IRPin : IRPins) {
        pinMode(IRPin, INPUT);
      }
    }

    void calibrate(const uint32_t CalibrationTime = CalibrationTime) {
      //This Function calibrate the threshold
        uint16_t maxRead[NumPins] = {0};
        uint16_t minRead[NumPins] = {1023};
        const uint32_t CalibrationStart = millis();
        uint16_t value = 0;

        while ((millis() - CalibrationStart) < CalibrationTime) {
            for (uint8_t i = 0; i < NumPins; i++) {
                value = analogRead(IRPins[i]);
                maxRead[i] = max(maxRead[i], value);
                minRead[i] = min(minRead[i], value);
            }
        }

        for (uint8_t i = 0; i < NumPins; i++) {
            threshold[i] = (2 * maxRead[i] + minRead[i]) / 3;
        }

        return ;
    }

    void read(const uint8_t n = 3) {
        //This Function reads the current value of the IRsensor by calculating the mean value among n values

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

        return ;
    }

    void update(const uint8_t n) {
      //the hell is this
        if (n > 0) {
            read(n);
        } else {
            for (uint8_t i = 0; i < NumPins; i++) {
                if (static_cast<uint16_t>(analogRead(IRPins[i])) > threshold[i]) {
                    sensorValue |= (1 << i);
                }
            }
        }

        bool LastBit = (sensorValue & (1 << 0)) != 0;
        uint8_t pos = 1;
        uint8_t MappedValue = 0b000 | LastBit;
        const uint8_t LastMapped =  interMatrix & 0b111;

        for (uint8_t i = 1; i < NumPins; i++) {
            const bool Bit = (sensorValue & (1 << i)) != 0;
            if (Bit != LastBit) {
                MappedValue |= (Bit)? (1 << pos++) : 0b000;
                LastBit = Bit;
            }
            if (pos == 3) break;
        }

        if (pos == 1) {
            if (LastBit) MappedValue = 0b111;
        } else if (pos == 2) {
            if (LastBit) MappedValue |= 1 << pos;
        }

        // Need to verify Logic (it work maybe)
        if (MappedValue != LastMapped) {
            if (LastMapped == 0b011)
            {
                if (MappedValue & 0b100)
                    interMatrix |= 0b1111;
                else
                    interMatrix = (interMatrix<<3) | MappedValue;
            } else if (LastMapped == 0b110) {
                if (MappedValue & 0b001)
                    interMatrix |= 0b111;
                else
                    interMatrix = (interMatrix<<3) | MappedValue;
            } else if (LastMapped == 0b111) {
                if (MappedValue & 0b101 != 0)
                    interMatrix |= MappedValue;
                else
                    interMatrix = (interMatrix<<3) | MappedValue;
            } else {
                interMatrix = (interMatrix<<3) | MappedValue;
            }


        }
        interMatrix = interMatrix & 0b111111111;
    }


    uint8_t getLinePosition() const {
        //This function approximates the position of the line
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
            default: return 7; // Error: two line detected
        }
    }
};

class Motor {
public:
    const uint8_t forWordPin, reversePin;
    uint8_t speed = 0;

    Motor(const uint8_t forward, const uint8_t reverse) : forWordPin(forward), reversePin(reverse) {
        pinMode(forWordPin, OUTPUT);
        pinMode(reversePin, OUTPUT);
    }

    void setDirection(const Direction dir) const {
        switch (dir) {
            case FORWARD:
                analogWrite(forWordPin, speed);
                analogWrite(reversePin, 0);
                break;
            case BACKWARD:
                analogWrite(forWordPin, 0);
                analogWrite(reversePin, speed);
                break;
            case STOP:
                analogWrite(forWordPin, LOW);
                analogWrite(reversePin, LOW);
                break;
            default:
              break;
        }
        return;
    }

    void setSpeed(const uint8_t v) {
        speed = v;
        return ;
    }
};

/*
class PID {
  public:
    explicit PID() = default;
}*/


Motor MotorA( ForWordPinA, ReversePinA);
Motor MotorB(ForWordPinB, ReversePinB);
Sensor IRSensor;

void ForewordPID(const uint8_t pos) {
    Error = static_cast<int8_t>(TargetPosition - pos);
    IntegralError += Error;
    DerivativeError = Error - PreviousError;

    SpeedControl = Kp * Error + Ki * IntegralError + Kd * DerivativeError;
    PreviousError = Error;

    MotorASpeed = constrain(Speed - SpeedControl, MinSpeed, MaxSpeed);
    MotorBSpeed = constrain(Speed + SpeedControl, MinSpeed, MaxSpeed);

    MotorA.setSpeed(MotorASpeed);
    MotorB.setSpeed(MotorBSpeed);
}

// need to be calibrated
void TurnLeft() {
    MotorA.setSpeed(Speed);
    MotorB.setSpeed(Speed);
    MotorA.setDirection(FORWARD);
    MotorB.setDirection(BACKWARD);
    delay(20);
}

void TurnRight() {
    MotorA.setSpeed(Speed);
    MotorB.setSpeed(Speed);
    MotorA.setDirection(BACKWARD);
    MotorB.setDirection(FORWARD);
    delay(20);
}

void StopMotor() {
    MotorA.setDirection(BACKWARD);
    MotorB.setDirection(BACKWARD);
    delay(20);
    MotorA.setDirection(STOP);
    MotorB.setDirection(STOP);
}

void setup() {
    Serial.begin(9600);


    MotorA.setDirection(STOP);
    MotorB.setDirection(STOP);
    //MotorA.setSpeed(Speed);
    //MotorB.setSpeed(Speed);

    //Calibration
    digitalWrite(LED_BUILTIN, HIGH);
    IRSensor.calibrate();
    digitalWrite(LED_BUILTIN, LOW);

    MotorA.setDirection(FORWARD);
    MotorB.setDirection(FORWARD);


    /*
    Serial.print("Calibrated threshold: ");
    for (const uint16_t threshold: IRSensor.threshold) {
        Serial.print(threshold);
        Serial.print(" ");
    }
    Serial.println("");
    delay(1000);
    */

}

void loop() {
    //const uint32_t StartTime = millis();
    IRSensor.read();
    const uint8_t pos = IRSensor.getLinePosition();
    //Serial.println(pos);
    //Serial.println(0x100+IRSensor.sensorValue, BIN);

    /*if (pos == 0xFF) {
        ErrorDetected = true;
    } else {
        if (ErrorDetected) {
            ErrorDetected = false;
            switch (IRSensor.interMatrix) {
                case UP_LEFT_RIGHT:ForewordPID(LastPosition); break;
                case UP_RIGHT: ForewordPID(LastPosition); break;
                case UP_LEFT: ForewordPID(LastPosition); break;
                case LEFT_RIGHT: TurnRight(); break;
                case RIGHT_TURN: TurnRight(); break;
                case LEFT_TURN: TurnLeft(); break;
                default: StopMotor();
            }
            ErrorDetected = false;
        } else {
            ForewordPID(pos);
            LastPosition = pos;
        }
    }

  Serial.println(0x100 + IRSensor.sensorValue, BIN);
  */
    //if (pos != 0xFF)
    ForewordPID(pos);
    analogWrite(9,250);

}
