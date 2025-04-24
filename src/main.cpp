#include Arduino.h


 IR Sensor
constexpr uint8_t NumPins = 8;
constexpr uint8_t IRPins[NumPins] = {A7, A6, A5, A4, A3, A2, A1, A0};

 Motor A (RIGHT)
constexpr uint8_t ForWordPinA = 9;
constexpr uint8_t ReversePinA = 10;

 Motor B (LEFT)
constexpr uint8_t ForWordPinB =  5;
constexpr uint8_t ReversePinB =  6;

Motor regulation
constexpr int16_t BaseSpeed = 150;
constexpr int16_t MaxSpeed = 255;
constexpr int16_t MinSpeed = -255;
constexpr uint8_t ErrorMax = 5;
constexpr uint8_t DecreaseSpeed = 10;

Line Values Special Case
constexpr uint8_t BlackCase = 0xEE;
constexpr uint8_t WhiteCase = 0xFF;
constexpr uint8_t DoubleLineCase = 0xEF;
constexpr uint8_t OtherCase = 0xAA;
constexpr uint8_t LeftCase = 0xF0;
constexpr uint8_t RightCase = 0x0F;




constexpr uint32_t CalibrationTime = 5000;

bool ErrorDetected = false;
bool first = true;


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
public

    uint8_t sensorValue = 0b00000000;
    uint16_t threshold[NumPins] = {512};
    uint16_t interMatrix = 0b000000000 ;

    Sensor(){
      pinMode(LED_BUILTIN, OUTPUT);
      for(const uint8_t IRPin  IRPins) {
        pinMode(IRPin, INPUT);
      }
    }

    void calibrate(const uint32_t CalibrationTime = CalibrationTime) {
      This Function calibrate the threshold
        uint16_t maxRead[NumPins] = {0};
        uint16_t minRead[NumPins] = {1023};
        const uint32_t CalibrationStart = millis();
        uint16_t value = 0;

        while ((millis() - CalibrationStart)  CalibrationTime) {
            for (uint8_t i = 0; i  NumPins; i++) {
                value = analogRead(IRPins[i]);
                maxRead[i] = max(maxRead[i], value);
                minRead[i] = min(minRead[i], value);
            }
        }

        for (uint8_t i = 0; i  NumPins; i++) {
            threshold[i] = (2  maxRead[i] + minRead[i])  3;
        }
        return ;
    }

    void read(const uint8_t n = 3) {
        This Function reads the current value of the IRsensor by calculating the mean value among n values
        sensorValue = 0b00000000;

        uint8_t voteCount[NumPins] = {0};

        for (uint8_t j = 0; j  n; j++) {
            for (uint8_t i = 0; i  NumPins; i++) {
                if (static_castuint16_t(analogRead(IRPins[i]))  threshold[i]) {
                    voteCount[i]++;
                }
            }
        }

        for (uint8_t i = 0; i  NumPins; i++) {
            if (voteCount[i]  (n  2)) {
                sensorValue = (1  i);
            }
        }
    }

    void readv2() {
        This Function reads the current value of the IRsensor by calculating the mean value among n values
        sensorValue = 0b00000000;

        for (uint8_t i = 0; i  NumPins; i++) {
            if (static_castuint16_t(analogRead(IRPins[i]))  threshold[i]) {
                sensorValue = (1  i);
            }
        }
    }

    void update(const uint8_t n) {
      the hell is this
        if (n  0) {
            read(n);
        } else {
            for (uint8_t i = 0; i  NumPins; i++) {
                if (static_castuint16_t(analogRead(IRPins[i]))  threshold[i]) {
                    sensorValue = (1  i);
                }
            }
        }

        bool LastBit = (sensorValue & (1  0)) != 0;
        uint8_t pos = 1;
        uint8_t MappedValue = 0b000  LastBit;
        const uint8_t LastMapped =  interMatrix & 0b111;

        for (uint8_t i = 1; i  NumPins; i++) {
            const bool Bit = (sensorValue & (1  i)) != 0;
            if (Bit != LastBit) {
                MappedValue = (Bit) (1  pos++)  0b000;
                LastBit = Bit;
            }
            if (pos == 3) break;
        }

        if (pos == 1) {
            if (LastBit) MappedValue = 0b111;
        } else if (pos == 2) {
            if (LastBit) MappedValue = 1  pos;
        }

         Need to verify Logic (it work maybe)
        if (MappedValue != LastMapped) {
            if (LastMapped == 0b011)
            {
                if (MappedValue & 0b100)
                    interMatrix = 0b1111;
                else
                    interMatrix = (interMatrix3)  MappedValue;
            } else if (LastMapped == 0b110) {
                if (MappedValue & 0b001)
                    interMatrix = 0b111;
                else
                    interMatrix = (interMatrix3)  MappedValue;
            } else if (LastMapped == 0b111) {
                if (MappedValue & 0b101 != 0)
                    interMatrix = MappedValue;
                else
                    interMatrix = (interMatrix3)  MappedValue;
            } else {
                interMatrix = (interMatrix3)  MappedValue;
            }


        }
        interMatrix = interMatrix & 0b111111111;
    }


    uint8_t getLinePosition() const {
        This function approximates the position of the line
        uint8_t countOnes = 0;

        for (uint8_t i = 0; i  8; i++) {
            if (sensorValue & (1  i)) countOnes++;
        }

        switch (countOnes) {
            case 0
                return WhiteCase;
            case 1
                for (uint8_t i = 0; i  8; i++) {
                    if (sensorValue == (0b1  i)) return (2  i);
                }
            case 2
                for (uint8_t i = 0; i  7; i++) {
                    if (sensorValue == (0b11  i)) return (2  i + 1);
                }
                return DoubleLineCase;
            case 3
                for (uint8_t i = 0; i  6; i++) {
                    if (sensorValue == (0b111  i)) return (2  i + 2);
                } break;
            case 4
                for (uint8_t i = 0; i  5; i++) {
                    if (sensorValue == (0b1111  i)) {
                        if (i == 0) return  RightCase;
                        if (i == 5) return  LeftCase;
                        return (2  i + 3);
                    }
                } break;
            case 5
                for (uint8_t i = 0; i  4; i++) {
                    if (sensorValue == (0b11111  i)) {
                        if (i == 0) return  RightCase;
                        if (i == 4) return  LeftCase;
                        return (2  i + 4);
                    }
                } break;
            case 6
                for (uint8_t i = 0; i  3; i++) {
                    if (sensorValue == (0b111111  i)) return (2  i + 5);
                } break;
            case 7
                for (uint8_t i = 0; i  2; i++) {
                    if (sensorValue == (0b1111111  i)) return (2  i + 6);
                } break;
            case 8
                return BlackCase;
            default return OtherCase;
        }

        return OtherCase;
    }


};

class Motor {
public
    const uint8_t forWordPin, reversePin;
    int16_t speed = static_castint16_t(BaseSpeed);

    Motor(const uint8_t forward, const uint8_t reverse)  forWordPin(forward), reversePin(reverse) {
        pinMode(forWordPin, OUTPUT);
        pinMode(reversePin, OUTPUT);
    }

    void setDirection(const Direction dir) const {
        switch (dir) {
            case FORWARD
                analogWrite(forWordPin, speed);
                analogWrite(reversePin, 0);
                break;
            case BACKWARD
                analogWrite(forWordPin, 0);
                analogWrite(reversePin, speed);
                break;
            case STOP
                analogWrite(forWordPin, HIGH);
                analogWrite(reversePin, HIGH);
                break;
            default
              break;
        }

    }

    void setSpeed(const int16_t v) {
        speed = v;
    }

    void drive(int16_t v) {

        if (v == 0) {
            setSpeed(v);
            analogWrite(forWordPin, HIGH);
            analogWrite(reversePin, HIGH);
            return;
        }
        if (v  0) {
            if (v  MaxSpeed)v = MaxSpeed;
            setSpeed(v);
            analogWrite(forWordPin, speed);
            analogWrite(reversePin, 0);
            return;
        }
        if (v  MinSpeed) v = MinSpeed;
        setSpeed(-v);
        analogWrite(forWordPin, 0);
        analogWrite(reversePin, speed);
    }

    int16_t getSpeed() const {
        return speed;
    }
};


class PID {
    public
        const float Kp, Fi, Td;
        int8_t PreviousError = 0;
        int64_t IntegralError = 0;
        const int8_t TargetPosition = 7;

    PID(const float Kp, const float Fi, const float Td)  Kp(Kp), Fi(Fi), Td(Td) {};

    void Regulate(const uint8_t pos, Motor M1, Motor M2) {
        const auto Error = static_castint8_t(TargetPosition - pos);

        const auto SpeedControl = static_castint16_t(Kp(1 + Fi + Td)  Error + IntegralError - Kp  Td  PreviousError);
        const auto SpeedControl = static_castint16_t(Kp(1 + Td)  Error - Kp  Td  PreviousError);
        const auto Speed = BaseSpeed - abs(Error)DecreaseSpeed;
        const int16_t speed1 = Speed - SpeedControl, speed2 = Speed + SpeedControl;
        const auto Speed = BaseSpeed - abs(Error)15;

        if (abs(Error)  ErrorMax) {
             M1.setSpeed(constrain(BaseSpeed - SpeedControl, MinSpeed, MaxSpeed));
             M2.setSpeed(constrain(BaseSpeed + SpeedControl, MinSpeed, MaxSpeed));
        }
        else {
            M1.setSpeed(constrain(0.4  BaseSpeed - SpeedControl, MinSpeed, MaxSpeed));
            M2.setSpeed(constrain(0.4  BaseSpeed + SpeedControl, MinSpeed, MaxSpeed));
        }

        M1.drive(speed1);
        M2.drive(speed2);

        IntegralError += static_castint64_t(KpFiError);
        PreviousError = Error;
    }
};


Motor MotorA( ForWordPinA, ReversePinA);
Motor MotorB(ForWordPinB, ReversePinB);
Sensor IRSensor;
PID MyPID(65.0, 0.0, 0.1);



void ForewordPID(const uint8_t pos) {
    int8_t Error = static_castint8_t(TargetPosition - pos);

    int16_t SpeedControl = (Kp + Ki + Kd)  Error + IntegralError - Kd  PreviousError;
    MotorA.setSpeed(constrain(Speed - SpeedControl, MinSpeed, MaxSpeed));
    MotorB.setSpeed(constrain(Speed + SpeedControl, MinSpeed, MaxSpeed));

    MotorA.setDirection(FORWARD);
    MotorB.setDirection(FORWARD);

    IntegralError += KiError;
    PreviousError = Error;

    return;
}


 need to be calibrated
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
    MotorA.setDirection(STOP);
    MotorB.setDirection(STOP);
}

void TurnRight() {
    uint8_t pos;
    do {
        IRSensor.readv2();
        pos = IRSensor.getLinePosition();
        MotorA.drive(-255);
        MotorB.drive(255);

    } while (pos != 3);
}

void TurnLeft() {
    uint8_t pos;
    do {
        IRSensor.readv2();
        pos = IRSensor.getLinePosition();
        MotorA.drive(255);
        MotorB.drive(-255);

    } while (pos != 11);
}
void forward() {
    MotorA.drive(BaseSpeed);
    MotorB.drive(BaseSpeed);
}
void setup() {
    Serial.begin(9600);


    MotorA.setDirection(STOP);
    MotorB.setDirection(STOP);
    MotorA.setSpeed(Speed);
    MotorB.setSpeed(Speed);

    Calibration
    digitalWrite(LED_BUILTIN, HIGH);
    IRSensor.calibrate();
    digitalWrite(LED_BUILTIN, LOW);




    
    Serial.print(Calibrated threshold );
    for (const uint16_t threshold IRSensor.threshold) {
        Serial.print(threshold);
        Serial.print( );
    }
    Serial.println();
    delay(1000);
    

}


bool Turns[5] = {true, false, true, false, true};
uint8_t counter = 0;
void loop() {
    IRSensor.readv2();
    uint8_t pos = IRSensor.getLinePosition();

    if(pos == WhiteCase) return;
    
    if(inter) {
        switch (IRSensor.interMatrix) {
            case UP_LEFT_RIGHT TurnRight(); break;
            case LEFT_RIGHT TurnRight(); break;
            case UP_RIGHT TurnRight(); break;
            case UP_LEFT  TurnLeft(); break;
            case LEFT_TURN TurnLeft(); break;
            case RIGHT_TURN TurnRight(); break;
        }
        inter = false;
    } else {
        if (pos == BlackCase){
            StopMotor();
            delay(20);
            IRSensor.readv2();
            pos = IRSensor.getLinePosition();
            if (pos == WhiteCase) {
                if (Turns[counter]) {
                    forward();
                    delay(100);
                    StopMotor();
                    delay(20);
                    TurnRight();   Priority Right
                } else {
                    forward();
                    delay(100);
                    StopMotor();
                    delay(20);
                    TurnLeft();   Priority Left
                }
                counter = (counter + 1 ) % 5;
            } else if (pos == BlackCase) {
                StopMotor();
            }
        } else if (pos == WhiteCase) {
            return;
        } else if (pos == RightCase) {
            forward();
            delay(100);
            StopMotor();
            IRSensor.readv2();
            pos = IRSensor.getLinePosition();
            if (pos == WhiteCase  pos == RightCase) {
                delay(20);
                TurnRight();
            }
        } else if (pos == LeftCase) {
            forward();
            delay(100);
            StopMotor();
            IRSensor.readv2();
            pos = IRSensor.getLinePosition();
            if (pos == WhiteCase  pos == LeftCase) {
                delay(20);
                TurnLeft();
            }
        } else if (pos == OtherCase) {
            return;
        } else MyPID.Regulate(pos, MotorA, MotorB);
        first = true;
    }

    
    else if (first) {
        first = false;
         MotorA.setDirection(STOP);
         MotorB.setDirection(STOP);

        MotorA.setSpeed(constrain(MotorA.getSpeed() - 20, MinSpeed, MaxSpeed));
        MotorB.setSpeed(constrain(MotorB.getSpeed() - 20, MinSpeed, MaxSpeed));
        MotorA.setDirection(FORWARD);
        MotorB.setDirection(FORWARD);
    } else {
        MotorA.setDirection(FORWARD);
        MotorB.setDirection(FORWARD);
    }
