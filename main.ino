#include <Arduino.h>

// Define the Motor struct
struct Motor {
    int inputPin1;                // Input 1 on the L298N
    int inputPin2;                // Input 2 on the L298N
    int enablePin;                // Enable (PWM) pin on the L298N (optional)
    bool isClockwise = true;      // Boolean indicating the rotation direction
    int rotationSpeed = 255;      // Motor rotation speed (0-255)

    // Constructor to initialize the motor with an optional enable pin
    Motor(int inPin1, int inPin2, int enPin = -1, int speed = 255)
        : inputPin1(inPin1), inputPin2(inPin2), enablePin(enPin), rotationSpeed(speed) {
        pinMode(inputPin1, OUTPUT);
        pinMode(inputPin2, OUTPUT);
        if (enablePin != -1) {
            pinMode(enablePin, OUTPUT);
        }
    }

    // Method to rotate the motor in the specified direction
    void rotate() {
        if (isClockwise) {
            digitalWrite(inputPin1, LOW);
            digitalWrite(inputPin2, HIGH);
        } else {
            digitalWrite(inputPin1, HIGH);
            digitalWrite(inputPin2, LOW);
        }
    }

    // Method to set the motor speed (if PWM is used)
    void setSpeed(int speed) {
        rotationSpeed = constrain(speed, 0, 255);
        if (enablePin != -1) {
            analogWrite(enablePin, rotationSpeed);
        }
    }

    // Method to stop the motor
    void stop() {
        digitalWrite(inputPin1, LOW);
        digitalWrite(inputPin2, LOW);
        if (enablePin != -1) {
            analogWrite(enablePin, 0); // Set PWM to 0 for a complete stop
        }
    }
};

// Define the Potentiometer struct
struct Potentiometer {
    int analogInputPin; // Input pin on the Arduino board

    Potentiometer(int outPin) : analogInputPin(outPin) {
        pinMode(analogInputPin, INPUT);
    }

    // Method to read potentiometer value
    // Note: The reading of the potentiometer increases in the anti-clockwise direction
    float readValue() {
        int potValue = analogRead(analogInputPin);
        return map(potValue, 0, 1023, 100.0, 0.0);
    }

    // Method to print potentiometer data to Serial
    void printData() {
        Serial.print("Percentage: ");
        Serial.print(readValue());
        Serial.println("%");
    }
};

struct ProgramTimer {
public:
    unsigned long timerInterval = 0;
    unsigned long currentMillis;
    unsigned long lastMillis;
    unsigned long elapsedIntervals = 0;

    ProgramTimer() {
        lastMillis = millis();
    }

    void reset () {
        lastMillis = millis();
        elapsedIntervals = 0;
    }

    // Method to check if the specified timer interval has elapsed
    bool interval() {
        if (timerInterval != 0) {
            currentMillis = millis();
            if (currentMillis - lastMillis >= timerInterval) {
                elapsedIntervals += (currentMillis - lastMillis) / timerInterval;
                lastMillis = currentMillis;
                return true;
            } else {
                return false;
            }
        }
    }
};

// Define the RotationSpeedPair struct
struct RotationSpeedPair {
    float rotationGap;  // Use float for rotationGap
    int motorSpeed;
};

// Define the RotationSpeedTable struct
struct RotationSpeedTable {
private:
    RotationSpeedPair* table;  // Pointer to an array of RotationSpeedPair
    int tableSize;             // Size of the array
    bool interpolate = true;   // Determines if linear interpolation between intervals is used

public:
    // Constructor to initialize the RotationSpeedTable with an array
    RotationSpeedTable(RotationSpeedPair* initialTable, int size, bool interpolate = true)
        : table(new RotationSpeedPair[size]), tableSize(size), interpolate(interpolate) {
        // Copy the values from the initial array to the table
        for (int i = 0; i < size; ++i) {
            table[i] = initialTable[i];
        }
    }

    // Method to get the motor speed based on the rotation gap
    float getSpeed(float rotationGap) {
        int index = 0;
        float calculatedSpeed;

        for (int i = 0; i < tableSize; ++i) {
            if (rotationGap >= table[i].rotationGap) {
                index = i;
                if (i + 1 < tableSize - 1) {
                    break;
                }
            }
        }

        if (interpolate) {
            // Calculation for linear interpolation
            float gapDifference = table[index + 1].rotationGap - table[index].rotationGap;
            int speedDifference = table[index + 1].motorSpeed - table[index].motorSpeed;

            float gapRatio = (rotationGap - table[index].rotationGap) / gapDifference;

            calculatedSpeed = gapRatio * speedDifference + table[index].motorSpeed;
        } else {
            calculatedSpeed = table[index].motorSpeed;
        }

        return calculatedSpeed;
    }

    // Destructor to free the memory allocated for the array
    ~RotationSpeedTable() {
        delete[] table;
    }
};

// Define the parameters struct
struct MotorControlParameters {
    int minPotValue;
    int maxPotValue;
    float rotationUncertainty;
    int maxLockDuration; // -1 for disable
    int speedBoostAmount; // -1 for disable
    bool isDecaying;
    int maxPrecisionResets; // -1 for disable
    int motorEffectiveRPM;
};

// Define the ControlledMotor class
class ControlledMotor {
public:
    Motor motor;                             // Motor instance
    Potentiometer pot;                       // Potentiometer instance
    RotationSpeedTable speedTable;           // SpeedTable instance
    ProgramTimer lockUpTimer;                      // ProgramTimer instance
    ProgramTimer preciseRotationTimer;

    // User-changeable parameters
    int minPotValue;
    int maxPotValue;
    float rotationUncertainty;
    int maxLockDuration;
    int speedBoostAmount;
    bool isDecaying;
    int maxPrecisionResets;
    int motorEffectiveRPM;

    bool setAngleRotation = false;
    float targetValue = 50.0;               // Desired potentiometer value for motor rotation
    bool isMoving = false;
    bool motorSet = false;
    int lockTime = 0;
    int lastValue = -1;
    int currentValue = -1;
    bool isBoosting = false;
    bool rotationCheck = false;
    int precisionResets = 0;

    // Modified constructor to include RotationSpeedTable parameter and user-changeable parameters
    ControlledMotor(const Motor& m, const Potentiometer& p, const RotationSpeedTable& st, const MotorControlParameters& params)
        : motor(m), pot(p), speedTable(st), minPotValue(params.minPotValue), maxPotValue(params.maxPotValue),
          rotationUncertainty(params.rotationUncertainty), maxLockDuration(params.maxLockDuration),
          speedBoostAmount(params.speedBoostAmount), isDecaying(params.isDecaying), maxPrecisionResets(params.maxPrecisionResets), motorEffectiveRPM(params.maxPrecisionResets) {
            
            lockUpTimer.timerInterval = 250;
            preciseRotationTimer.timerInterval= 500;
    }

    // Method to check if the value is within the uncertainty range
    bool isOutsideRange(float value, float desired, float uncertainty) {
        return (value < desired - uncertainty) || (value > desired + uncertainty);
    }

    // Method to set the motor rotation to a specified angle (percentage)
    void setRotation(float angle) {
        motorSet = false;
        setAngleRotation = true;
        targetValue = constrain(angle, minPotValue, maxPotValue);
        float value = pot.readValue();
        motor.isClockwise = value < targetValue;

        if (isOutsideRange(value, targetValue, rotationUncertainty)) {
            isMoving = true;
        }
    }

    void stopMotor() {
        motor.stop();
        isMoving = false;
        setAngleRotation = false;
        precisionResets = 0;
        lockTime = 0;
        lastValue = -1;
    }

    void startMotor() {
        motor.rotate();
        motorSet = true;
    }

    // Overloaded method to set motor rotation direction without specifying an angle
    void setRotation(bool clockwise) {
        motorSet = false;
        setAngleRotation = false;

        motor.isClockwise = clockwise;
        float value = pot.readValue();

        if (clockwise) {
            targetValue = maxPotValue;
            if (value < targetValue) {
                isMoving = true;
            }
        } else {
            targetValue = minPotValue;
            if (value > targetValue) {
                isMoving = true; 
            }
        }
    }

    // Main method to control motor rotation
    void rotate() {
        if (isMoving) {
            if (!motorSet) {
                startMotor();
            }
            // Check for lockups / motor 'catching'
            if (!rotationCheck) {
                if (maxLockDuration != -1 || speedBoostAmount != -1) {
                    if (lastValue == -1) {
                        lastValue = pot.readValue();
                    } else {
                        if (lockUpTimer.interval()) {
                            currentValue = pot.readValue();

                            float delta = abs(currentValue - lastValue);
                            float rateOfChange = 1000 * delta / lockUpTimer.timerInterval; 
    
                            // expectedChange = (100.0f*motor.rotationSpeed/255.0f) * (motorEffectiveRPM/60) * (3.0f/4.0f);
                            float expectedChange = (motor.rotationSpeed * motorEffectiveRPM / 204.0f);

                            float changeRatio = rateOfChange / expectedChange;

                            if (changeRatio <= 0.1f) {
                                if (maxLockDuration != -1) {
                                    lockTime += lockUpTimer.timerInterval;
                                    if (lockTime >= maxLockDuration * (1.5f - motor.rotationSpeed / 255.0f)) {
                                        stopMotor();
                                        Serial.println("Motor stopped due to lock-up");
                                    }
                                }
                            }

                            if (changeRatio <= 0.5f) {
                                if (speedBoostAmount != -1) {
                                    isBoosting = true;
                                }
                            } else {
                                lockTime = 0;
                                isBoosting = false;
                            }

                            lastValue = currentValue;
                        }
                    }
                }
            }
            // Decay the speed of rotation using PWM
            if (isDecaying) {
                float difference = abs(pot.readValue() - targetValue);
                int speed = speedTable.getSpeed(difference);
                if (isBoosting) {
                    speed += speedBoostAmount;
                }
                motor.setSpeed(speed);
            }

            // Check if the motor has reached rotation limits
            float value = pot.readValue();
            if (((value <= minPotValue) && !motor.isClockwise) || ((value >= maxPotValue) && motor.isClockwise)) {
                stopMotor();
                Serial.println(value);
                //Serial.println("Rotation limit exceeded");
            }

            // Check if the motor is within the accepted range
            if (setAngleRotation && !rotationCheck) {
                if (!isOutsideRange(value, targetValue, rotationUncertainty)) {
                    if (maxPrecisionResets != -1 && precisionResets != maxPrecisionResets) {
                        motor.stop();
                        preciseRotationTimer.reset();
                        rotationCheck = true;
                    }
                    else {
                        stopMotor();
                    }
                }
            }
            if (maxPrecisionResets != -1) {
                if (rotationCheck == true) {
                    if (preciseRotationTimer.interval()) {
                        
                        float delayedValue = pot.readValue();
                        if (isOutsideRange(delayedValue, targetValue, rotationUncertainty)) {
                            setRotation(targetValue);
                            precisionResets++;
                        } else {
                            stopMotor();
                        }
                        rotationCheck = false;
                    }
                }
            }
        }
    }
};

void clearSerialMonitor() {
    Serial.write("\033[2J\033[H"); // ANSI escape code to clear the screen and move cursor to (0,0)
}

struct Button {

    int digitalInputPin;
    ProgramTimer debounceTimer;
    ProgramTimer pressTimer;
    bool currentState = LOW;
    bool lastState = LOW;
    bool outputState = LOW;

    // Constructor to initialize the Button
    Button(int digitalInputPin) : digitalInputPin(digitalInputPin) {
        pinMode(digitalInputPin, INPUT);
        debounceTimer.timerInterval = 10;
        pressTimer.timerInterval = 5;
    }

    // Method to read the button state and handle debouncing
    bool readButton() {

        currentState = digitalRead(digitalInputPin);

        // Check if the button state has changed
        if (currentState != lastState) {
            // Button is released (transition from HIGH to LOW)
            if (currentState == LOW) {
                debounceTimer.reset(); // Reset the timer when button is released
                lastState = currentState;
                outputState = LOW;
            }
            // Button is pressed (transition from LOW to HIGH)
            else if (currentState == HIGH) {
                // Check if the timer interval has elapsed (debounce)
                if (debounceTimer.interval()) {
                    pressTimer.reset();
                    lastState = currentState;
                    outputState = HIGH;
                }
            }
        }
        return outputState;
    }

    // Returns the time the button has been pressed for in milliseconds
    int getPressDuration() {
        if (outputState == LOW) {
            return 0;
        } else if (outputState == HIGH) {
            pressTimer.interval();
            return pressTimer.elapsedIntervals * pressTimer.timerInterval;
        }
    }
};

class Wrist {
private:
  Button input1 = Button(2); // leftmost button
  Button input2 = Button(3); // rightmost button

  bool lastb1State = false;
  bool lastb2State = false;

  Motor motor1{8, 9, 6}; // In a real-life circuit, even numbers on Arduino and L298N are connected
  Potentiometer pot1{A0};

  RotationSpeedPair speedPairs[4] = {
      {0, 255},
      {40, 255},
      {80, 255},
      {100, 255},
  };

  int numberOfPairs = sizeof(speedPairs) / sizeof(speedPairs[0]);
  RotationSpeedTable speedTable{speedPairs, numberOfPairs, true};
  MotorControlParameters motorParams = {
      .minPotValue = 5,
      .maxPotValue = 95,
      .rotationUncertainty = 0.0f,
      .maxLockDuration = -1,
      .speedBoostAmount = 10,
      .isDecaying = true,
      .maxPrecisionResets = 3,
      .motorEffectiveRPM = 30,
  };

  ControlledMotor motor{motor1, pot1, speedTable, motorParams};

public:
  void loop() {
    pot1.printData();

    bool b1state = input1.readButton();
    bool b2state = input2.readButton();
    int b1duration = input1.getPressDuration();
    int b2duration = input2.getPressDuration();

    if (((b1state != lastb1State) || (b2state != lastb2State)) && !motor.setAngleRotation) {
      if (b1state == HIGH && b2state == LOW) {
        Serial.println("AntiClockwise");
        motor.setRotation(false);
      }
      if (b1state == LOW && b2state == HIGH) {
        Serial.println("Clockwise");
        motor.setRotation(true);
      }
      if (b1state == b2state) {
        Serial.println("Low");
        motor.stopMotor();
      }
      lastb1State = b1state;
      lastb2State = b2state;
    }

    if (b1state == HIGH && b2state == HIGH && !motor.setAngleRotation) {
      int holdDuration = 4000;
      if (b1duration >= holdDuration && b2duration >= holdDuration) {
        float normalAngle = 50.0f;
        motor.setRotation(normalAngle);
        Serial.println("Set Rotation");
      }
    }
    motor.rotate();
  }
};

class Hand {
private:
  Button input3 = Button(2); // leftmost button
  Button input4 = Button(3); // rightmost button

  bool lastb3State = false;
  bool lastb4State = false;

  bool latch = false;
  bool opening = true;

  Motor motor2{8, 9, 6}; // In a real-life circuit, even numbers on Arduino and L298N are connected
  Potentiometer pot2{A0};

  RotationSpeedPair speedPairs[4] = {
      {0, 255},
      {40, 255},
      {80, 255},
      {100, 255},
  };

  int numberOfPairs = sizeof(speedPairs) / sizeof(speedPairs[0]);

  RotationSpeedTable speedTable{speedPairs, numberOfPairs, true};
  MotorControlParameters motorParams = {
      .minPotValue = 5,
      .maxPotValue = 95,
      .rotationUncertainty = 0.0f,
      .maxLockDuration = 4000,
      .speedBoostAmount = 10,
      .isDecaying = true,
      .maxPrecisionResets = 3,
      .motorEffectiveRPM = 30,
  };

  ControlledMotor motor{motor2, pot2, speedTable, motorParams};

public:
  void loop() {
    bool b3state = input3.readButton();
    bool b4state = input4.readButton();
    int b3duration = input3.getPressDuration();
    int b4duration = input4.getPressDuration();

    if (b4state != lastb4State) {
      if (b4state && !opening) {
        latch = !latch;
      }
      lastb4State = b4state;
    }

    if (b3state != lastb3State) {
      if (!latch) {
        if (!b3state) {
          float openFingersAngle = 25.0f;
          motor.stopMotor();
          motor.setRotation(openFingersAngle);
          opening = true;
        } else {
          motor.stopMotor();
          motor.setRotation(true);
          opening = false;
        }
      }
      lastb3State = b3state;
    }
    if (!opening) {
      if (latch) {
        motor.stopMotor();
      }
    }

    motor.rotate();
  }
};

void setup() {
  // Setup code goes here
  Serial.begin(115200);
}

Hand hand;
//Wrist wrist;

void loop() {
  hand.loop();
  //wrist.loop();
}