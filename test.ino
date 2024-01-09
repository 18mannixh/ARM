/*#include <Arduino.h>

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
            digitalWrite(inputPin1, HIGH);
            digitalWrite(inputPin2, LOW);
        } else {
            digitalWrite(inputPin1, LOW);
            digitalWrite(inputPin2, HIGH);
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
      currentMillis = millis();
      lastMillis = currentMillis;
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
    bool preciseRotation;
};

// Define the ControlledMotor class
class ControlledMotor {
private:
    Motor motor;                             // Motor instance
    Potentiometer pot;                       // Potentiometer instance
    RotationSpeedTable speedTable;           // SpeedTable instance
    ProgramTimer timer;                      // ProgramTimer instance

    // User-changeable parameters
    int minPotValue;
    int maxPotValue;
    float rotationUncertainty;
    int maxLockDuration;
    int speedBoostAmount;
    bool isDecaying;
    bool preciseRotation;

    float targetValue = 50.0;               // Desired potentiometer value for motor rotation
    bool isMoving = false;
    int lockTime = 0;
    int lastValue = -1;
    int currentValue = -1;
    bool isBoosting = false;
    bool setAngleRotation = false;

public:
    // Modified constructor to include RotationSpeedTable parameter and user-changeable parameters
    ControlledMotor(const Motor& m, const Potentiometer& p, const RotationSpeedTable& st, const MotorControlParameters& params)
        : motor(m), pot(p), speedTable(st), minPotValue(params.minPotValue), maxPotValue(params.maxPotValue),
          rotationUncertainty(params.rotationUncertainty), maxLockDuration(params.maxLockDuration),
          speedBoostAmount(params.speedBoostAmount), isDecaying(params.isDecaying), preciseRotation(params.preciseRotation) {
    }

    // Method to check if the value is within the uncertainty range
    bool isOutsideRange(float value, float desired, float uncertainty) {
        return (value < desired - uncertainty) || (value > desired + uncertainty);
    }

    // Method to set the motor rotation to a specified angle (percentage)
    void setRotation(float angle) {
        setAngleRotation = true;
        targetValue = constrain(angle, minPotValue, maxPotValue);
        float value = pot.readValue();
        motor.isClockwise = value < targetValue;

        if (isOutsideRange(value, targetValue, rotationUncertainty)) {
            isMoving = true;
        }
    }

    // Overloaded method to set motor rotation direction without specifying an angle
    void setRotation(bool clockwise) {
        setAngleRotation = false;
        motor.isClockwise = clockwise;

        if (clockwise) {
            targetValue = maxPotValue;
        } else {
            targetValue = minPotValue;
        }

        float value = pot.readValue();
        if (isOutsideRange(value, targetValue, rotationUncertainty)) {
            isMoving = true;
        }
    }

    // Main method to control motor rotation
    void rotate() {
        if (isMoving) {
            motor.rotate();

            // Check for lockups / motor 'catching'
            if (maxLockDuration != -1 || speedBoostAmount != -1) {
                if (lastValue == -1) {
                    lastValue = pot.readValue();
                    timer.timerInterval = 250;
                } else {
                    if (timer.interval()) {
                        currentValue = pot.readValue();

                        float delta = abs(currentValue - lastValue);
                        float rateOfChange = 1000 * delta / timer.timerInterval;

                        float expectedChange = 5.0f * motor.rotationSpeed / 17.0f;

                        Serial.print("Interval: ");
                        Serial.print(timer.elapsedIntervals);
                        Serial.print(" RateOfChange: ");
                        Serial.print(rateOfChange);
                        Serial.print(" ExpectedChange: ");
                        Serial.println(expectedChange);

                        float changeRatio = rateOfChange / expectedChange;

                        if (changeRatio <= 0.1f) {
                            if (maxLockDuration != -1) {
                                lockTime += timer.timerInterval;
                                if (lockTime >= maxLockDuration * (1.5f - motor.rotationSpeed / 255.0f)) {
                                    motor.stop();
                                    lastValue = -1;
                                    isMoving = false;
                                    lockTime = 0;
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
                motor.stop();
                isMoving = false;
                Serial.println(value);
                Serial.println("Rotation limit exceeded");
            }

            // Check if the motor is within the accepted range
            if (setAngleRotation) {
                if (!isOutsideRange(value, targetValue, rotationUncertainty)) {
                    motor.stop();
                    isMoving = false;
                    Serial.println("Inside accepted range");
                    Serial.print("Percentage: ");
                    Serial.println(pot.readValue());
                    delay(500);
                    Serial.println("DelayedPercentage: ");
                    Serial.println(pot.readValue());
                }
            }

            if (preciseRotation) {

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
    debounceTimer.timerInterval = 150;
    pressTimer.timerInterval = 100;
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
        if (outputState == LOW)
        {
            return 0;
        }
        else if (outputState == HIGH) 
        {
            pressTimer.interval();
            return pressTimer.elapsedIntervals * pressTimer.timerInterval;
        }
    }
};

// Struct to represent a pair of buttons 
struct ButtonPair {
    private:
    Button button1;
    Button button2;

  ButtonPair(Button button1, Button button2) : button1(button1), button2(button2) {

  }

  int getState() {
    bool b1state = button1.readButton();
    bool b2state = button2.readButton();

    if (b1state == HIGH && b2state == LOW) {
      return 1; 
    }
    if (b1state == LOW && b2state == HIGH) {
      return 2; 
    }
    if (b1state == HIGH && b2state == HIGH) {
      return 3;
    }
  }
};

class Wrist {
    private: 
    Button input1;
    Button input2;
    ControlledMotor motor;

    void rotateToNormal() {

    }

    void rotate(bool clockwise) {

    }

    void loop() {

    }

};

class Hand {
    private:
    Button input1;
    Button input2; 
    ControlledMotor motor; 

    void closeHand() {

    }

    void openHand() {

    }

    void loop()
    {
        
    }

};

Button button1(2);

void setup() {
    // Setup code goes here
    Serial.begin(115200);
}

void loop()
{
    button1.readButton();
    Serial.println(button1.getPressDuration());
}

*/