#include <Arduino.h>

// Define the Motor struct
struct Motor {
    int input1;           // Input 1 on the L298N
    int input2;           // Input 2 on the L298N
    int enable;           // Enable (PWM) pin on the L298N (optional)
    bool isClockwise = true; // Boolean indicating the rotation direction
    int speed = 255;      // Motor rotation speed (0-255)

    // Constructor to initialize the motor with an optional enable pin
    Motor(int input1, int input2, int enable = -1, int speed = 255)
        : input1(input1), input2(input2), enable(enable), speed(speed) {
        pinMode(input1, OUTPUT);
        pinMode(input2, OUTPUT);
        if (enable != -1) {
            pinMode(enable, OUTPUT);
        }
    }

    // Method to rotate the motor in the specified direction
    void rotate() {
        if (isClockwise) {
            digitalWrite(input1, HIGH);
            digitalWrite(input2, LOW);
        } else {
            digitalWrite(input1, LOW);
            digitalWrite(input2, HIGH);
        }
    }

    // Method to set the motor speed (if PWM is used)
    void setSpeed(int s) {
        speed = constrain(s, 0, 255);
        if (enable != -1) {
            analogWrite(enable, speed);
        }
    }

    // Method to stop the motor
    void stop() {
        digitalWrite(input1, LOW);
        digitalWrite(input2, LOW);
        if (enable != -1) {
            analogWrite(enable, 0); // Set PWM to 0 for a complete stop
        }
    }
};

// Define the Potentiometer struct
struct Potentiometer {
    int output; // Output pin on the Arduino board

    Potentiometer(int output) : output(output) {
        pinMode(output, INPUT);
    }

    // Method to read potentiometer value
    // Note: The reading of the potentiometer increases in the anti-clockwise direction
    float readValue() {
        int potValue = analogRead(output);
        return map(potValue, 0, 1023, 100.0, 0.0);
    }

    void printData() {
        Serial.print("Percentage: ");
        Serial.print(readValue());
        Serial.println("%");
    }
};

struct ProgramTimer {
public:
    unsigned long timerInterval = 0;
    unsigned long lastMillis;
    unsigned long elapsedIntervals = 0;

    ProgramTimer() {
        lastMillis = millis();
    }

    bool intervalElapsed() {
        if (timerInterval != 0) {
        unsigned long currentMillis = millis();
        if (currentMillis - lastMillis >= timerInterval) {
            lastMillis = currentMillis;
            elapsedIntervals++;
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

// Define the ControlledMotor class
class ControlledMotor {
private:
    Motor motor;                             // Motor instance
    Potentiometer pot;                       // Potentiometer instance
    RotationSpeedTable speedTable;          // SpeedTable instance
    ProgramTimer timer;                     // ProgramTimer instance
    int minPotValue = 5;                    // Minimum potentiometer value
    int maxPotValue = 95;                   // Maximum potentiometer value
    float uncertainty = 0.0f;               // Standard uncertainty/error in the motor rotation (measured as a percentage)
    float targetValue = 50.0;              // Desired potentiometer value for motor rotation
    bool isMoving = false;
    bool motorSet = false;
    bool preventLockUp = false;
    int maxLockTime = 1000;
    int lockTime = 0;
    int speedBoost = 10;                    // Boosts motor speed when 'catching'. Set to -1 to disable. Only works with variable motor speed. 
    bool isDecaying = true;
    int lastValue = -1;
    int currentValue = -1;
    bool collectingData = true;
    bool isBoosting = false;

    bool setAngleRotation = false; 

public:
    // Modified constructor to include RotationSpeedTable parameter
    ControlledMotor(const Motor& m, const Potentiometer& p, const RotationSpeedTable& speedTable, bool rotationMode = false, float uncertainty = 0.0f, int min = 5, int max = 95)
        : motor(m), pot(p), speedTable(speedTable), uncertainty(uncertainty), minPotValue(min), maxPotValue(max) {
    }

    // Method to check if the value is within the uncertainty range
    bool isOutsideRange(float value, float desired, float uncertainty) {
        return (value < desired - uncertainty) || (value > desired + uncertainty);
    }

    // Method to set the motor rotation to a specified angle (percentage)
    void setRotation(float angle) {
        targetValue = constrain(angle, minPotValue, maxPotValue);
        float value = pot.readValue();
        if (isOutsideRange(value, targetValue, uncertainty)) {
            isMoving = true;
            motorSet = false;
        }
    }

    // Method to set the motor rotation direction and start the motor
    void moveMotor(float target) {
        if (setAngleRotation) {
            motor.isClockwise = pot.readValue() < target;
        }
        else
        {
            if (target == minPotValue) {
                motor.isClockwise = false;
            }
            else if (target == maxPotValue) {
                motor.isClockwise = true; 
            }
        }
        motor.rotate();
        motorSet = true;
    }

    // Main method to control motor rotation
    void rotate() {
        if (isMoving) {
            if (!motorSet) {
                moveMotor(targetValue);
            }


            // Check for lockups / motor 'catching' 
            if (preventLockUp || speedBoost != -1) {
                if (lastValue == -1)
                {
                    lastValue = pot.readValue();
                    timer.timerInterval = 250;
                    
                } else {
                    if (timer.intervalElapsed()) {
                        currentValue = pot.readValue();

                        float delta = abs(currentValue - lastValue);
                        float rateOfChange = 1000 * delta / timer.timerInterval;


                        float expectedChange = 5.0f * motor.speed / 17.0f; // NOTE: this may need to be changed when the battery is powering the load due to voltage induced speed differences

                        
                        Serial.print("Interval: ");
                        Serial.print(timer.elapsedIntervals);
                        Serial.print(" RateOfChange: ");
                        Serial.print(rateOfChange);
                        Serial.print(" ExpectedChange: ");
                        Serial.println(expectedChange);
                        

                        float changeRatio = rateOfChange / expectedChange;

                        if (changeRatio <= 0.1f) {
                            if (preventLockUp) {
                                lockTime += timer.timerInterval;
                                if (lockTime >= maxLockTime * (1.5f - motor.speed / 255.0f)) {
                                    motor.stop();
                                    lastValue = -1;
                                    isMoving = false;
                                    motorSet = false;
                                    lockTime = 0;
                                    Serial.println("Motor stopped due to lock-up");
                                }
                            }
                        } 

                        if (changeRatio <= 0.5f) {
                            if (speedBoost != -1) {
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
                    speed += speedBoost;
                }
                motor.setSpeed(speed);
            }

            // Check if the motor has reached rotation limits
            float value = pot.readValue();
            if (((value <= minPotValue) && !motor.isClockwise) || ((value >= maxPotValue) && motor.isClockwise)) {
                motor.stop();
                isMoving = false;
                motorSet = false;
                Serial.println(value);
                Serial.println("Rotation limit exceeded");
            }

            // Check if the motor is within the accepted range
            if (setAngleRotation) {

                if (!isOutsideRange(value, targetValue, uncertainty)) {
                    motor.stop();
                    isMoving = false;
                    motorSet = false;
                    Serial.println("Inside accepted range");
                    Serial.print("Percentage: ");
                    Serial.println(pot.readValue());
                    delay(500);
                    Serial.println("DelayedPercentage: ");
                    Serial.println(pot.readValue());
            }
            }

        }
    }
};

void clearSerialMonitor() {
    Serial.write("\033[2J\033[H"); // ANSI escape code to clear the screen and move cursor to (0,0)
}

// Create instances of the Motor and Potentiometer structs
Motor motor1(8, 9, 6); // In a real-life circuit, even numbers on Arduino and L298N are connected
Potentiometer pot1(A0);

// Create a speed table
RotationSpeedPair speedPairs[] = {
    {0, 45},
    {40, 180},
    {80, 255},
    {100, 255},
};

int numberOfPairs = sizeof(speedPairs) / sizeof(speedPairs[0]);
RotationSpeedTable speedTable(speedPairs, numberOfPairs, true);

// Create an instance of the ControlledMotor class with the speed table
ControlledMotor motor2(motor1, pot1, speedTable, true);

void setup() {
    // Setup code goes here
    Serial.begin(115200);

    motor2.setRotation(100.0f);
}

int outputCounter = 0;

void loop() {
    // Check for incoming Serial data
    if (Serial.available() > 0) {
        // Read and constrain the user input to a range of 0 to 100
        int userInput = Serial.parseInt();
        int constrainedInput = constrain(userInput, 0, 100);

        // Set the motor rotation angle based on the constrained input
        motor2.setRotation(constrainedInput);

        // Increment and reset the output counter every 10 iterations
        if (++outputCounter == 10) {
            clearSerialMonitor();
            outputCounter = 0;
        }

        // Clear any remaining characters in the Serial buffer
        while (Serial.available() > 0) {
            char c = Serial.read();
        }
    }

    motor2.rotate();
}


//CHANGE ALL SETROTATIONANGLE STUFF TO USE A CONSTRUCTOR THAT TAKES DIFFERENT TYPES