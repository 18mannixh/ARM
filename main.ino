#include <Arduino.h>
#include <avr/power.h>

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
    ProgramTimer systemCheckTimer; 
 
    // User-changeable parameters
    int minPotValue;
    int maxPotValue;
    float rotationUncertainty;
    int maxLockDuration;
    int speedBoostAmount;
    bool isDecaying;
    int maxPrecisionResets;
    int motorEffectiveRPM;
    bool systemCheck = false; 
 
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
            preciseRotationTimer.timerInterval = 500;
            systemCheckTimer.timerInterval = 100;

        systemCheck = checkSystem();
    }

    bool checkSystem() {
        int firstValue = pot.readValue();
        setRotation(targetValue);
        motor.rotate();
        delay(300);
        motor.stop();
        int secondValue = pot.readValue();

        if (abs(secondValue - targetValue) > abs(firstValue - targetValue)) {
            Serial.print("ERROR: Motor is not synced with potentiometer");
            return false; 
        }

        return true; 
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
        pot.printData();
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
                    if (maxPrecisionResets != -1 && precisionResets != maxPrecisionResets-1) {
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
  Button input1 = Button(12); // leftmost button
  Button input2 = Button(11); // rightmost button
 
  bool lastb1State = false;
  bool lastb2State = false;
 
  Motor motor1{9, 8,10}; // In a real-life circuit, even numbers on Arduino and L298N are connected
  Potentiometer pot1{A1};
 
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
      .maxLockDuration = 2000,
      .speedBoostAmount = 10,
      .isDecaying = true,
      .maxPrecisionResets = 3,
      .motorEffectiveRPM = 30,
  };
 
  ControlledMotor motor{motor1, pot1, speedTable, motorParams};
 
public:
  void loop() {
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

    if (b1state == LOW && b2state == LOW && !motor.setAngleRotation) {
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
  Button input3 = Button(5); // leftmost button
  Button input4 = Button(4); // rightmost button
 
  bool lastb3State = false;
  bool lastb4State = false;

  bool latch = false;
  bool opening = true;
 
  Motor motor2{13,12,11}; // In a real-life circuit, even numbers on Arduino and L298N are connected
  Potentiometer pot2{A1};
 
  RotationSpeedPair speedPairs[4] = {
      {0, 100},
      {40, 100},
      {80, 255},
      {100, 255},
  };
 
  int numberOfPairs = sizeof(speedPairs) / sizeof(speedPairs[0]);
 
  RotationSpeedTable speedTable{speedPairs, numberOfPairs, true};
  MotorControlParameters motorParams = {
      .minPotValue = 20,
      .maxPotValue = 85,
      .rotationUncertainty = 0.0f,
      .maxLockDuration = 2000,
      .speedBoostAmount = 10,
      .isDecaying = true,
      .maxPrecisionResets = -1,
      .motorEffectiveRPM = 30,
  };
 
  ControlledMotor motor{motor2, pot2, speedTable, motorParams};
 
public:
  void loop() {
    pot2.printData();
    //Serial.print(pot2.readValue());
    //Serial.print(input3.readButton());
    if (true) {
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
            float openFingersAngle = 40.0f;
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
  }
};

class Battery {
private:
    int managementKey;           // Pin connected to the charge/discharge/protection/boost board's KEY input
    int offSwitchPin;            // Pin connected to the off switch
    ProgramTimer toggleTimer;    // Timer to control the toggling of the management key pin
    ProgramTimer highTimer;      // Timer to keep the pin high for one second
    bool isHigh = false;         // Flag to track the state of the output pin
    bool toggle = false;         // Flag to toggle the output pin
    bool isOff = false;          // Flag to track if the output is turned off

public:

    Battery(int keyPin = 13, int offPin = 2) : managementKey(keyPin), offSwitchPin(offPin) {
        pinMode(managementKey, OUTPUT);
        pinMode(offSwitchPin, INPUT_PULLUP); // Assuming the off switch pin is connected to ground when pressed
        toggleTimer.timerInterval = 20000; // Set timer interval to 20 seconds for toggling
        highTimer.timerInterval = 1000;    // Set timer interval to 1 second for keeping pin high
    }

    // Loop function to toggle the management key pin state at a regular interval
    void loop() {
        // Check if the off switch is pressed
        if (digitalRead(offSwitchPin) == LOW) {
            isOff = true;
            Serial.print("OFF");
            return;
        } else {
            isOff = false;
        }
        
        // If not turned off, continue normal operation
        if (toggleTimer.interval()) { 
            toggle = true; // Set toggle flag to true every 20 seconds
        }
        
        if (toggle && !isOff) {
            if (!isHigh) {
                digitalWrite(managementKey, HIGH);
                isHigh = true;
                highTimer.reset();
            }

            if (highTimer.interval()) {
                digitalWrite(managementKey, LOW);
                isHigh = false;
                toggle = false;
            }
        }
    }
};

class Management {
private:
    // Variables
    int mode; // Mode identifier
    Button* buttonArray; // Pointer to an array of Button objects
    int numberOfButtons; // Number of buttons in the array
    Potentiometer* potArray; // Pointer to an array of Potentiometer objects
    int numberOfPotentiometers; // Number of potentiometers in the array
    Motor* motor; // Pointer to a Motor object
    ProgramTimer timer; // Timer for managing the interval between operations
    unsigned long timerInterval; // Timer interval

public:
    // Constructors
    Management() : mode(0), buttonArray(nullptr), numberOfButtons(0), potArray(nullptr), numberOfPotentiometers(0), motor(nullptr) {} // Initialize mode to 0

    // Overloaded constructor for button panel test mode
    Management(Button* buttons, int count, unsigned long interval = 5000) : mode(1), buttonArray(buttons), numberOfButtons(count), timerInterval(interval) {
        timer.timerInterval = timerInterval;
    }

    // Overloaded constructor for potentiometer display mode
    Management(Potentiometer* pots, int count, unsigned long interval = 5000) : mode(2), potArray(pots), numberOfPotentiometers(count), timerInterval(interval) {
        timer.timerInterval = timerInterval;
    }

    // Overloaded constructor for motor rotation mode
    Management(Motor* m, unsigned long interval = 5000) : mode(3), motor(m), timerInterval(interval) {
        timer.timerInterval = timerInterval;
        motor->setSpeed(255);
    }

    // Methods
    void executeOperation() {
        if (mode == 1) { // Button panel test mode
            displayButtonValues();
        } else if (mode == 2) { // Potentiometer display mode
            displayPotentiometerValues();
        } else if (mode == 3) { // Motor rotation mode
            rotateMotor();
        }
    }

    void displayButtonValues() {
    if (mode == 1) { // Button panel test mode
            Serial.println("Button Panel Test Mode");
            Serial.println("----------------------");
            for (int i = 0; i < numberOfButtons; ++i) {
                Serial.print("Button ");
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.println(buttonArray[i].readButton() ? "Pressed" : "Released");
            }
            Serial.println();
        }
    }

    void displayPotentiometerValues() {
        if (mode == 2) { // Potentiometer display mode
            Serial.println("Potentiometer Display Mode");
            Serial.println("--------------------------");
            for (int i = 0; i < numberOfPotentiometers; ++i) {
                Serial.print("Potentiometer ");
                Serial.print(i + 1);
                Serial.print(": ");
                Serial.print("Value: ");
                Serial.println(potArray[i].readValue());
            }
            Serial.println();
        }
    }    

    void rotateMotor() {
        if (Serial.available() > 0) {
            char input = Serial.read();
            if (input == 'C' || input == 'c') {
                motor->isClockwise = true;
            } else if (input == 'A' || input == 'a') {
                motor->isClockwise = false;
            }

            motor->rotate();
            delay(500); // Rotate for half a second
            motor->stop();
        }
    }

    void mainLoop() {
        if (timer.interval()) {
            executeOperation();
        }
    }
};

struct IO {
    int* livePins;     // Array of live pin numbers
    int* groundPins;   // Array of ground pin numbers
    int liveCount;     // Number of live pins
    int groundCount;   // Number of ground pins

    // Constructor to initialize the IO struct with pins
    IO(int* live, int liveCount, int* ground, int groundCount)
        : livePins(live), liveCount(liveCount), groundPins(ground), groundCount(groundCount) {}

    // Method to set live pins HIGH and ground pins LOW
    void setLiveAndGround() {
        for (int i = 0; i < liveCount; ++i) {
            pinMode(livePins[i], OUTPUT);
            digitalWrite(livePins[i], HIGH);
        }

        for (int i = 0; i < groundCount; ++i) {
            pinMode(groundPins[i], OUTPUT);
            digitalWrite(groundPins[i], LOW);
        }
    }
};



// Define live and ground pins
int livePins[] = {A0, 7};   
int groundPins[] = {A2, 6};   
IO io(livePins, sizeof(livePins) / sizeof(livePins[0]), groundPins, sizeof(groundPins) / sizeof(groundPins[0]));



void setup() {
    Serial.begin(115200);
    delay(1000);
    io.setLiveAndGround();
    
    
}
//Potentiometer potentiometers[] = {Potentiometer(A1)};
//Management Pmanager(potentiometers, sizeof(potentiometers) / sizeof(potentiometers[0]), 200);

//Button buttons[] = {Button(11), Button(10), Button(9), Button(8)};
//Management manager(buttons, sizeof(buttons) / sizeof(buttons[0]), 500);

//Motor motortest{12,13,11};

//435 for wrist 
//201 for hand
//Management manager(&motortest,10);



Hand hand;
//Battery battery;

void loop() {
  //manager.mainLoop();
  //Pmanager.mainLoop();
  hand.loop();
    //battery.loop();
}



// HAND: LOWER IS 20, UPPER 85  
// OPEN IS 70
