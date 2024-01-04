// Define the Motor struct
struct Motor {
    int input1;           // Input 1 on the L298N
    int input2;           // Input 2 on the L298N
    int enable;           // Enable (PWM) pin on the L298N (optional)
    bool isClockwise = true; // Boolean indicating the rotation direction
    int speed = 255;      // Motor rotation speed (0-255)

    // Constructor to initialize the motor with optional enable pin
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
    int output; // Output pin on Arduino board

    Potentiometer(int output) : output(output) {
        pinMode(output, INPUT);
    }

    // Method to read potentiometer value
    // Note: The reading of the potentiometer increases in the anti-clockwise direction
    int readValue() {
        int potValue = analogRead(output);
        return map(potValue, 0, 1023, 100, 0);
    }

    void printData()
    {
        Serial.print("Percentage: ");
        Serial.print(readValue());
        Serial.println("%");
    }
};

// Define the ControlledMotor class
class ControlledMotor {
private:
    Motor motor;           // Motor instance
    Potentiometer pot;     // Potentiometer instance
    int minPotValue = 5;   // Minimum potentiometer value
    int maxPotValue = 95;  // Maximum potentiometer value
    float uncertainty = 1.0f; // Standard uncertainty/error in the motor rotation (measured as a percentage)
    int targetValue = 50;   // Desired potentiometer value for motor rotation
    bool isMoving = false;
    bool speedSet = false;
    bool preventLockUp = false;

    int delayInterval = 10; // Time delay between motor rotation checks
    int lastValue = -1;
    int currentValue = -1;

public:
    // Constructor
    ControlledMotor(const Motor& m, const Potentiometer& p, float uncertainty = 1.0f, int min = 5, int max = 95)
        : motor(m), pot(p), uncertainty(uncertainty), minPotValue(min), maxPotValue(max) {
    }

    // Method to check if the value is within the uncertainty range
    bool isOutsideRange(int value, int desired, float uncertainty) {
        return (value < desired - uncertainty) || (value > desired + uncertainty);
    }

    // Method to set the motor rotation to a specified angle (percentage)
    void setRotation(int angle) {
        targetValue = constrain(angle, minPotValue, maxPotValue);
        int value = pot.readValue();
        if (isOutsideRange(value, targetValue, uncertainty)) {
            isMoving = true;
            speedSet = false;
        }
    }

    // Method to set the motor rotation direction and start the motor
    void moveMotor(int target) {
        motor.isClockwise = pot.readValue() < target; 
        Serial.println(motor.isClockwise);
        Serial.println(pot.readValue());
        motor.setSpeed(50);
        motor.rotate();
        speedSet = true;
    }

    // Main method to control motor rotation
    void rotate() {
        if (isMoving) {
            if (!speedSet) {
                moveMotor(targetValue);
            }

            // Check for motor lock-up
            if (preventLockUp) {
                if (lastValue == -1) {
              lastValue = pot.readValue();
            }
            else {
              currentValue = pot.readValue();
              int delta = (currentValue - lastValue);
              float rateOfChange = delta / delayInterval; 

              float expectedChange = 340.0 * motor.speed; // 4/3 * 255
              // Note: this may need to account for loss in rotation speed when powered by battery 

              float lockThreshold = 0.1;

              lastValue = currentValue;
            
              if (rateOfChange / expectedChange <= lockThreshold) {
                motor.stop();
                lastValue = -1;
                isMoving = false;
                Serial.println("Motor stopped due to lock-up");
              }
            }
            }
           
            // Check if motor has reached rotation limits
            int value = pot.readValue();
            if (((value <= minPotValue) && !motor.isClockwise) || ((value >= maxPotValue) && motor.isClockwise)) {
                motor.stop();
                isMoving = false; 
                Serial.println("Rotation limit exceeded");
            }

            // Check if motor is within accepted range
            if (!isOutsideRange(value, targetValue, uncertainty)) {
                motor.stop();
                isMoving = false;
                speedSet = false;
                Serial.println("Inside accepted range");
                Serial.print("Percentage: ");
                Serial.println(pot.readValue());
                delay(500);
                Serial.println("DelayedPercentage: ");
                Serial.println(pot.readValue());
                
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

// Create an instance of the ControlledMotor class
ControlledMotor motor2(motor1, pot1);

void setup() {
    // Setup code goes here
    Serial.begin(115200);

    /*
    delay(1000);
    motor1.setSpeed(100);
    motor1.isClockwise = false;
    motor1.rotate();
    delay(10000);
    motor1.setSpeed(100);
    motor1.isClockwise = true;
    motor1.rotate();
    delay(10000);
    motor1.stop();
    */
    
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
    //pot1.printData();
};