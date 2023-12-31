// Define the Motor struct
struct Motor {
    int input1;           // Input 1 on the L298N
    int input2;           // Input 2 on the L298N
    int enable;           // Enable (PWM) pin on the L298N (optional)
    bool clockwise = true; // Boolean indicating the rotation direction
    int motorSpeed = 255;  // Motor rotation speed (0-255)

    // Constructor to initialize the motor with optional enable pin
    Motor(int input1, int input2, int enable = -1, int motorSpeed = 255) : input1(input1), input2(input2), enable(enable), motorSpeed(motorSpeed) {
        pinMode(input1, OUTPUT);
        pinMode(input2, OUTPUT);
        if (enable != -1) {
            pinMode(enable, OUTPUT);
        }
    }

    // Method to rotate the motor in the specified direction
    void rotate() {
        if (clockwise) {
            digitalWrite(input1, HIGH);
            digitalWrite(input2, LOW);
        } else {
            digitalWrite(input1, LOW);
            digitalWrite(input2, HIGH);
        }
    }

    // Method to set the motor speed (if PWM is used)
    void setMotorSpeed(int speed) {
        motorSpeed = constrain(speed, 0, 255);
        if (enable != -1) {
            analogWrite(enable, motorSpeed);
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
    int readPot() {
        int potValue = analogRead(output);
        return map(potValue, 0, 1023, 100, 0);
    }
};

// Define the ControlledMotor class
class ControlledMotor {
private:
    Motor motor;           // Motor instance
    Potentiometer pot;     // Potentiometer instance
    int minValue = 5;      // Minimum potentiometer value
    int maxValue = 95;     // Maximum potentiometer value
    float stdUncertainty = 1.0f; // Standard uncertainty/error in the motor rotation (measured as a percentage)
    int desiredValue = 50; // Desired potentiometer value for motor rotation
    bool isRotating = false;
    bool motorSet = false;

public:
    // Constructor
    ControlledMotor(const Motor& motor, const Potentiometer& pot, float stdUncertainty = 1.0f, int minValue = 5, int maxValue = 95)
        : motor(motor), pot(pot), stdUncertainty(stdUncertainty), minValue(minValue), maxValue(maxValue) {
    }

    // Method to check if the value is within the uncertainty range
    bool outsideUncertaintyRange(int value, int desired, float uncertainty) {
        return (value < desired - uncertainty) || (value > desired + uncertainty);
    }

    // Method to set the motor rotation to a specified angle (percentage)
    void setRotationAngle(int target) {
        // Ensure the desired percentage is within valid boundaries
        desiredValue = constrain(target, minValue, maxValue);
        int currentValue = pot.readPot();
        if (outsideUncertaintyRange(currentValue, desiredValue, stdUncertainty)) {
            isRotating = true;
            motorSet = false;
        }
    }

    // Method to set the motor rotation based on the desired angle
    void setMotor(int target) {
        motor.clockwise = pot.readPot() < target; // set the direction of rotation towards the desired angle
        motor.rotate();
        motorSet = true;
    }

    // Method to rotate the motor
    // Note: need to add code to "decay" rotation speed when reaching target
    void rotate() {
        if (isRotating) {
            if (!motorSet) {
                setMotor(desiredValue);
            }

            int currentValue = pot.readPot();
            if (((currentValue <= minValue) && !motor.clockwise) || ((currentValue >= maxValue) && motor.clockwise)) {
                motor.stop();
                isRotating = false; // Stop rotating when reaching the boundaries
                Serial.println("Rotation limit exceeded");
            }

            if (!outsideUncertaintyRange(currentValue, desiredValue, stdUncertainty)) {
                motor.stop();
                isRotating = false;
                motorSet = false;
                Serial.println("Inside accepted range");
                Serial.print("Percentage: ");
                Serial.println(currentValue);
            }
        }
    }
};

void clearSerialMonitor() {
    Serial.write("\033[2J\033[H"); // ANSI escape code to clear the screen and move cursor to (0,0)
}


// Create instances of the Motor and Potentiometer structs
Motor motor1(8, 9, 6); //in real life circuit even numbers on Arduino and L298N are connected
Potentiometer pot1(A0);

// Create an instance of the ControlledMotor class
ControlledMotor motor2(motor1, pot1);

void setup() {
    // Setup code goes here
    Serial.begin(115200);
    delay(1000);
    motor1.setMotorSpeed(100);
    motor1.clockwise = false;
    motor1.rotate();
    delay(10000);
    motor1.setMotorSpeed(100);
    motor1.clockwise = true;
    motor1.rotate();
    delay(10000);
    motor1.stop();
}

int outputCounter = 0;

void loop()
{

}
/*
void loop() {
    // Check for incoming Serial data
    if (Serial.available() > 0) {
        // Read and constrain the user input to a range of 0 to 100
        int userInput = Serial.parseInt();
        int constrainedInput = constrain(userInput, 0, 100);

        // Set the motor rotation angle based on the constrained input
        motor2.setRotationAngle(constrainedInput);

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


    
    Serial.print("Percentage: ");
    Serial.print(pot1.readPot());
    Serial.println("%");
    

    motor2.rotate();
}
*/
