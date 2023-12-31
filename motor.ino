/*struct Motor {
    int input1;           // Input 1 on the L298N
    int input2;           // Input 2 on the L298N
    bool clockwise = true;  // Boolean indicating the rotation direction
    int motorSpeed = 255; // Motor rotation speed (0-255)

    // Constructor to initialize the motor
    Motor(int input1, int input2) : input1(input1), input2(input2) {
        pinMode(input1, OUTPUT);
        pinMode(input2, OUTPUT);
    }

    // Method to rotate the motor in the specified direction
    void rotate() {
        if (clockwise) {
            analogWrite(input1, motorSpeed);
            digitalWrite(input2, LOW);
        } else {
            digitalWrite(input1, LOW);
            analogWrite(input2, motorSpeed);
        }
    }

    // Method to stop the motor
    void stop() {
        digitalWrite(input1, LOW);
        digitalWrite(input2, LOW);
    }
};

Motor myMotor = Motor(8,9);

void setup() {
    Serial.begin(9600);
}

void loop() {
    // Example usage
    Motor myMotor(6, 7);  // Assuming pin 6 and pin 7 are connected to the motor
    myMotor.rotate();      // Rotate the motor according to the current direction and speed
    delay(2000);          // Wait for 2 seconds
    myMotor.clockwise = false;  // Change the direction
    myMotor.rotate();      // Rotate in the opposite direction
    delay(2000);          // Wait for 2 seconds
    myMotor.stop(); 
}
*/