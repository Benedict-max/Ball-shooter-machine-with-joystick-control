#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Define motor control pins
const int motorPin1 = 8;        // Connect to one terminal of the motor
const int motorPin2 = 7;        // Connect to the other terminal of the motor
const int enablePin = 6;        // Connect to the enable pin of the motor driver

// Define joystick pins
const int joyXPin = A1;         // Connect X-axis of joystick to analog pin A0

// Define limit switch pins
const int leftLimitSwitchPin = 5;   // Connect to limit switch for left end
const int rightLimitSwitchPin = 4;  // Connect to limit switch for right end

// Define joystick thresholds
const int joyThreshold = 20;    // Adjust as needed

// Define variables to store motor speed and direction
int motorSpeed = 0;             // Initial speed is 0
int motorDirection = 0;         // Initial direction is stopped

// Initialize LCD
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  // Set motor control pins as outputs
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);

  // Set limit switch pins as inputs with pullup resistors
  pinMode(leftLimitSwitchPin, INPUT_PULLUP);
  pinMode(rightLimitSwitchPin, INPUT_PULLUP);

  // Initialize serial communication
  Serial.begin(9600);

  // Initialize LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Ball Shooter Machine");
  lcd.setCursor(0, 1);
  lcd.print("By Perez and Erimina");
}

void loop() {
  // Read joystick value
  int joyX = analogRead(joyXPin);

  // Check if joystick moved left
  if (joyX < (512 - joyThreshold)) {
    // Check if not at left limit
    if (digitalRead(leftLimitSwitchPin) == HIGH) {
      // Set motor direction to left
      motorDirection = -1;
    }
  }
  // Check if joystick moved right
  else if (joyX > (512 + joyThreshold)) {
    // Check if not at right limit
    if (digitalRead(rightLimitSwitchPin) == HIGH) {
      // Set motor direction to right
      motorDirection = 1;
    }
  }
  // Joystick centered, stop the motor
  else {
    motorDirection = 0;
  }

  // Update motor speed based on direction
  if (motorDirection != 0) {
    // Set motor to move in the determined direction
    if (motorDirection == -1) {
      digitalWrite(motorPin1, HIGH);
      digitalWrite(motorPin2, LOW);
    } else {
      digitalWrite(motorPin1, LOW);
      digitalWrite(motorPin2, HIGH);
    }
    // Set the motor speed (PWM) using analogWrite
    analogWrite(enablePin, 150); // Set your desired speed here (0-255)
  } else {
    // Stop the motor
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
    analogWrite(enablePin, 0); // Set speed to 0 to stop the motor
  }

  // Update LCD display
  lcd.setCursor(0, 2);
  if (motorDirection == 1) {
    lcd.print("Moving Left    ");
  } else if (motorDirection == -1) {
    lcd.print("Moving Right   ");
  } else {
    lcd.print("Stopped        ");
  }

  // Display limit switch status on the fourth row of the LCD
  lcd.setCursor(0, 3);
   lcd.print(digitalRead(leftLimitSwitchPin) == HIGH ? "SWL:OPEN " : "SWL:CLOSED");
  lcd.print(" ");
  lcd.print(digitalRead(rightLimitSwitchPin) == HIGH ? "SWR:OPEN" : "SWR:CLOSED");

  // Print joystick value and motor status (for debugging)
  Serial.print("JoyX: ");
  Serial.print(joyX);
  Serial.print(" Motor Direction: ");
  Serial.print(motorDirection);
  Serial.print(" Limit Switches - Left: ");
  Serial.print(digitalRead(leftLimitSwitchPin));
  Serial.print(" Right: ");
  Serial.println(digitalRead(rightLimitSwitchPin));

  // Add a short delay to avoid reading the joystick too quickly
  delay(50);
}
