# Gas Detector Project

## Project Description

This project implements a simple gas detector using an Arduino and an LCD display. The system reads sensor data from an analog input, processes it, and provides visual and auditory feedback on the presence of gas. It uses an LCD to display the status of the gas level, a buzzer to sound an alarm, and LEDs to indicate the status.

## Hardware Requirements

- **Arduino Board** (e.g., Arduino Uno)
- **LCD Display** (I2C, 20x4)
- **Gas Sensor** (Analog)
- **Red LED**
- **Green LED**
- **Buzzer**
- **Resistors and Breadboard** (for circuit connections)
- **Jumper Wires**

## Wiring Diagram

1. **Gas Sensor**: Connect the sensor's output to the analog input pin A0.
2. **Red LED**: Connect the anode to pin 2 and the cathode to the ground (use a current-limiting resistor).
3. **Green LED**: Connect the anode to pin 3 and the cathode to the ground (use a current-limiting resistor).
4. **Buzzer**: Connect one terminal to pin 4 and the other terminal to the ground.
5. **LCD Display**: Connect the I2C pins of the LCD to the corresponding SDA and SCL pins on the Arduino. Typically, SDA is A4 and SCL is A5 on an Arduino Uno.

## Code Explanation

The provided Arduino sketch performs the following tasks:

1. **Initialization**:
   - The LCD is initialized and configured to display the initial messages.
   - Pin modes are set for the sensor, LEDs, and buzzer.

2. **Main Loop**:
   - The gas sensor's value is read from the analog input (A0).
   - If the value exceeds a predefined threshold (`MAX`), the system activates the red LED and buzzer while turning off the green LED. The LCD displays a "Gas is Detected" message.
   - If the gas level is below the threshold, the system turns off the red LED and buzzer while activating the green LED. The LCD displays a "Gas level is LOW" message.

## Code
## Usage
   - Upload the Code: Connect your Arduino to your computer using a USB cable and upload the code to the Arduino using the Arduino IDE.
   - Connect Hardware: Ensure all components are connected as described in the hardware requirements section.
   - Monitor Output: Open the Serial Monitor in the Arduino IDE to view the sensor values and status messages.
## Troubleshooting
   - LCD Not Displaying: Check the wiring and ensure that the I2C address is correct. You may need to use an I2C scanner sketch to verify the address.
   - Components Not Working: Verify that all components are connected properly and that the Arduino is receiving power.
   - Inconsistent Sensor Readings: Ensure that the gas sensor is properly calibrated and connected.

![gasdetector](https://github.com/user-attachments/assets/7ab29187-b791-4b28-9fc5-e3c15d319285)

```cpp
#include <LiquidCrystal_I2C.h> // Include the LiquidCrystal_I2C library for I2C LCD control

// Initialize the LCD with I2C address 0x27, 20 columns, and 4 rows
LiquidCrystal_I2C lcd(0x27, 20, 4);

int Input = A0; // Analog input pin for the sensor
int R_LED = 2; // Pin for the Red LED
int G_LED = 3; // Pin for the Green LED
int Buzzer = 4; // Pin for the Buzzer
int val; // Variable to store the sensor value
int MAX = 200; // Threshold value for gas detection

void setup() {
  Serial.begin(9600); // Start serial communication at 9600 baud
  lcd.init(); // Initialize the LCD
  lcd.backlight(); // Turn on the backlight of the LCD

  // Display initial messages on the LCD
  lcd.setCursor(3, 1);
  lcd.print("Micro Masters");
  lcd.setCursor(3, 3);
  lcd.print("Gas Detector");
  delay(4000); // Wait for 4 seconds
  lcd.clear(); // Clear the LCD display

  // Set pin modes
  pinMode(Input, INPUT); // Set the sensor pin as input
  pinMode(R_LED, OUTPUT); // Set the Red LED pin as output
  pinMode(G_LED, OUTPUT); // Set the Green LED pin as output
  pinMode(Buzzer, OUTPUT); // Set the Buzzer pin as output

  delay(1500); // Wait for 1.5 seconds
}

void loop() {
  val = analogRead(Input); // Read the value from the sensor
  Serial.println(val); // Print the sensor value to the Serial Monitor

  // Check if the sensor value is above or equal to the threshold
  if (val >= MAX) {
    // If the gas level is high
    analogWrite(A2, 255); // Turn on an additional output (A2) at full brightness (not used in this example)
    digitalWrite(R_LED, HIGH); // Turn on the Red LED
    digitalWrite(Buzzer, HIGH); // Turn on the Buzzer
    digitalWrite(G_LED, LOW); // Turn off the Green LED

    // Display "Gas is Detected" on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Value : ");
    lcd.print(val); // Print the sensor value
    lcd.setCursor(0, 1);
    lcd.print("Gas is Detected");

    delay(300); // Wait for 300 milliseconds
    lcd.clear(); // Clear the LCD display
  } else {
    // If the gas level is low
    analogWrite(A2, 0); // Turn off the additional output (A2) (not used in this example)
    digitalWrite(R_LED, LOW); // Turn off the Red LED
    digitalWrite(Buzzer, LOW); // Turn off the Buzzer
    digitalWrite(G_LED, HIGH); // Turn on the Green LED

    // Display "Gas level is LOW" on the LCD
    lcd.setCursor(0, 0);
    lcd.print("Value : ");
    lcd.print(val); // Print the sensor value
    lcd.setCursor(0, 1);
    lcd.print("Gas level is LOW");
    delay(300); // Wait for 300 milliseconds
    Serial.println("NORMAL"); // Print "NORMAL" to the Serial Monitor
  }
}
