# 🚨 **Gas Detector Project**

## 📜 **Project Description**

This project implements a gas detection system using an Arduino and an LCD display. The system monitors gas levels via a sensor, provides real-time feedback through an LCD, and alerts with visual and auditory indicators. The project is designed to detect the presence of gas, activate alarms, and display status messages effectively.

## 🛠️ **Hardware Requirements**

- **Arduino Board** (e.g., Arduino Uno)
- **LCD Display** (I2C, 20x4)
- **Gas Sensor** (Analog)
- **Red LED**
- **Green LED**
- **Buzzer**
- **Resistors and Breadboard** (for circuit connections)
- **Jumper Wires**

## 📐 **Wiring Diagram**

1. **Gas Sensor**: Connect the sensor's output to the analog input pin **A0**.
2. **Red LED**: Connect the anode to pin **2** and the cathode to the ground (use a current-limiting resistor).
3. **Green LED**: Connect the anode to pin **3** and the cathode to the ground (use a current-limiting resistor).
4. **Buzzer**: Connect one terminal to pin **4** and the other terminal to the ground.
5. **LCD Display**: Connect the I2C pins of the LCD to the corresponding **SDA** and **SCL** pins on the Arduino. Typically, **SDA** is **A4** and **SCL** is **A5** on an Arduino Uno.

## 🧩 **Code Explanation**

The Arduino sketch performs the following tasks:

### **Initialization**
- Configures the LCD to display initial messages.
- Sets the pin modes for the sensor, LEDs, and buzzer.

### **Main Loop**
- Reads the gas sensor's value from analog input **A0**.
- If the value exceeds the predefined threshold (`MAX`), the system:
  - Activates the **Red LED** and **Buzzer**.
  - Deactivates the **Green LED**.
  - Displays "Gas is Detected" on the LCD.
- If the gas level is below the threshold, the system:
  - Deactivates the **Red LED** and **Buzzer**.
  - Activates the **Green LED**.
  - Displays "Gas level is LOW" on the LCD.

## 📂 **Code**

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
