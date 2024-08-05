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
