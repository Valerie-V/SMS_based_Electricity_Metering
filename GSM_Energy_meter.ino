/* 
Created by VAAV Innovative Solutions LTD:

************** Arduino Connections ************************************
** Arduino's TX pin -> SIM800L module's RX pin.
** Arduino's RX pin -> SIM800L module's TX pin.
** Arduino's SoftwareSerial RX pin (pin 10) -> PZEM-004T module's TX pin.
** Arduino's SoftwareSerial TX pin (pin 11) -> PZEM-004T module's RX pin.
** Arduino's 5V pin to the VCC pins of the SIM800L module and PZEM-004T module.
** Separate 5v power supply for SIM800L module
** Arduino's GND pin -> GND pins: SIM800L module and PZEM-004T module.

**************** PZEM-004T Connections *************************************
Using 4 wires
** PZEM-004T module's RS485 A and B pins to the respective terminals of the energy meter. 
**** RS485 Communication is Vcc, Rx, Tx, GND

*********************** LCD Connection USING I2C ************************
Just 4 wires

The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * LCD VSS pin to ground
 * LCD VCC pin to 5V
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)

Power Supply: using lm2956 to power sim module:
**5V, GND

*/


#include <SoftwareSerial.h>
#include <PZEM004T.h>
#include <Adafruit_FONA.h>
//#include <LiquidCrystal_I2C.h>
#include <LiquidCrystal.h>

// Pins for SIM800L module
#define SIM800L_RX_PIN 10
#define SIM800L_TX_PIN 11
#define SIM800L_BAUDRATE 9600

// Pins for PZEM004T module
#define PZEM004T_RX_PIN 12
#define PZEM004T_TX_PIN 13

// SIM card PIN and GSM network APN
#define SIM_PIN "1234" // Replace with your SIM card PIN
#define APN "your_APN" // Replace with your GSM network's APN

/* // LCD settings
#define LCD_ADDRESS 0x27
#define LCD_COLUMNS 16
#define LCD_ROWS 2 */

const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
// Create objects for SIM800L, PZEM004T, and LCD
SoftwareSerial sim800lSerial(SIM800L_RX_PIN, SIM800L_TX_PIN);
Adafruit_FONA sim800l = Adafruit_FONA(SIM800L_RST_PIN);
PZEM004T pzem(PZEM004T_RX_PIN, PZEM004T_TX_PIN);
//LiquidCrystal_I2C lcd(LCD_ADDRESS, LCD_COLUMNS, LCD_ROWS);
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

unsigned long previousMillis = 0;
const unsigned long interval = 180000; // 3 minutes interval

void setup() {
  // Initialize serial communication
  sim800lSerial.begin(SIM800L_BAUDRATE);
  Serial.begin(9600);

  // Check if SIM800L module is connected
  if (!sim800l.begin(sim800lSerial)) {
    Serial.println("Couldn't find SIM800L module");
    while (1);
  }

  // Unlock SIM card
  if (!sim800l.unlockSIM(SIM_PIN)) {
    Serial.println("SIM card unlock failed");
    while (1);
  }

  // Enable GPRS network
  if (!sim800l.enableGPRS(true)) {
    Serial.println("GPRS network failed to enable");
    while (1);
  }

  // Attach to GPRS
  if (!sim800l.attachGPRS(APN)) {
    Serial.println("GPRS attachment failed");
    while (1);
  }

  // Initialize LCD
  lcd.begin(LCD_COLUMNS, LCD_ROWS);
  lcd.clear();
}

void loop() {
  unsigned long currentMillis = millis();

  // Read energy consumption from PZEM004T module every 3 minutes
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    float voltage = pzem.voltage();
    float current = pzem.current();
    float power = pzem.power();
    float energy = pzem.energy();

    // Display data on LCD
    lcd.setCursor(0, 0);
    lcd.print("Voltage: ");
    lcd.print(voltage);
    lcd.print("V");
    lcd.setCursor(0, 1);
    lcd.print("Power: ");
    lcd.print(power);
    lcd.print("W");

    // Compose the hourly log message
    String logMessage = "Hourly Log\n";
    logMessage += "Voltage: " + String(voltage) + "V\n";
    logMessage += "Current: " + String(current) + "A\n";
    logMessage += "Power: " + String(power) + "W\n";
    logMessage += "Energy: " + String(energy) + "Wh\n";

    // Send hourly log via SMS
    sim800l.sendSMS("recipient_number", logMessage);
  }

  // Check for power restoration and place a call
  if (sim800l.getNetworkStatus() == Adafruit_FONA::NETWORK_OK) {
    // Place a call
    sim800l.callVoice("recipient_number");
    delay(10000); // Wait for 10 seconds
    sim800l.hangUp();
  }
}
