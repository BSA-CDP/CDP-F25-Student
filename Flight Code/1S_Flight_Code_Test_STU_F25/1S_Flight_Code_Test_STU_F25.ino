// FLIGHT CODE TEST (1S) - STUDENT VERSION
// FOR USE IN BLACK STUDENTS IN AEROSPACE CUBESAT DEMONSTRATION PROGRAM
// RELEASED: 08/25/2025

// Include necessary libraries
#include <Wire.h>                  // Enables I2C communication
#include <SPI.h>                   // Enables SPI communication
#include <Adafruit_Sensor.h>       // Base class for sensor objects (used by BMP388 library)
#include <Adafruit_BMP3XX.h>       // Library for the BMP388 temperature/pressure sensor
#include "SdFat.h"                 // High-performance SD card library

#define SEALEVELPRESSURE_HPA (1013.25)  // Standard sea level pressure in hPa (used for altitude calc)

// SD CARD SETUP
#define SD_CS_PIN 23 // Chip select pin for the Feather RP2040 Adalogger SD slot
SdFat SD;                            // Create an SdFat object to interface with the SD card
FsFile dataFile;                     // Create a file object for reading/writing
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);  // Configure SD

Adafruit_BMP3XX bmp;                // Create an object for the BMP388 sensor
unsigned long startTime;           // Variable to track when logging starts

// RGB LED pin definitions
const int redPin = ***Fill_In_Here***; // Define digital pin on Feather for Red
const int greenPin = ***Fill_In_Here***; // Define digital pin on Feather for Green
const int bluePin = ***Fill_In_Here***; // Define digital pin on Feather for Blue

void setup() {
  Serial.begin(115200);            // Begin serial communication at 115200 baud
  while (!Serial);                 // Wait until Serial Monitor is open
  delay(100);                      // Small delay for USB and SD to stabilize

  Serial.println("Flight Code Test");

    // initialize the digitals pin as an outputs
  pinMode(redPin, OUTPUT);
  pinMode(greenPin, OUTPUT);
  pinMode(bluePin, OUTPUT);

  // Turn the LED off initially
  digitalWrite(redPin, LOW);
  digitalWrite(greenPin, LOW);
  digitalWrite(bluePin, LOW);

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(config)) {      // Try to initialize the SD card with given config
    Serial.println("Initialization failed!");
    digitalWrite(redPin, HIGH);  // Turn on red LED for SD card failure
    while(1);                   // Halt program execution
  }
  Serial.println("SD card initialized.");

  // File Management
  if (SD.exists("Flight_Code_Test.csv")) {
    // File exists: append log marker
    dataFile = SD.open("***Fill_In_Here***", FILE_WRITE);
    if (dataFile) {
      dataFile.println("# --- NEW LOG STARTED ---");
      dataFile.close();
      Serial.println("Existing data file found. Appending…");
    }
  } else {
    // File doesn't exist: create and write header
    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE);
    if (dataFile) {
    dataFile.println("Timestamp,Temperature (C),Pressure (hPa),Altitude (m)");
    dataFile.close();
    } else {
      Serial.println("Error opening Flight_Code_Test.csv");  // Print message if file couldn't be created
      digitalWrite(redPin, HIGH); // Turn the red pin on 
      while (1);  // Halt program execution
    }
  }

  // Initialize I2C and BMP388 
  Wire.begin();                     // Start I2C communication on default SDA and SCL pins on RP2040)

  if (!bmp.begin_I2C()) {           // Initialize BMP388 sensor over I2C
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    digitalWrite(redPin, HIGH);     // Turn on Red LED if BMP388 sensor could not be found
    while (1);                      // Halt program execution if sensor not found
  }

  // Default BMP388 configuration
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Higher precision temp readings
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Medium precision pressure
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Filter out noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                // Sensor outputs data at 100 Hz

  pinMode(PIN_LED, OUTPUT);         // Set built-in LED pin as output
  startTime = millis();             // Record the start time

  Serial.println("Data collection starting...");
}

void loop() {
  // Logic to Stop After 20 Minutes
  if (millis() - startTime >= ***Fill_In_Here***) {     // Check if 20 mins in milliseconds have passed
    Serial.println("20 minutes passed. Stopping data logging.");
    digitalWrite(PIN_LED, LOW);  // turn the LED off (HIGH is the voltage level)
    digitalWrite(redPin, LOW);  // Turn off red LED
    digitalWrite(greenPin, LOW);  // Turn off green LED
    digitalWrite(bluePin, HIGH);   // Turn on blue LED
    while (1);                            // Halt the program
  }

 // Log data to the file if the SD file can be found
  if (sdAvailable())) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / 1000;  // `millis()` returns milliseconds, so divide by 1000 to get seconds

    dataFile = SD.open("Flight_Code_Test.csv", FILE_WRITE); // Reopen file in prep to write to it

    dataFile.print(timestamp);               // Write the timestamp
    dataFile.print(",");                     // CSV delimiter (comma)

    // Take a reading from the BMP388 sensor
    if (bmp.performReading()) {
      // If reading can be  taken, write the sensor data to the SD card in CSV format
      dataFile.print(bmp.temperature);         // Write the temperature in Celsius
      dataFile.print(",");                     // CSV delimiter
      dataFile.print(bmp.pressure / 100.0);    // Write the pressure in hPa (Pa to hPa conversion)
      dataFile.print(",");                     // CSV delimiter
      dataFile.println(bmp.readAltitude(SEALEVELPRESSURE_HPA));  // Write the calculated altitude in meters
      digitalWrite(redPin, LOW);    // Turn off red LED
      digitalWrite(greenPin, HIGH);   // Turn on green LED for successful logging
    } else {
      // If reading fails, print an error message and proceed
      Serial.println("Failed to read BMP388! Putting placeholder...");
      dataFile.println("***Fill_In_Here***,***Fill_In_Here***,***Fill_In_Here***"); // Placeholder values for no sensor read
    }

  } else {
    // If the file can't be accessed, print an error
    Serial.println("Error writing to Flight_Code_Test.csv");
    digitalWrite(greenPin, LOW); // Turn off green LED
    digitalWrite(redPin, HIGH); // Turn on red LED
    // Start retry attempts
    int retryCount = 0; // Create integer to track retries 
    const int maxRetries = ***Fill_In_Here***; // Set max retry number
    // Enter while loop for up set amount of retries
    while (***Fill_In_Here*** < maxRetries) {
      // Try to reconnect to the SD card 
      if (SD.begin(***Fill_In_Here***)) {
        break; // exits the while loop if successful
      } else {
      // Print to serial if reconnect attempt unsuccessful
      Serial.print("Retrying SD card initialization... Attempt ");
      Serial.println(retryCount + 1); 
      retryCount++; //increase count by 1
      delay(500); // wait 0.5 seconds before trying again
      }
    }
    // Stop code once max retry attempts reach
    if (retryCount == ***Fill_In_Here***) {
      Serial.print("SD is lost...");
      while(1);
    }
    Serial.println("SD connection recovered..."); // Signify SD connection recovered
  }
  dataFile.close();                     // Close file to prevent corruption
  digitalWrite(PIN_LED, HIGH);         // Turn on LED to show activity
  delay(1000);                         // Wait 1 second (in milliseconds) before next reading
}

bool sdAvailable() {
  // Check if SD card displays an error code
  if (SD.card()->errorCode()) {
    return false;  // Card reports an error
  }

  return true;  // SD looks good
}