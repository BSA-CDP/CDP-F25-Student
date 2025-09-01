// BMP388 UNIT TEST - STUDENT VERSION
// FOR USE IN BLACK STUDENTS IN AEROSPACE CUBESAT DEMONSTRATION PROGRAM
// RELEASED: 08/25/2025

// Include necessary libraries
#include <*** FILL_IN_HERE ***>                  // Enables I2C communication
#include <*** FILL_IN_HERE ***>                   // Enables SPI communication
#include <*** FILL_IN_HERE ***>       // Base class for sensor objects (used by BMP388 library)
#include <*** FILL_IN_HERE ***>       // Library for the BMP388 temperature/pressure sensor
#include "*** FILL_IN_HERE ***"                 // High-performance SD card library

#define SEALEVELPRESSURE_HPA (*** FILL_IN_HERE ***)  // Standard sea level pressure in hPa (used for altitude calc)

// SD CARD SETUP
#define SD_CS_PIN *** FILL_IN_HERE ***                 // Chip select pin for the Feather RP2040 Adalogger SD slot
SdFat SD;                            // Create an SdFat object to interface with the SD card
FsFile dataFile;                     // Create a file object for reading/writing
SdSpiConfig config(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &SPI1);  // Configure SD

Adafruit_BMP3XX bmp;                // Create an object for the BMP388 sensor
unsigned long startTime;           // Variable to track when logging starts

void setup() {
  Serial.begin(115200);            // Begin serial communication at 115200 baud
  while (!Serial);                 // Wait until Serial Monitor is open
  delay(100);                      // Small delay for USB and SD to stabilize

  Serial.println("Feather RP2040 - BMP388 Unit Test");

  // Initialize SD Card
  Serial.print("Initializing SD card...");
  if (!SD.begin(*** FILL_IN_HERE ***)) {      // Try to initialize the SD card with given config
    Serial.println("Initialization failed!");
    while(1);                   // Halt program execution
  }
  Serial.println("SD card initialized.");

  // File Management
  if (SD.exists("BMP388_data.csv")) {   // Check if a data file already exists
    SD.remove("BMP388_data.csv");       // Delete it to start fresh
    Serial.println("Existing data file deleted.");
  }

  dataFile = SD.open("BMP388_data.csv", FILE_WRITE);  // Open new file for writing
  if (dataFile) {
    dataFile.println("*** FILL_IN_HERE *** (s),*** FILL_IN_HERE *** (C),*** FILL_IN_HERE *** (hPa),*** FILL_IN_HERE *** (m)");  // Write CSV header for data we want to collect
    dataFile.close();  // Close immediately to avoid keeping file open across loops
  } else {
    Serial.println("Error opening BMP388_data.csv");  // Print message if file couldn't be created
    while (1);  // Halt program execution
  }

  // Initialize I2C and BMP388 
  Wire.begin();                     // Start I2C communication on default SDA and SCL pins on RP2040)

  if (!bmp.begin_I2C()) {           // Initialize BMP388 sensor over I2C
    Serial.println("Could not find a valid BMP388 sensor, check wiring!");
    while (1);                      // Halt program execution if sensor not found
  }

  // Default BMP388 configuration
  bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);  // Higher precision temp readings
  bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);     // Medium precision pressure
  bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);        // Filter out noise
  bmp.setOutputDataRate(BMP3_ODR_100_HZ);                // Sensor outputs data at 100 Hz

  pinMode(PIN_LED, OUTPUT);         // Set built-in LED pin as output
  startTime = millis();             // Record the start time

  Serial.println("*** FILL_IN_HERE ***"); // Print to serial monitor that data is starting to be collected
}

void loop() {
  // Logic to Stop After 1 Minute
  if (millis() - startTime >= *** FILL_IN_HERE ***) {     // Check if 1 min (in milliseconds) have passed
    Serial.println("1 minute passed. Stopping data logging.");
    digitalWrite(PIN_LED, LOW);           // Turn off LED
    while (1);                            // Halt the program
  }

 // Log data to the file if the SD file can be found
  if (sdAvailable()) {
    // Calculate the timestamp (in seconds) since the program started
    unsigned long timestamp = millis() / *** FILL_IN_HERE ***;  // `millis()` returns milliseconds, so divide by an amount to get seconds

    dataFile = SD.open("BMP388_data.csv", FILE_WRITE); // Reopen file in prep to write to it

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
    } else {
      // If reading fails, print an error message and halt the program
      Serial.println("Failed to read BMP Sensor!");
      while(1);
    }

  } else {
    // If the file can't be accessed, print an error
    Serial.println("Error writing to BMP388_data.csv");
    while(1);
  }
  dataFile.close();                     // Close file to prevent corruption
  digitalWrite(*** FILL_IN_HERE ***, *** FILL_IN_HERE ***);         // Turn on LED to show activity
  delay(*** FILL_IN_HERE ***);                         // Wait 1 second (in milliseconds) before next reading
}

// SD check Function
bool sdAvailable() {
  // Check if card throws an error, an indicator it is unplugged
  if (SD.card()->errorCode()) {
    return false;  // Card reports an error,
  }

  return true;  // SD looks good
}