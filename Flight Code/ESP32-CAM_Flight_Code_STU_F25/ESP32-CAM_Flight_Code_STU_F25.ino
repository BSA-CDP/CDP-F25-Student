// ESP32-CAM Flight Code - STUDENT VERSION
// FOR USE IN BLACK STUDENTS IN AEROSPACE CUBESAT DEMONSTRATION PROGRAM
// RELEASED: 08/25/2025

#include "esp_camera.h" // ESP32 Camera Driver library
#include "FS.h"         // File System library
#include "SD_MMC.h"     // SD card library for ESP32’s built-in SD/MMC hardware interface
#include "soc/soc.h"           // Required to disable brownout problems
#include "soc/rtc_cntl_reg.h"  // Required to disable brownout problems

// AI Thinker ESP32-CAM Pin Definitions
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

#define FLASH_LED_PIN      4  // ESP32-CAM onboard flash LED

int photoCount = 0; // variable to track photo count
const int maxPhotos = ***Fill_In_Here***; // variable to track max photo count
unsigned long lastPhotoTime = 0; // variable to track last photo time
const unsigned long photoInterval = ***Fill_In_Here***; // 10 seconds in ms

void setup() {
  Serial.begin(115200);
  Serial.println("\nBooting...");

  // Disable brownout detector
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  // Use JPEG format for saving images
  config.pixel_format = PIXFORMAT_JPEG;

  // Adjust resolution depending on whether PSRAM is available
  if (psramFound()) {
    Serial.println("PSRAM detected → using SXGA (1280x1024), quality=25, 2 frame buffers.");
    config.frame_size = FRAMESIZE_SXGA;
    config.jpeg_quality = 25;
    config.fb_count = 2;
    config.xclk_freq_hz = 15000000;
  } else {
    Serial.println("No PSRAM → using VGA (640x480), quality=25, 1 frame buffer.");
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 25;
    config.fb_count = 1;
    config.xclk_freq_hz = 10000000;
  }

  // Initialize the camera
  if (esp_camera_init(&config) != ESP_OK) {
    Serial.println("Camera init failed!");
    blinkErrorAndRestart();
  }
  Serial.println("Camera initialized.");

  // Initialize SD card
  if (!SD_MMC.begin()) {
    Serial.println("SD Card Mount Failed!");
    blinkErrorAndRestart();
  }
  Serial.println("SD Card initialized.");

  // Photo detection logic to prevent overwrites
  ***Fill_In_Here*** = getNextPhotoNumber();
  Serial.printf("Starting at photo %d\n", photoCount + 1);

  lastPhotoTime = millis(); // Start timer
}

void loop() {
  unsigned long currentTime = millis();

  if (photoCount < maxPhotos && (currentTime - lastPhotoTime >= photoInterval)) {
    lastPhotoTime = currentTime; // Reset timer

    Serial.printf("Capturing photo %d...\n", photoCount + 1);
    camera_fb_t *fb = esp_camera_fb_get(); // photo sequence
    if (!fb) {
      Serial.println("Capture failed!");
      ***Fill_In_Here*** // Call restart function
    }

    String path = "/photo_" + String(photoCount + 1) + ".jpg";
    File file = SD_MMC.open(path.c_str(), FILE_WRITE);
    if (!file) {
      Serial.println("Failed to write to SD card.");
      ***Fill_In_Here*** // Call restart function
    } else {
      file.write(fb->buf, fb->len);
      file.close();
      Serial.printf("Saved: %s, size: %u bytes\n", path.c_str(), fb->len);
    }

    esp_camera_fb_return(fb);
    photoCount++;
  }

  if (photoCount >= maxPhotos) {
    Serial.println("All photos taken. Halting.");
    while (true); // Stop execution
  }
}

// Blink flash 5 times, then restart
void blinkErrorAndRestart() {
  pinMode(FLASH_LED_PIN, OUTPUT);
  for (int i = 0; i < 5; i++) {
    digitalWrite(FLASH_LED_PIN, HIGH);
    delay(200);
    digitalWrite(FLASH_LED_PIN, LOW);
    delay(200);
  }
  Serial.println("Restarting ESP32...");
  delay(500);
  ESP.restart();
}

// Find the next available photo number at startup
int getNextPhotoNumber() {
  int count = 0;
  while (true) {
    String path = "/photo_" + String(count + 1) + ".jpg";
    if (!SD_MMC.exists(path.c_str())) {
      return count; // last existing photo index
    }
    count++;
  }
}