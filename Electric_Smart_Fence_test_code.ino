#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <WiFi.h>
#include <HTTPClient.h>

#define MPU6050_ADDRESS_1 0x68  // MPU6050 I2C address for sensor 1
#define MPU6050_ADDRESS_2 0x69  // MPU6050 I2C address for sensor 2
#define MPU6050_ADDRESS_3 0x6A  // MPU6050 I2C address for sensor 3
#define MPU6050_ADDRESS_4 0x6B  // MPU6050 I2C address for sensor 4
#define PIR_PIN_1 2            // PIR sensor pin for side 1
#define PIR_PIN_2 3            // PIR sensor pin for side 2
#define PIR_PIN_3 4            // PIR sensor pin for side 3
#define PIR_PIN_4 5            // PIR sensor pin for side 4

Adafruit_MPU6050 mpu1, mpu2, mpu3, mpu4;
SoftwareSerial gpsSerial(8, 9);  // RX, TX
TinyGPSPlus gps;

const int relayPin = 7;          // Relay control pin for electrifying the fence
const int laserPin = 6;          // Laser control pin
const int ldrPin = A0;            // LDR pin
const int thermalCameraPin = 11;  // Pin to trigger thermal camera

// WiFi settings
const char* ssid = "your_SSID";
const char* password = "your_PASSWORD";

// Google Drive API settings
const char* googleDriveAPIKey = "your_google_drive_api_key";
const char* googleDriveFolderID = "your_google_drive_folder_id";

// Email settings
const char* emailServer = "smtp.your-email-provider.com";
const int emailPort = 587;  // Port for your email server
const char* emailSender = "your_email@gmail.com";
const char* emailRecipient = "recipient_email@example.com";
const char* emailSubject = "Intrusion Alert!";
const char* emailBody = "Intrusion detected at Lat: %f, Lng: %f. View image here: %s";

float threshold = 1.5;  // Adjust the threshold for motion detection as needed

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  pinMode(laserPin, OUTPUT);
  pinMode(ldrPin, INPUT);
  pinMode(relayPin, OUTPUT);
  pinMode(thermalCameraPin, OUTPUT);
  pinMode(ledPin, OUTPUT);

  pinMode(PIR_PIN_1, INPUT);
  pinMode(PIR_PIN_2, INPUT);
  pinMode(PIR_PIN_3, INPUT);
  pinMode(PIR_PIN_4, INPUT);

  if (!mpu1.begin(MPU6050_ADDRESS_1) || !mpu2.begin(MPU6050_ADDRESS_2) ||
      !mpu3.begin(MPU6050_ADDRESS_3) || !mpu4.begin(MPU6050_ADDRESS_4)) {
    Serial.println("Failed to find one or more MPU6050 chips");
    while (1);
  }

  // Connect to Wi-Fi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");
}

void loop() {
  // Read MPU6050 data for sensor 1
  sensors_event_t a1, g1, temp1;
  mpu1.getEvent(&a1, &g1, &temp1);

  // Read MPU6050 data for sensor 2
  sensors_event_t a2, g2, temp2;
  mpu2.getEvent(&a2, &g2, &temp2);

  // Read MPU6050 data for sensor 3
  sensors_event_t a3, g3, temp3;
  mpu3.getEvent(&a3, &g3, &temp3);

  // Read MPU6050 data for sensor 4
  sensors_event_t a4, g4, temp4;
  mpu4.getEvent(&a4, &g4, &temp4);

  // Read GPS data
  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print("Location: ");
        Serial.print(gps.location.lat(), 6);
        Serial.print(", ");
        Serial.println(gps.location.lng(), 6);
      }
    }
  }

  // Check if any MPU6050 detects excessive motion
  if (a1.acceleration.x > threshold || a1.acceleration.y > threshold || a1.acceleration.z > threshold ||
      a2.acceleration.x > threshold || a2.acceleration.y > threshold || a2.acceleration.z > threshold ||
      a3.acceleration.x > threshold || a3.acceleration.y > threshold || a3.acceleration.z > threshold ||
      a4.acceleration.x > threshold || a4.acceleration.y > threshold || a4.acceleration.z > threshold) {
    // Fence motion detected, trigger actions
    alertActions(gps.location.lat(), gps.location.lng());
  }

  // Check laser and LDR
  if (digitalRead(ldrPin) == HIGH) {
    // Laser reached LDR, no intrusion
  } else {
    // Laser blocked, trigger actions
    alertActions(gps.location.lat(), gps.location.lng());
  }

  // Check PIR sensors and trigger thermal camera
  if (digitalRead(PIR_PIN_1) == HIGH || digitalRead(PIR_PIN_2) == HIGH ||
      digitalRead(PIR_PIN_3) == HIGH || digitalRead(PIR_PIN_4) == HIGH) {
    // PIR sensor triggered, activate thermal camera
    digitalWrite(thermalCameraPin, HIGH);
    delay(500);  // Assuming the thermal camera needs some time to capture an image
    sendImageToGoogleDrive();
    digitalWrite(thermalCameraPin, LOW);
  }
}

void alertActions(float lat, float lng) {
  // Perform actions on intrusion detection
  Serial.println("Intrusion detected!");

  // Activate relay to electrify the fence
  digitalWrite(relayPin, HIGH);

  // Send alert message with GPS location via email
  sendEmail(lat, lng);

  delay(5000);  // Electrify the fence for 5 seconds 

  // Deactivate relay
  digitalWrite(relayPin, LOW);

  // Reset MPU6050 interrupts
  mpu1.resetFIFO();
  mpu2.resetFIFO();
  mpu3.resetFIFO();
  mpu4.resetFIFO();
}

void sendEmail(float lat, float lng) {
  WiFiClient client;
  if (client.connect(emailServer, emailPort)) {
    client.print("EHLO arduino\r\n");
    delay(500);
    client.print("AUTH LOGIN\r\n");
    delay(500);
    // Your Base64-encoded email and password (use an online tool to encode)
    client.print("your_base64_encoded_email\r\n");
    delay(500);
    client.print("your_base64_encoded_password\r\n");
    delay(500);
    client.print("MAIL FROM:<" + String(emailSender) + ">\r\n");
    delay(500);
    client.print("RCPT TO:<" + String(emailRecipient) + ">\r\n");
    delay(500);
    client.print("DATA\r\n");
    delay(500);
    client.print("Subject: " + String(emailSubject) + "\r\n");
    client.print("From: " + String(emailSender) + "\r\n");
    client.print("To: " + String(emailRecipient) + "\r\n");
    client.print("Content-Type: text/plain\r\n\r\n");
    client.print("Intrusion detected at Lat: ");
    client.print(lat, 6);
    client.print(", Lng: ");
    client.println(lng, 6);
    client.println(".");
    delay(500);
    client.print("QUIT\r\n");
    delay(500);
    client.stop();
    Serial.println("Email sent!");
  } else {
    Serial.println("Failed to connect to email server");
  }
}

void sendImageToGoogleDrive() 
{
  // Implementation for sending image to Google Drive (use previous Google Drive sending logic)
 
}
