#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

Adafruit_MPU6050 mpu;

// Define thresholds
const float ACCIDENT_ACCEL_THRESHOLD = 15.0; // Acceleration change threshold in m/s^2
const float ACCIDENT_ORIENTATION_THRESHOLD = 0.5; // Orientation shift threshold in rad/s

// WiFi credentials
const char* ssid = "";
const char* password = "";

// WhatsApp configuration
String phoneNumber = ""; // Your WhatsApp phone number (in international format)
String apiKey = ""; // CallMeBot API key

// GPS setup
TinyGPSPlus gps;
HardwareSerial gpsSerial(2); // Use Serial2 for GPS communication
#define RXD2 16
#define TXD2 17

// Variables for GPS data
double latitude = 0.0;
double longitude = 0.0;

void setup(void) {
  Serial.begin(115200);
  
  // Initialize I2C with SDA on pin 21 and SCL on pin 22
  Wire.begin(21, 22); 

  // Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  // Set accelerometer range and gyro range
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  // Initialize GPS
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("GPS module initialized");

  // Connect to WiFi
  connectToWiFi();

  delay(100);
}

void loop() {
  // Get new sensor events with the readings
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // Monitor acceleration changes
  float total_acceleration = sqrt(a.acceleration.x * a.acceleration.x +
                                  a.acceleration.y * a.acceleration.y +
                                  a.acceleration.z * a.acceleration.z);

  // Monitor orientation shifts
  bool overturned = (a.acceleration.y < 0 && a.acceleration.z < 0); // Detect if the car is overturned

  // Check for a significant acceleration spike (possible crash)
  if (total_acceleration > ACCIDENT_ACCEL_THRESHOLD) {
    Serial.println("Crash detected due to sharp acceleration!");
    getGPSData();
    sendAccidentAlert("Crash detected due to sharp acceleration!", total_acceleration);
  }

  // Check for an overturn
  if (overturned) {
    Serial.println("Possible overturn detected due to orientation shift!");
    getGPSData();
    sendAccidentAlert("Possible overturn detected due to orientation shift!", total_acceleration);
  }

  // Print out sensor values for debugging
  Serial.print("Acceleration (m/s^2): X=");
  Serial.print(a.acceleration.x);
  Serial.print(", Y=");
  Serial.print(a.acceleration.y);
  Serial.print(", Z=");
  Serial.println(a.acceleration.z);

  Serial.print("Rotation (rad/s): X=");
  Serial.print(g.gyro.x);
  Serial.print(", Y=");
  Serial.print(g.gyro.y);
  Serial.print(", Z=");
  Serial.println(g.gyro.z);

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" °C");

  // Read GPS data
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  // Print GPS data for debugging
  if (gps.location.isUpdated()) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
    Serial.print("Latitude: ");
    Serial.println(latitude, 6);
    Serial.print("Longitude: ");
    Serial.println(longitude, 6);
  } else {
    Serial.println("No GPS data available");
  }

  delay(500);  // Delay for readability
}

void getGPSData() {
  unsigned long start = millis();
  bool newData = false;

  // Wait for GPS data to be available (up to 5 seconds)
  while (millis() - start < 5000) {
    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) {
        newData = true;
      }
    }
  }

  if (newData) {
    latitude = gps.location.lat();
    longitude = gps.location.lng();
  } else {
    Serial.println("Failed to get GPS data.");
  }
}

void sendAccidentAlert(String message, float acceleration) {
  String completeMessage = message + " Acceleration: " + String(acceleration) + " m/s^2. Location: Lat " + String(latitude, 6) + ", Lon " + String(longitude, 6);
  
  Serial.println("Sending WhatsApp alert...");
  sendMessage(completeMessage);
}

void sendMessage(String message) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    String url = "https://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&text=" + message + "&apikey=" + apiKey;

    // Send HTTP GET request
    http.begin(url);
    int httpResponseCode = http.GET();

    // Check response
    if (httpResponseCode > 0) {
      Serial.println("WhatsApp message sent successfully");
    } else {
      Serial.print("Error sending WhatsApp message. Error code: ");
      Serial.println(httpResponseCode);
    }
    http.end(); // Close connection
  } else {
    Serial.println("WiFi not connected");
  }
}

void connectToWiFi() {
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");
}
