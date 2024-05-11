#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_ADXL345_U.h>

// WiFi settings
const char* ssid = "YOUR_SSID";
const char* password = "YOUR_PASSWORD";

// Pins
const int pulsePin = A0;

// BMP180 Pressure Sensor
Adafruit_BMP085 bmp;

// ADXL345 Accelerometer
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup() {
  Serial.begin(9600);
  delay(100);

  // Connect to WiFi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("WiFi connected");

  // Initialize sensors
  if (!bmp.begin()) {
    Serial.println("Could not find BMP180 sensor, check wiring!");
    while (1);
  }
  Serial.println("BMP180 sensor found");

  if (!accel.begin()) {
    Serial.println("Could not find a valid ADXL345 sensor, check wiring!");
    while (1);
  }
  Serial.println("ADXL345 sensor found");

  // Set accelerometer range
  accel.setRange(ADXL345_RANGE_16_G);
}

void loop() {
  // Read sensor data
  float temperature = bmp.readTemperature();
  float pressure = bmp.readPressure();
  int pulseValue = analogRead(pulsePin); // Read pulse sensor value
  sensors_event_t event;
  accel.getEvent(&event);
  float accelerationX = event.acceleration.x;
  float accelerationY = event.acceleration.y;
  float accelerationZ = event.acceleration.z;

  // Print sensor data
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" *C");
  Serial.print("Pressure: ");
  Serial.print(pressure);
  Serial.println(" Pa");
  Serial.print("Pulse Sensor Value: ");
  Serial.println(pulseValue);
  Serial.print("Acceleration (X,Y,Z): ");
  Serial.print(accelerationX);
  Serial.print(", ");
  Serial.print(accelerationY);
  Serial.print(", ");
  Serial.print(accelerationZ);
  Serial.println(" m/s^2");

  // Wait before next reading
  delay(1000);
}
