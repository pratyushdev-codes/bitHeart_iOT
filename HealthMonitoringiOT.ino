#include <Wire.h>
#include <Adafruit_BMP085.h>
#include <PulseSensorPlayground.h>
#include <Adafruit_ADXL345_U.h>

// Define Arduino Interrupts for PulseSensor
#define USE_ARDUINO_INTERRUPTS true

// On-board LED PIN
const int LED_PIN = 13;

// Threshold for detecting a heartbeat
const int THRESHOLD = 550;

// Heart Rate Pulse Sensor
const int PULSE_SENSOR_PIN = A0;
PulseSensorPlayground pulseSensor;

// BMP180 Pressure Sensor
// Adafruit_BMP085 bmp;

// ADXL345 Accelerometer
const int xpin = A3;
const int ypin = A2;
const int zpin = A1;

// Analog pins used for power and ground
const int groundpin = 18; // analog input pin 4 -- ground
const int powerpin = 19;  // analog input pin 5 -- voltage

// Previous y-axis value
int prevY = 0;

// Step count
int stepCount = 0;

void setup() {
  Serial.begin(9600);

  // Initialize BMP180
  // if (!bmp.begin()) {
  //   Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  //   while (1);
  // }

  // Initialize PulseSensor
  pulseSensor.analogInput(PULSE_SENSOR_PIN);
  pulseSensor.blinkOnPulse(LED_PIN);
  pulseSensor.setThreshold(THRESHOLD);

  // Check if PulseSensor is initialized
  if (pulseSensor.begin()) {
    Serial.println("PulseSensor object created successfully!");
  }

  // Provide ground and power by using the analog inputs as normal digital pins.
  pinMode(groundpin, OUTPUT);
  pinMode(powerpin, OUTPUT);
  digitalWrite(groundpin, LOW);
  digitalWrite(powerpin, HIGH);
}

void loop() {
  // Read and print BMP180 sensor data
  // Serial.print("Temperature = ");
  // Serial.print(bmp.readTemperature());
  // Serial.println(" *C");
  // Serial.print("Pressure = ");
  // Serial.print(bmp.readPressure());
  // Serial.println(" Pa");
  // Serial.print("Altitude = ");
  // Serial.print(bmp.readAltitude());
  // Serial.println(" meters");
  // Serial.print("Pressure at sealevel (calculated) = ");
  // Serial.print(bmp.readSealevelPressure());
  // Serial.println(" Pa");
  // Serial.print("Real altitude = ");
  // Serial.print(bmp.readAltitude(bmp.readSealevelPressure() / 100));
  // Serial.println(" meters");

  // Read and print accelerometer sensor data
  int yValue = analogRead(ypin);
  // Serial.print("Y-axis = ");
  // Serial.println(yValue);

  // Check for step
  if (isStep(yValue)) {
    stepCount++;
    Serial.print("Step Count: ");
    Serial.println(stepCount);
    // Optionally, you can toggle an LED or perform other actions when a step is detected
    digitalWrite(LED_PIN, HIGH); // Turn on LED
    delay(100); // LED on time
    digitalWrite(LED_PIN, LOW); // Turn off LED
  }

  // Check for heartbeat and print BPM
  int currentBPM = pulseSensor.getBeatsPerMinute();
  if (pulseSensor.sawStartOfBeat()) {
    Serial.println("♥ A HeartBeat Happened!");
    Serial.print("BPM: ");
    Serial.println(currentBPM);
  }

  // Add delay
  delay(1000);
}

// Function to check if a step is detected
bool isStep(int yValue) {
  // Define a threshold value for step detection
  int threshold = 100;

  // Check if the current y-axis value is greater than the threshold
  // and the previous y-axis value was less than the threshold
  if (yValue > threshold && prevY <= threshold) {
    // Update the previous y-axis value
    prevY = yValue;
    return true; // Step detected
  }

  // Update the previous y-axis value
  prevY = yValue;
  return false; // No step detected
}
