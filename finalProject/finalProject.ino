#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 16, 2);
#include "DHT.h"

//DHT
#define DHT_PIN 5
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
float temperature, humidity; 

//Adxl
const uint8_t X_PIN = A0;
const uint8_t Y_PIN = A1;
const uint8_t Z_PIN = A3;

int16_t x_axis = 0;
int16_t y_axis = 0;
int16_t z_axis = 0;
int16_t delta_x = 0;
int16_t delta_y = 0;
int16_t delta_z = 0;

const uint8_t VIBRATION_DELAY = 2;
const uint8_t DE_VIBRATION_DELAY = 75;
const uint8_t SENSITIVITY = 20;
const uint32_t DEBOUNCE_TIME = 2000;

uint32_t last_detection_time = 0;
uint32_t impact_time = 0;
bool impact_detected = false;
String impactMessage ;

//Gps
//float latitude = 3.169200;
//float longitude = 3.112000;

//Fci Location
float latitude ;
float longitude;


//int RXPin = 4;
//int TXPin = 3;
//int GPSBaud = 115200;
//TinyGPSPlus gps;
//SoftwareSerial gpsSerial(RXPin, TXPin);

//Flame
int flame_sensor = 6;
int flame_detected;

//Buzzer
int buzzer = 7;

//I2C Connection
const uint8_t ESP_SLAVE_ADDR = 8;
const uint8_t DATA_LEN = 6;

void setup() {

  Serial.begin(115200);
  Wire.begin();

  //LCD
  lcd.init();
  lcd.backlight();
  displayProjectName();

  //Adxl335
  pinMode(X_PIN, INPUT);
  pinMode(Y_PIN, INPUT);
  pinMode(Z_PIN, INPUT);

  //Gps
  //  gpsSerial.begin(GPSBaud);

  //Flame
  pinMode(flame_sensor, INPUT);

  //Buzzer
  pinMode(buzzer, OUTPUT);

  //DHT
  dht.begin();


}

void loop() {
//    latitude++;
//    longitude++;

  latitude = 30.586100;
  longitude = 31.523242;

  byte latitudeBytes[DATA_LEN];
  memcpy(latitudeBytes, &latitude, sizeof(latitudeBytes));
  byte longitudeBytes[DATA_LEN];
  memcpy(longitudeBytes, &longitude, sizeof(longitudeBytes));

  sendDataToESP32(latitudeBytes, longitudeBytes, impactMessage);

  if (micros() - last_detection_time > DEBOUNCE_TIME) {
    detect_impact();
  }
  if (impact_detected) {

    Serial.println("Impact detected!");
    Serial.print("Magnitude:");
    Serial.println(magnitude());
    Serial.print("Impact time: ");
    Serial.println(impact_time);
    impactMessage = "true";


    sendDataToESP32(latitudeBytes, longitudeBytes, impactMessage);

    digitalWrite(buzzer, HIGH);
    delay(500);
    noTone(buzzer);

    displayAccLcd();
    lcd.clear();
    displayProjectName();


    impact_detected = false;
    last_detection_time = micros();
  }
  else {
    impactMessage = "false";
    Serial.println(impactMessage);

    Serial.println("No acceleration detected");
    delay(1000);
  }


  flame_detected = digitalRead(flame_sensor);
  Serial.println(flame_detected);

  if (flame_detected == 0) {
    flameDetected();
  } else {
    noFlameDetected();
  }

  sendDataToESP32(latitudeBytes, longitudeBytes, impactMessage);

  temperature = getTemperature();
  humidity = getHumidity();
  delay(1000);
  lcd.clear();
  displayTemperature(temperature);
  displayHumidity(humidity);
  delay(1100);
  lcd.clear();
  displayProjectName();
  delay(1000);
}

void detect_impact()
{
  int16_t old_x = x_axis;
  int16_t old_y = y_axis;
  int16_t old_z = z_axis;

  x_axis = analogRead(X_PIN);
  y_axis = analogRead(Y_PIN);
  z_axis = analogRead(Z_PIN);

  delta_x = x_axis - old_x;
  delta_y = y_axis - old_y;
  delta_z = z_axis - old_z;

  if (magnitude() >= SENSITIVITY) {
    impact_detected = true;
    impact_time = millis();
  }
}

////////////////////////////////////////

int16_t magnitude()
{
  return sqrt(sq(delta_x) + sq(delta_y) + sq(delta_z));
}

////////////////////////////////////////

//void displayInfo()
//{
//  Latitude = (gps.location.lat(), 6);
//  Longitude = (gps.location.lng(), 6);
//
//  if (gps.location.isValid())
//  {
//    Serial.print("Latitude: ");
//    Serial.println(gps.location.lat(), 6);
//    Serial.print("Longitude: ");
//    Serial.println(gps.location.lng(), 6);
//    Serial.print("Altitude: ");
//    Serial.println(gps.altitude.meters());
//  }
//  else
//  {
//    Serial.println("Location: Not Available");
//  }
//
//  Serial.print("Date: ");
//  if (gps.date.isValid())
//  {
//    Serial.print(gps.date.month());
//    Serial.print("/");
//    Serial.print(gps.date.day());
//    Serial.print("/");
//    Serial.println(gps.date.year());
//  }
//  else
//  {
//    Serial.println("Not Available");
//  }
//
//  Serial.print("Time: ");
//  if (gps.time.isValid())
//  {
//    if (gps.time.hour() < 10) Serial.print(F("0"));
//    Serial.print(gps.time.hour());
//    Serial.print(":");
//    if (gps.time.minute() < 10) Serial.print(F("0"));
//    Serial.print(gps.time.minute());
//    Serial.print(":");
//    if (gps.time.second() < 10) Serial.print(F("0"));
//    Serial.print(gps.time.second());
//    Serial.print(".");
//    if (gps.time.centisecond() < 10) Serial.print(F("0"));
//    Serial.println(gps.time.centisecond());
//  }
//  else
//  {
//    Serial.println("Not Available");
//  }
//
//  Serial.println();
//  Serial.println();
//  delay(1000);
//}

////////////////////////////////////////
void flameDetected() {
  Serial.println("Flame detected.");

  digitalWrite(buzzer, HIGH);
  displayFlameLcd();
  lcd.clear();
  displayProjectName();
  noTone(buzzer);

}

////////////////////////////////////////

void noFlameDetected() {
  Serial.println("No flame detected.");
  digitalWrite(buzzer, LOW);
}

////////////////////////////////////////

void sendDataToESP32(byte *latitudeBytes, byte *longitudeBytes, String impactMessage ) {

  Wire.beginTransmission(ESP_SLAVE_ADDR);
  Wire.write(latitudeBytes, DATA_LEN);
  Wire.write(longitudeBytes, DATA_LEN);
  Wire.write(impactMessage.c_str());
  uint8_t transmissionStatus = Wire.endTransmission();

  switch (transmissionStatus) {
    case 0:
      Serial.println("Transmission successful");
      break;
    case 1:
      Serial.println("Transmission error: data too long");
      break;
    case 2:
      Serial.println("Transmission error: NACK on address");
      break;
    case 3:
      Serial.println("Transmission error: NACK on data");
      break;
    default:
      Serial.println("Transmission error: other error");
      break;
  }
}


////////////////////////////////////////

void displayProjectName()
{
  lcd.setCursor(4, 0);
  lcd.print("Accident");
  lcd.setCursor(0, 1);
  lcd.print("Detection");
  lcd.setCursor(10, 1);
  lcd.print("System");
}

////////////////////////////////////////

void displayAccLcd()
{

  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print("Accident");
  lcd.setCursor (6, 1);
  lcd.print("Detection");
  delay(1500);
}

////////////////////////////////////////

void displayFlameLcd()
{
  lcd.clear();
  lcd.setCursor (1, 0);
  lcd.print("Fire");
  lcd.setCursor (7, 1);
  lcd.print("Detected");
  delay(1500);
}


float getTemperature() {
  return dht.readTemperature();
}

////////////////////////////////////////

float getHumidity() {
  return dht.readHumidity();
}

////////////////////////////////////////

void displayTemperature(float temperature) {
  lcd.setCursor(0, 0);
  lcd.print("Temp: ");
  lcd.print(temperature);
  lcd.print(" C");
}

////////////////////////////////////////

void displayHumidity(float humidity) {
  lcd.setCursor(0, 1);
  lcd.print("Humidity: ");
  lcd.print(humidity);
  lcd.print("%");
}
