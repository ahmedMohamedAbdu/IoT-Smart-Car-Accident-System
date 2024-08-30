#include <Wire.h>
#include <WiFi.h>
#include <FirebaseESP32.h>

//#define WIFI_SSID "Vodafone_VDSL_51A0"
//#define WIFI_PASSWORD "YQXNV5HLQLCHJWNH"

#define WIFI_SSID "KIRA"
#define WIFI_PASSWORD "123456789"

//#define API_KEY "AIzaSyBk6p5lkddvNH_vNb7l8MTV-scCZjzDGJ8"
//#define DATABASE_URL "https://caraccidentesp8266-default-rtdb.firebaseio.com/"

#define FIREBASE_HOST "https://roadhelper-90bab-default-rtdb.firebaseio.com/"
#define FIREBASE_AUTH "AIzaSyDOUCRViVfnB6lja1ad2TjfqFgPtgpVf8s‏"


FirebaseData fbdo;

int byteCount;

float latitude;
float longitude;
String Location;
String Lat;
String Long;

String impactMessage;

const uint8_t I2C_SLAVE_ADDRESS = 8;
const uint8_t DATA_LEN = 6;
const uint8_t MESSAGE_LEN = 20;

void setup()
{
  Serial.begin(115200);
  Wire.begin(I2C_SLAVE_ADDRESS);
  Wire.onReceive(receiveData);

  WiFi.begin (WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting...");

  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
}


void loop()
{
  delay(1000);
  receiveData(byteCount);
  delay(2000);

  Location = "https://www.google.com/maps/dir//" + String(latitude, 6) + "," + String(longitude, 6);
  Lat  = String(latitude, 6);
  Long = String(longitude, 6);
  delay(1000);

  postDataToFirebase();
  delay(1100);
}

void postDataToFirebase()
{
  Serial.printf("Set Latitude... %s\n", Firebase.setString(fbdo, "/data/Latitude", Lat) ? "ok" : fbdo.errorReason().c_str());
  Serial.printf("Set Longitude... %s\n", Firebase.setString(fbdo, "/data/Longitude", Long) ? "ok" : fbdo.errorReason().c_str());
  Serial.printf("Set Location... %s\n", Firebase.setString(fbdo, "/data/Location", Location) ? "ok" : fbdo.errorReason().c_str());
  Serial.printf("Set Impact Message... %s\n", Firebase.setString(fbdo, "/data/Impact Message", impactMessage) ? "ok" : fbdo.errorReason().c_str());
}

void receiveData(int byteCount) {

  byte latitudeBytes[DATA_LEN];
  byte longitudeBytes[DATA_LEN];
  char impactMessageBuffer[MESSAGE_LEN];

  for (int i = 0; i < DATA_LEN; i++) {
    latitudeBytes[i] = Wire.read();
  }
  memcpy(&latitude, latitudeBytes, sizeof(latitude));

  for (int i = 0; i < DATA_LEN; i++) {
    longitudeBytes[i] = Wire.read();
  }
  memcpy(&longitude, longitudeBytes, sizeof(longitude));

  int i = 0;
  while (Wire.available()) {
    impactMessageBuffer[i] = Wire.read();
    i++;
  }
  impactMessageBuffer[i] = '\0';
  impactMessage = String(impactMessageBuffer);

  Serial.print("Latitude: ");
  Serial.println(latitude, 6);
  Serial.print("Longitude: ");
  Serial.println(longitude, 6);
  Serial.print("Impact Message: ");
  Serial.println(impactMessage);

}
