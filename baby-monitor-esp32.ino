/*
   Arduino IDE v1.8.13
*/

/*
   Libraries - Communications
   WiFi comes within the ESP32 library
   Firebase ESP32 Client by Mobizt v3.7.6
*/
#include <WiFi.h>
#include <FirebaseESP32.h>

/*
   Libraries - Accelerometer
   Adafruit LIS3DH v1.2.0
   Adafruit Unified Sensor v1.1.4
   Adafruit BusIO v1.5.0
*/
#include <SPI.h>
#include <Adafruit_LIS3DH.h>
#include <Adafruit_Sensor.h>

/*
   Libraries - Thermometer
   Adafruit MLX90614 Library v1.1.0
*/
#include <Wire.h>
#include <Adafruit_MLX90614.h>

/*
    Credentials for Wi-Fi and Firebase
*/
#include "Credentials.h"

// Accelerometer pins
#define LIS3DH_CLK 18
#define LIS3DH_MISO 19
#define LIS3DH_MOSI 23
#define LIS3DH_CS 5

// Thermometer pins
#define I2C_SDA_PIN 17
#define I2C_SCL_PIN 22

// Sensor objects
Adafruit_MLX90614 therm = Adafruit_MLX90614();
Adafruit_LIS3DH accel = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// Thermometer and Accelerometer Variables
unsigned long nextTimeTherm = 30000;  // Do this every second or 30 000 milliseconds
unsigned long nextTimeAccel = 200;   // Do this every second or 200 milliseconds

unsigned long goTimeTherm;
unsigned long goTimeAccel;

// The Firebase RT Database Paths
#define RT_DATABASE_NODE_ID String("Node-02")
#define RT_DATABASE_CLIENT_TOKENS RT_DATABASE_NODE_ID + "/ClientTokens"
#define RT_DATABASE_THERMOMETER_READINGS RT_DATABASE_NODE_ID + "/ThermometerReadings"
#define RT_DATABASE_THERMOMETER_LAST_READING RT_DATABASE_NODE_ID + "/LastThermometerReading"
#define RT_DATABASE_ACCELEROMETER_READINGS RT_DATABASE_NODE_ID + "/AccelerometerReadings"
#define RT_DATABASE_THERMOMETER_LAST_SLEEP_STATE RT_DATABASE_NODE_ID + "/LastSleepState"

// Firebase Variables
float currentTemp = 0;

// Firebase connection object
FirebaseData firebaseData;

// Firebase realtime database objects
FirebaseJson temps;
FirebaseJson accels;

void setup() {
  Serial.begin(115200);
  goTimeTherm = millis();
  goTimeAccel = millis();

  // Start the thermometer
  Serial.println("Starting thermometer (I2C)");
  therm.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // Start the accelerometer
  Serial.println("Starting accelerometer (SPI)");
  if (! accel.begin()) {   // change this to 0x18 or 0x19 to use i2c or leave blank for spi
    Serial.println("Couldnt start accelerometer");
    while (1) yield();
  }

  // Connect to Wi-Fi
  connectToWiFi();

  // Setup the NTP server for date and time requests
  setupNTPServer();

  // Setup the Firebase connection
  setupFirebaseConnection();
}

void loop() {
  readThermometerValue();
  readAccelerometerValues();
}

/*
   Read the thermometer values and trigger events if the temperature is too high or too low
*/
void readThermometerValue() {
  if (millis() >= goTimeTherm) {
    currentTemp = therm.readObjectTempC();

    // Print in Console
    goTimeTherm = millis() + nextTimeTherm;
    Serial.println("  ");
    Serial.println("Leitura de Temperatura: ");
    Serial.print("Temperatura Ambiente = "); Serial.print(therm.readAmbientTempC());
    Serial.print("ºC\tTemperatura do Objeto = "); Serial.print(currentTemp); Serial.println("*C");
    Serial.print("Temperatura Ambiente = "); Serial.print(therm.readAmbientTempF());
    Serial.print("*F\tTemperatura do Objeto = "); Serial.print(therm.readObjectTempF()); Serial.println("*F");
    Serial.println();

    // Save the current reading in the database list
    temps.set("temp", currentTemp);
    temps.set("timestamp", getTimestampUTC());
    pushFirebaseEntry(RT_DATABASE_THERMOMETER_READINGS, accels);

    // Save the current reading as the last reading
    pushFirebaseFloatValue(RT_DATABASE_THERMOMETER_LAST_READING, currentTemp);
  }
}

/*
   Read the accelerometer values and trigger events as necessary
*/
void readAccelerometerValues() {
  if (millis() >= goTimeAccel) {
    goTimeAccel = millis() + nextTimeAccel;

    // Get X Y and Z data at once
    accel.read();

    // Get a new sensor event, normalized
    sensors_event_t event;
    accel.getEvent(&event);

    // Display the results in m/s^2
    Serial.print("Aceleração:");
    Serial.print("\tEixo oX: "); Serial.print(event.acceleration.x);
    Serial.print("\tEixo oY: "); Serial.print(event.acceleration.y);
    Serial.print("\tEixo oZ: ");
    Serial.print(event.acceleration.z);
    Serial.print("\t(m/s^2)");
    Serial.println();

    // Save the current reading in the database list
    accels.set("x_axis", event.acceleration.x);
    accels.set("y_axis", event.acceleration.y);
    accels.set("z_axis", event.acceleration.z);
    accels.set("timestamp", getTimestampUTC());
    pushFirebaseEntry(RT_DATABASE_ACCELEROMETER_READINGS, temps);
  }
}

/*
   Connect to the Wi-Fi network defined in the Credentials file
*/
void connectToWiFi() {
  // Set the Wi-Fi credentials
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  // Try to connect with a delay
  while (WiFi.status() != WL_CONNECTED) {
    delay(5000);
    Serial.println("Connecting to WiFi..");
  }

  Serial.println("Connected to the WiFi network");

  Serial.print("IP: ");
  Serial.print(WiFi.localIP());
  Serial.print(" MAC: ");
  Serial.println(WiFi.macAddress());
}

/*
   Start the firebase connection and the necessary properties
*/
void setupFirebaseConnection() {
  // Setup Firebase credentials
  Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);

  // Optional, set AP reconnection in setup()
  Firebase.reconnectWiFi(true);

  // Optional, set number of error retry
  Firebase.setMaxRetry(firebaseData, 3);

  // Optional, set number of error resumable queues
  Firebase.setMaxErrorQueue(firebaseData, 30);

  // Optional, use classic HTTP GET and POST requests.
  // This option allows get and delete functions (PUT and DELETE HTTP requests) works for
  // device connected behind the Firewall that allows only GET and POST requests.
  Firebase.enableClassicRequest(firebaseData, true);
}

/*
   Setup the ntp server for date and time requests with a couple of servers
*/
void setupNTPServer() {
  // Configure the NTP server with the GMT offset, daylight offset and servers
  configTime(0, 0, "pool.ntp.org", "time.nist.gov");
}

/*
   Push a json object to the given database path in Firebase
*/
void pushFirebaseEntry(const String &databasePath, FirebaseJson &jsonToPush) {
  if (Firebase.pushJSON(firebaseData, databasePath, jsonToPush)) {
    //Serial.println(firebaseData.dataPath() + "/" + firebaseData.pushName());
  } else {
    Serial.println(firebaseData.errorReason());
  }
}

/*
   Set the float value into the given database path in Firebase
*/
void pushFirebaseFloatValue(const String &databasePath, float floatValue) {
  if (Firebase.setFloat(firebaseData, databasePath, floatValue)) {
    //Serial.println(firebaseData.dataPath() + "/" + firebaseData.pushName());
  } else {
    Serial.println(firebaseData.errorReason());
  }
}

/*
   Get the UTC date and time in the format YYYYMMDDHHMMSS
*/
String getTimestampUTC() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    return "";
  }

  //Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  //Serial.printf("Now is : %d-%02d-%02d %02d:%02d:%02d\n", (timeinfo.tm_year) + 1900, ( timeinfo.tm_mon) + 1, timeinfo.tm_mday, timeinfo.tm_hour , timeinfo.tm_min, timeinfo.tm_sec);

  char finalTimestamp[15];
  sprintf(
    finalTimestamp,
    "%d%02d%02d%02d%02d%02d",
    (timeinfo.tm_year) + 1900,
    (timeinfo.tm_mon) + 1,
    timeinfo.tm_mday,
    timeinfo.tm_hour,
    timeinfo.tm_min,
    timeinfo.tm_sec
  );

  String finalTimestampString = finalTimestamp;

  return finalTimestampString;
}
