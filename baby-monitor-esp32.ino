/*
   Arduino IDE v1.8.13
*/

/*
  Libraries - String parsing
*/
#include <iomanip>
#include <sstream>

/*
  Libraries - List management
  QList v0.6.7
*/
#include <QList.h>

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
#define LIS3DH_CLK    18
#define LIS3DH_MISO   19
#define LIS3DH_MOSI   23
#define LIS3DH_CS     5

// Thermometer pins
#define I2C_SDA_PIN   17
#define I2C_SCL_PIN   22

// Sensor objects
Adafruit_MLX90614 therm = Adafruit_MLX90614();
Adafruit_LIS3DH accel = Adafruit_LIS3DH(LIS3DH_CS, LIS3DH_MOSI, LIS3DH_MISO, LIS3DH_CLK);

// Thermometer and Accelerometer Variables
unsigned long nextTimeTherm = 30000;  // Do this every second or 30 000 milliseconds
unsigned long nextTimeAccel = 200;   // Do this every second or 200 milliseconds

unsigned long goTimeTherm;
unsigned long goTimeAccel;

// The Firebase RT Database Paths
#define RT_DATABASE_NODE_ID String("node_01")
#define RT_DATABASE_CLIENT_TOKENS RT_DATABASE_NODE_ID + "/client_tokens"
#define RT_DATABASE_THERMOMETER_READINGS RT_DATABASE_NODE_ID + "/thermometer_readings"
#define RT_DATABASE_LAST_THERMOMETER_READING RT_DATABASE_NODE_ID + "/last_thermometer_reading"
#define RT_DATABASE_ACCELEROMETER_READINGS RT_DATABASE_NODE_ID + "/accelerometer_readings"
#define RT_DATABASE_THERMOMETER_LAST_SLEEP_STATE RT_DATABASE_NODE_ID + "/last_sleep_state"
#define RT_DATABASE_TEMPERATURE_THRESHOLDS RT_DATABASE_NODE_ID + "/temperature_thresholds"
#define RT_DATABASE_HIGH_TEMP_THRESHOLD_KEY String("high_temp")
#define RT_DATABASE_LOW_TEMP_THRESHOLD_KEY String("low_temp")

// Temperature types of warnings
#define HIGH_TEMP_WARNING_FLAG  1
#define LOW_TEMP_WARNING_FLAG   -1

// Temperature thresholds (changed dynamically from database stream)
float highTempThreshold = 37.5;
float lowTempThreshold = 36.0;

// Temperature variables
float currentTemp = 0;
float currentTempRounded = 0;
String currentTempTimestamp = "";

// Temperature notifications control
#define HIGH_TEMP_NOTIFICATION_SENT 1
#define TEMP_NOTIFICATION_IDLING    0
#define LOW_TEMP_NOTIFICATION_SENT  -1
int currentTempNotificationState = TEMP_NOTIFICATION_IDLING;

// Acceleration variables for deviation calculation
#define MAX_ACCEL_DEVIATION_PERCENT 10.24
#define MAX_ACCEL_VALUES_IN_LIST    5
QList<float> accelList;

// Sleep state notification control
#define MAX_DEVIATION_TIMESTAMPS_IN_LIST  5
#define MIN_DEVIATION_TIME_DIFF_FOR_NOTIF 120000 // 2 minutes in milliseconds
#define SLEEP_STATE_RESET_INTERVAL        180000 // 3 minutes in milliseconds
int lastSleepStateNotificationSentAt = 0;
QList<long> deviationTimestamps;

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
    //while (1) yield();
  }

  // Connect to Wi-Fi
  connectToWiFi();

  // Setup the NTP server for date and time requests
  setupNTPServer();

  // Setup the Firebase connection
  setupFirebaseConnection();

  // Setup the Firebase cloud messaging
  setupFirebaseCloudMessaging();

  // Setup the Firebase realtime database stream
  setupFirebaseStreamCallbacks();

  // Setup the temperature threshold values stream
  setupFirebaseTemperatureThresholdsStream();
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
    currentTempTimestamp = getTimestampUTC();

    // Print in Console
    goTimeTherm = millis() + nextTimeTherm;
    Serial.println("  ");
    Serial.println("Leitura de Temperatura: ");
    Serial.print("Temperatura Ambiente = "); Serial.print(therm.readAmbientTempC());
    Serial.print("ºC\tTemperatura do Objeto = "); Serial.print(currentTemp); Serial.println("*C");
    //Serial.print("Temperatura Ambiente = "); Serial.print(therm.readAmbientTempF());
    //Serial.print("*F\tTemperatura do Objeto = "); Serial.print(therm.readObjectTempF()); Serial.println("*F");
    Serial.println();

    // Save values to database
    saveThermometerValueInDatabase(currentTemp, currentTempTimestamp);

    // Trigger notification if required
    triggerTemperatureNotificationIfRequired(currentTemp);
  }
}

/*
   Save a temperature value and timestamp to the Firebase realtime database
*/
void saveThermometerValueInDatabase(float currentTemp, String currentTempTimestamp) {
  // Save the current reading in the database list
  temps.set("temp", currentTemp);
  temps.set("timestamp", currentTempTimestamp);
  pushFirebaseJsonEntry(RT_DATABASE_THERMOMETER_READINGS, temps);

  // Save the current reading as the last reading
  setFirebaseJsonEntry(RT_DATABASE_LAST_THERMOMETER_READING, temps);
}

/*
   Trigger a Firebase cloud messaging notification if the temperature value reaches a threshold
*/
void triggerTemperatureNotificationIfRequired(float currentTemp) {
  // Round the current temperature to one decimal place for better comparison
  currentTempRounded = round(currentTemp * 10) / 10;

  // Trigger a notification if the current reading exceeds one of the thresholds
  if (currentTempRounded >= highTempThreshold && currentTempNotificationState != HIGH_TEMP_NOTIFICATION_SENT) {
    currentTempNotificationState = HIGH_TEMP_NOTIFICATION_SENT;
    sendTemperatureNotification(currentTempRounded, HIGH_TEMP_WARNING_FLAG);
  } else if (currentTempRounded <= lowTempThreshold && currentTempNotificationState != LOW_TEMP_NOTIFICATION_SENT) {
    currentTempNotificationState = LOW_TEMP_NOTIFICATION_SENT;
    sendTemperatureNotification(currentTempRounded, LOW_TEMP_WARNING_FLAG);
  } else if (currentTempRounded < highTempThreshold && currentTempRounded > lowTempThreshold) {
    currentTempNotificationState = TEMP_NOTIFICATION_IDLING;
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

    // Save values to database
    saveAccelerationValueInDatabase(event.acceleration);

    // Trigger notification if required
    triggerSleepStateNotificationIfRequired(event.acceleration.z);
  }
}

/*
   Save an acceleration value to the Firebase realtime database
*/
void saveAccelerationValueInDatabase(sensors_vec_t &accelEvent) {
  // Save the current reading in the database list
  accels.set("x_axis", accelEvent.x);
  accels.set("y_axis", accelEvent.y);
  accels.set("z_axis", accelEvent.z);
  accels.set("timestamp", getTimestampUTC());
  pushFirebaseJsonEntry(RT_DATABASE_ACCELEROMETER_READINGS, accels);
}

/*
   Trigger a Firebase cloud messaging notification if the current Z Axis value has
   a deviation higher than the max permitted by correlating it to the mean of the
   previous values present in the list
*/
void triggerSleepStateNotificationIfRequired(float zAxisValue) {
  int accelListSize = accelList.size();

  // Only check the deviation if we have the right amount of acceleration values in the list
  if (accelListSize < MAX_ACCEL_VALUES_IN_LIST) {
    accelList.push_front(zAxisValue);
    return;
  }

  float deviation = calculateAccelerationDeviation(zAxisValue, accelListSize);

  // Check if the deviation surpasses the limit
  if (deviation >= MAX_ACCEL_DEVIATION_PERCENT) {
    reactUponSurpassedDeviation(zAxisValue, deviation);
  } else {
    // Remove the last item in the list and insert the current one at the top
    accelList.pop_back();
    accelList.push_front(zAxisValue);

    // Check if enough time has passed since the last disturbance so that we can reset
    // the sleep state in the Firebase realtime database
    if (lastSleepStateNotificationSentAt != 0 && millis() - lastSleepStateNotificationSentAt >= SLEEP_STATE_RESET_INTERVAL) {
      Serial.println();
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.println("Resetting the sleep state");
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.println("+++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
      Serial.println();

      // Reset timestamp
      lastSleepStateNotificationSentAt = 0;

      // Update the sleep state variable in the Firebase realtime database

    }
  }
}

float calculateAccelerationDeviation(float zAxisValue, int accelListSize) {
  // Calculate the mean of the values in the list
  float accelListMean = 0.0;
  for (size_t i = 0; i < accelListSize; i++) {
    accelListMean = accelListMean + accelList.get(i);
  }
  accelListMean = accelListMean / accelListSize;

  // Calculate the current deviation
  float deviation = 100 - (zAxisValue * 100 / accelListMean);

  // Normalize the deviation
  if (deviation < 0) {
    deviation = deviation * -1;
  }

  Serial.println("Deviation is: " + String(deviation) + "%");

  return deviation;
}

void reactUponSurpassedDeviation(float zAxisValue, float deviation) {
  Serial.println();
  Serial.println("*********************************************************");
  Serial.println("*********************************************************");
  Serial.println("Max deviation surpassed: " + String(deviation) + "%");
  Serial.println("*********************************************************");
  Serial.println("*********************************************************");
  Serial.println();

  // Clean the list and start filling it with new values
  accelList.clear();
  accelList.push_front(zAxisValue);

  // Register the current deviation timestamp
  deviationTimestamps.push_front(millis());

  if (deviationTimestamps.size() == MAX_DEVIATION_TIMESTAMPS_IN_LIST) {
    long deviationsInterval = deviationTimestamps.get(0) - deviationTimestamps.get(MAX_DEVIATION_TIMESTAMPS_IN_LIST - 1);

    Serial.println("Deviations interval = " + String(deviationsInterval) + " millis");

    if (deviationsInterval <= MIN_DEVIATION_TIME_DIFF_FOR_NOTIF) {
      // Update the last time the sleep state notification was sent
      lastSleepStateNotificationSentAt = millis();

      // Clear the list for a fresh start
      deviationTimestamps.clear();

      // Update the sleep state variable in the Firebase realtime database

      // Send the sleep state notification

      Serial.println();
      Serial.println("---------------------------------------------------------");
      Serial.println("---------------------------------------------------------");
      Serial.println("Notification sent for sleep state disturbance");
      Serial.println("---------------------------------------------------------");
      Serial.println("---------------------------------------------------------");
      Serial.println();
    } else {
      // Remove the oldest deviation timestamp registered in the list in order to keep it updated and
      // within the size limit
      deviationTimestamps.pop_back();
    }
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
   Start the Firebase connection and the necessary properties
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
void pushFirebaseJsonEntry(const String &databasePath, FirebaseJson &jsonToPush) {
  if (Firebase.pushJSON(firebaseData, databasePath, jsonToPush)) {
    //Serial.println(firebaseData.dataPath() + "/" + firebaseData.pushName());
  } else {
    Serial.println(firebaseData.errorReason());
  }
}

/*
   Set the json value into the given database path in Firebase
*/
void setFirebaseJsonEntry(const String &databasePath, FirebaseJson &jsonToSet) {
  if (Firebase.setJSON(firebaseData, databasePath, jsonToSet)) {
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

/*
   Setup the Firebase cloud messaging properties
*/
void setupFirebaseCloudMessaging() {
  firebaseData.fcm.begin(FIREBASE_FCM_SERVER_KEY);

  firebaseData.fcm.setPriority("high");

  firebaseData.fcm.setTimeToLive(1000);
}

/*
   Fetch the client device tokens from the Firebase realtime database
   and add them to the Firebase cloud messaging object in order to
   target a specifc set of devices
*/
void addDeviceTokensToFCM() {
  // Remove the previous clients from the FCM object in order to always have
  // an up to date list without repeated clients
  firebaseData.fcm.clearDeviceToken();

  // Fetch the client tokens
  if (Firebase.getJSON(firebaseData, RT_DATABASE_CLIENT_TOKENS)) {

    if (firebaseData.dataType() == "json") {
      FirebaseJson &json = firebaseData.jsonObject();

      //Print all object data
      Serial.println();
      Serial.println("Pretty printed JSON data:");
      String jsonStr;
      json.toString(jsonStr, true);
      Serial.println(jsonStr);
      Serial.println();

      // Iterate through the json object
      size_t len = json.iteratorBegin();
      String key, value = "";
      int type = 0;
      for (size_t i = 0; i < len; i++) {
        json.iteratorGet(i, type, key, value);

        // Add the device token
        firebaseData.fcm.addDeviceToken(value);
      }
      json.iteratorEnd();
    }

  } else {
    Serial.println(firebaseData.errorReason());
  }
}

/*
   Send a notification to the registered clients about a certain temperature with
   a given warning level
*/
void sendTemperatureNotification(float temperature, int typeOfTempWarning) {
  Serial.println("------------------------------------");
  Serial.println("Sending temperature notification");

  // Add the devices to which we want to send the notification
  addDeviceTokensToFCM();

  // Force temperature  to only have one decimal place
  std::stringstream stream;
  stream << std::fixed << std::setprecision(1) << temperature;
  std::string finalTemperature = stream.str();

  // Define the title and body of the notification message according to the type of warning
  if (typeOfTempWarning == HIGH_TEMP_WARNING_FLAG) {
    firebaseData.fcm.setNotifyMessage("High Temperature", "(Fever) Your baby's temperature is " + String(finalTemperature.c_str()) + " ºC");
  } else if (typeOfTempWarning == LOW_TEMP_WARNING_FLAG) {
    firebaseData.fcm.setNotifyMessage("Low Temperature", "(Cold) Your baby's temperature is " + String(finalTemperature.c_str()) + " ºC");
  }

  // Define the data object sent inside the notification
  firebaseData.fcm.setDataMessage("{\"temperature\":" + String(finalTemperature.c_str()) + ", \"typeOfTempWarning\":" + String(typeOfTempWarning) + "}");

  // Send the notification to all of the registerd clients
  if (Firebase.broadcastMessage(firebaseData)) {
    Serial.println("PASSED");
    Serial.println(firebaseData.fcm.getSendResult());
    Serial.println("------------------------------------");
    Serial.println();
  } else {
    Serial.println("FAILED");
    Serial.println("REASON: " + firebaseData.errorReason());
    Serial.println("------------------------------------");
    Serial.println();
  }
}

/**
   Setup the Firebase realtime database stream callback functions
*/
void setupFirebaseStreamCallbacks() {
  Firebase.setStreamCallback(firebaseData, firebaseStreamCallback, firebaseStreamTimeoutCallback);
}

/*
   Setup the Firebase realtime database stream of the the temperature threshold values
*/
void setupFirebaseTemperatureThresholdsStream() {
  // Start the stream
  if (!Firebase.beginStream(firebaseData, RT_DATABASE_TEMPERATURE_THRESHOLDS)) {
    //Could not begin stream connection, then print out the error detail
    Serial.println(firebaseData.errorReason());
  }
}

/*
   Callback function for the Firebase realtime database stream
*/
void firebaseStreamCallback(StreamData dataReceived) {
  if (dataReceived.dataType() == "int") {
    if (isTemperatureThresholdsUpdate(dataReceived)) {
      if (isHighTempThresholdUpdate(dataReceived)) {
        highTempThreshold = dataReceived.intData();
      } else if (isLowTempThresholdUpdate(dataReceived)) {
        lowTempThreshold = dataReceived.intData();
      }
    }

    printTemperatureThresholds();
    return;
  }

  if (dataReceived.dataType() == "float") {
    if (isTemperatureThresholdsUpdate(dataReceived)) {
      if (isHighTempThresholdUpdate(dataReceived)) {
        highTempThreshold = dataReceived.floatData();
      } else if (isLowTempThresholdUpdate(dataReceived)) {
        lowTempThreshold = dataReceived.floatData();
      }
    }

    printTemperatureThresholds();
    return;
  }

  if (dataReceived.dataType() == "json" && isTemperatureThresholdsUpdate(dataReceived)) {
    //Print out the value
    //Serial.println(dataReceived.jsonString());

    FirebaseJson &json = dataReceived.jsonObject();

    // Iterate through the json object
    size_t len = json.iteratorBegin();
    String key, value = "";
    int type = 0;
    for (size_t i = 0; i < len; i++) {
      json.iteratorGet(i, type, key, value);

      if (isHighTempThresholdUpdate(key)) {
        highTempThreshold = value.toFloat();
      } else if (isLowTempThresholdUpdate(key)) {
        lowTempThreshold = value.toFloat();
      }
    }
    json.iteratorEnd();

    printTemperatureThresholds();
    return;
  }
}

/*
   Callback function that notifies when the stream connection is lost
*/
void firebaseStreamTimeoutCallback(bool timeout) {
  if (timeout) {
    //Stream timeout occurred
    Serial.println("Temperature threshold values stream timeout, resume streaming...");
  }
}

boolean isTemperatureThresholdsUpdate(StreamData dataReceived) {
  return dataReceived.streamPath() == "/" + RT_DATABASE_TEMPERATURE_THRESHOLDS;
}

boolean isHighTempThresholdUpdate(StreamData dataReceived) {
  return dataReceived.dataPath() == "/" + RT_DATABASE_HIGH_TEMP_THRESHOLD_KEY;
}

boolean isHighTempThresholdUpdate(String keyName) {
  return keyName == RT_DATABASE_HIGH_TEMP_THRESHOLD_KEY;
}

boolean isLowTempThresholdUpdate(StreamData dataReceived) {
  return dataReceived.dataPath() == "/" + RT_DATABASE_LOW_TEMP_THRESHOLD_KEY;
}

boolean isLowTempThresholdUpdate(String keyName) {
  return keyName == RT_DATABASE_LOW_TEMP_THRESHOLD_KEY;
}

void printTemperatureThresholds() {
  Serial.println("(Thresholds) High temp: " + String(highTempThreshold) + " Low temp: " + String(lowTempThreshold));
}
