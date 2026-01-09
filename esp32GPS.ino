/* https://www.circuitdigest.cloud/geolinker-web-app
 * ===============================================================
 *              GEOLINKER GPS TRACKER (NEO-7M SAFE)
 * ===============================================================
 * Target:
 *   - ESP32
 *   - u-blox NEO-7M GPS
 *   - WiFi backend
 *
 * Behavior:
 *   - Waits for GPS fix before uploading
 *   - Avoids false GPS parse errors
 *   - Stable and deterministic operation
 * ===============================================================
 */

 //my personal goal is to make multiple circuits and create a network of with cool visuals and maps
 //Which I could then use for a 3d website for my personal portfolio

#include <GeoLinker.h>
#include <WiFi.h>
#include "DHT.h"
#include "credentials.h"

// ===============================================================
// HARDWARE CONFIGURATION
// ===============================================================
HardwareSerial gpsSerial(1);   // UART1 for GPS

#define GPS_RX 16              // ESP32 RX ← GPS TX
#define GPS_TX 17              // ESP32 TX → GPS RX (optional)
#define GPS_BAUD 9600

// Pin configuration and sensor type
#define DHTPIN 4        // GPIO pin connected to DHT22
#define DHTTYPE DHT22   // Specify DHT22 sensor

DHT dht(DHTPIN, DHTTYPE);

#define LEDPIN 15
// ===============================================================
// NETWORK CONFIGURATION (WiFi)
// ===============================================================
const char* ssid     = mySSID;
const char* password = myPASSWORD;

// ===============================================================
// GEOLINKER CONFIGURATION
// ===============================================================
const char* apiKey   = myGPSAPIKey;
const char* deviceID = "espKleveNeo7M";

// =================================================
//MQTT
// =================================================
const char* mqtt_server = "192.168.107.93"; // <-- Pi’s LAN IP
const int mqtt_port = 1883;
const char* mqtt_topic = "iot/espKleveNeo7M/telemetry";



const uint16_t updateInterval = 10;   // seconds
const bool enableOfflineStorage = true;
const uint8_t offlineBufferLimit = 20;
const bool enableAutoReconnect = true;

// Timezone
const int8_t timeOffsetHours   = 5;
const int8_t timeOffsetMinutes = 30;

// ===============================================================
GeoLinker geo;

// ===============================================================
// INTERNAL STATE
// ===============================================================
bool gpsFixAcquired = false;
uint32_t gpsStartTime = 0;

uint32_t dhtStartTime = 0;

struct GPS_Coordinates {
  double latitude;
  double longitude;
  bool valid;
};

GPS_Coordinates parseGPRMC(String nmea) {
  GPS_Coordinates coord;
  coord.valid = false;

  if (nmea.startsWith("$GPRMC")) {
    // Split by comma
    int parts[12];
    String tokens[12];
    int idx = 0;
    int start = 0;
    for (int i = 0; i < nmea.length() && idx < 12; i++) {
      if (nmea[i] == ',' || nmea[i] == '*') {
        tokens[idx++] = nmea.substring(start, i);
        start = i + 1;
      }
    }

    if (tokens[2] == "A") { // Status = A means valid fix
      coord.valid = true;
      // Latitude
      double lat = tokens[3].toDouble();
      double lat_deg = floor(lat / 100.0);
      double lat_min = lat - lat_deg * 100.0;
      coord.latitude = lat_deg + lat_min / 60.0;
      if (tokens[4] == "S") coord.latitude = -coord.latitude;

      // Longitude
      double lon = tokens[5].toDouble();
      double lon_deg = floor(lon / 100.0);
      double lon_min = lon - lon_deg * 100.0;
      coord.longitude = lon_deg + lon_min / 60.0;
      if (tokens[6] == "W") coord.longitude = -coord.longitude;
    }
  }
  return coord;
}

/////////////////////DHT22/////////////


// ===============================================================
// SETUP
// ===============================================================
void setup(){
  Serial.begin(115200);
  delay(1000);

  dht.begin();
  dhtStartTime = millis();

  pinMode(LEDPIN,OUTPUT);

  Serial.println("\n=== GeoLinker GPS Tracker (NEO-7M) ===");

  // ---- GPS UART ----
  gpsSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS serial initialized (UART1 @ 9600)");

  // ---- GeoLinker Core ----
  geo.begin(gpsSerial);
  geo.setApiKey(apiKey);
  geo.setDeviceID(deviceID);
  geo.setUpdateInterval_seconds(updateInterval);
  geo.setDebugLevel(DEBUG_BASIC);

  geo.enableOfflineStorage(enableOfflineStorage);
  geo.setOfflineBufferLimit(offlineBufferLimit);
  geo.enableAutoReconnect(enableAutoReconnect);

  geo.setTimeOffset(timeOffsetHours, timeOffsetMinutes);

  // ---- Network (WiFi) ----
  geo.setNetworkMode(GEOLINKER_WIFI);
  geo.setWiFiCredentials(ssid, password);

  Serial.print("Connecting to WiFi: ");
  Serial.println(ssid);

  if (!geo.connectToWiFi()) {
    Serial.println("⚠ WiFi connection failed (offline mode enabled)");
  } else {
    Serial.println("✓ WiFi connected");
  }

  ///first temp and hum
  
  // ---- GPS Warm-up ----
  gpsStartTime = millis();
  Serial.println("\nWaiting for GPS fix...");
  Serial.println("→ Move device outdoors with clear sky view");
  Serial.println("→ First fix may take 2–5 minutes\n");
}

// ===============================================================
// LOOP
// ===============================================================
void loop(){
  uint8_t status = geo.loop();

  // Read humidity and temperature data
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Error handling if sensor fails to provide data
  if (isnan(humidity) || isnan(temperature)) {
    Serial.println("Error: Unable to read data from DHT sensor.");
    return;
  }

  // Print temperature results to the serial monitor
  if(millis()-dhtStartTime > 30000){
    Serial.println();
    Serial.println();
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.print(" %\t");
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.println();
    Serial.println();

    dhtStartTime = millis();

  }

  // -------------------------------------------------------------
  // GPS FIX HANDLING
  // -------------------------------------------------------------
  if (!gpsFixAcquired) {
    if (status == STATUS_SENT) {
      gpsFixAcquired = true;
      Serial.println("\n✓ GPS FIX ACQUIRED");
      Serial.println("✓ Location uploads enabled\n");
    }
    else if (status == STATUS_PARSE_ERROR) {
      static uint32_t lastMsg = 0;
      if (millis() - lastMsg > 10000) {
        Serial.println("⏳ GPS has no fix yet (RMC status = V)");
        lastMsg = millis();
        signal(LEDPIN);
      }
    }
    else if (millis() - gpsStartTime > 300000) { // 5 minutes
      Serial.println("⚠ GPS fix taking longer than expected");
      gpsStartTime = millis(); // reset warning timer
    }

    delay(200);
    return;   // Do not proceed until fix exists
  }

  // -------------------------------------------------------------
  // NORMAL OPERATION (FIX ACQUIRED)
  // -------------------------------------------------------------
  if (status > 0) {
    Serial.print("[GeoLinker] ");

    switch (status) {
      case STATUS_SENT:
        Serial.println("Data sent successfully");
        digitalWrite(LEDPIN, HIGH);
        break;

      case STATUS_NETWORK_ERROR:
        Serial.println("Network error (buffering offline)");
        signal(LEDPIN);
        break;

      case STATUS_BAD_REQUEST_ERROR:
        Serial.println("Bad request (check API key)");
        signal(LEDPIN);
        break;

      case STATUS_INTERNAL_SERVER_ERROR:
        Serial.println("Server error");
        signal(LEDPIN);
        break;

      default:
        Serial.print("Status code: ");
        Serial.println(status);
        break;
    }
  }

  static double lastLat = 0, lastLon = 0;

  static String nmeaLine = "";
  while (gpsSerial.available()) {
    char c = gpsSerial.read();
    if (c == '\n') {
      GPS_Coordinates coord = parseGPRMC(nmeaLine);
      if (coord.valid) {
        Serial.print("NEW FIX → LAT: ");
        Serial.print(coord.latitude, 6);
        Serial.print(", LON: ");
        Serial.println(coord.longitude, 6);
        }
      nmeaLine = "";
      } else if (c != '\r') {
          nmeaLine += c;
      }
}

  
}


void signal(int pin){
  digitalWrite(pin, HIGH);
        delay(50);
        digitalWrite(pin, LOW);
        delay(50);
        digitalWrite(pin, HIGH);
        delay(50);
        digitalWrite(pin, LOW);
        delay(50);
        digitalWrite(pin, HIGH);
        delay(50);
        digitalWrite(pin, LOW);
        delay(50);
        digitalWrite(pin, HIGH);
        delay(50);
        digitalWrite(pin, LOW);
        delay(50);
}