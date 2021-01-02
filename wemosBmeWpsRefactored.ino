#include <BME280I2C.h>
#include <EnvironmentCalculations.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ESP8266HTTPClient.h>

#define WPS D4 // WPS button
#define SERIAL_BAUD 74880
#define FORCE_DEEPSLEEP

BME280I2C bme;

// configuration - begin
String userAgent = "Wireless Weather Station";
String clientVer = "0.1.1";

char* apiUrl = "YOUR_API_URL";
char* apiContentType = "application/x-www-form-urlencoded";

bool debug = true;
int minutes2sleep = 5;
// configuration - end

uint32_t chipID = ESP.getChipId();
char* sensor = "UNKNOWN";

float temperature;
float humidity;
float pressure;
float dewpoint;
float heatindex;
float lon;
float lat;

/**
   Le Setup
*/
void setup()
{
  Serial.begin(SERIAL_BAUD);

  if (true == debug)
  {
    Serial.setDebugOutput(true);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB
    }
  }

  startWIFI();
}

/**
   Looping Louie
*/
void loop()
{
  getSensor();            // Init the sensor
  getSensorData();        // Get data from sensors

  splashScreen();         // Show configuration

  if (!sendSensorData())  // Send data to API
  {
    Serial.println("[Sensor data] Failure!");
  }
  else
  {
    Serial.println("[Sensor data] Successfully sent.");
  }

  //goToBed(minutes2sleep); // Sending into deep sleep
  delay(10000);
}

void getSensor()
{
  if (true == debug)
  {
    Serial.print("[Sensor] Search for sensor ... ");
  }

  Wire.begin();
  while (!bme.begin())
  {
    Serial.println("Could not find BME280 sensor!");
    delay(1000);
  }

  switch (bme.chipModel()) {
    case BME280::ChipModel_BME280:
      sensor = "BME280";
      break;
    case BME280::ChipModel_BMP280:
      sensor = "BMP280";
      break;
    default:
      break;
  }

  if (true == debug)
  {
    Serial.printf("%s found.\n", sensor);
  }
}

void getSensorData()
{
  if (true == debug)
  {
    Serial.print("[Sensor data] Get data from sensor ... ");
  }

  BME280::TempUnit tempUnit(BME280::TempUnit_Celsius);
  BME280::PresUnit presUnit(BME280::PresUnit_bar);
  bme.read(pressure, temperature, humidity, tempUnit, presUnit);
  pressure = pressure * 1000; // convert to millibar

  dewpoint = EnvironmentCalculations::DewPoint(temperature, humidity);
  heatindex = EnvironmentCalculations::HeatIndex(temperature, humidity);

  if (true == debug)
  {
    Serial.println("done.");
  }
}

void splashScreen()
{
  Serial.println("#################################################");
  Serial.printf("# Chip ID:\t\t%08X\n", ESP.getChipId());
  Serial.printf("# Flash Chip ID:\t%08X\n", ESP.getFlashChipId());
  Serial.printf("# Flash Chip Speed:\t%d (Hz)\n", ESP.getFlashChipSpeed());
  Serial.printf("# Flash Chip Size:\t%d (bytes)\n", ESP.getFlashChipSize());
  Serial.printf("# Flash Heap Size:\t%d (bytes)\n", ESP.getFreeHeap());
  Serial.println("# --------------------------------------------- #");
  Serial.print("# Debug:\t\t");
  Serial.printf("%s\n", (true == debug ? "true" : "false"));
  Serial.printf("# Delay:\t\t%d ", minutes2sleep);
  Serial.println("minute(s)");
  Serial.println("# --------------------------------------------- #");
  Serial.printf("# WiFi SSID:\t\t%s\n", WiFi.SSID().c_str());
  Serial.print("# WiFi MAC:\t\t");
  Serial.println(WiFi.macAddress());
  Serial.printf("# WiFi RSSI:\t\t%d dBm\n", WiFi.RSSI());
  Serial.printf("# WiFi Hostname:\t%s\n", WiFi.hostname().c_str());
  Serial.print("# WiFi IP Address:\t");
  Serial.println(WiFi.localIP());
  Serial.printf("# API Url:\t\t%s\n", apiUrl);
  Serial.println("# --------------------------------------------- #");
  Serial.printf("# Sensor:\t\t%s\n", sensor);
  Serial.printf("# Temperature:\t\t%f °C\n", temperature);
  Serial.printf("# Humidity:\t\t%f %%\n", humidity);
  Serial.printf("# Pressure:\t\t%f hpa\n", pressure);
  Serial.printf("# Dew point:\t\t%f °C\n", dewpoint);
  Serial.printf("# Heat index:\t\t%f\n", heatindex);
  Serial.println("#################################################");
}

/**
   Sending device into deep sleep
*/
void goToBed (int minutes) {
#ifdef FORCE_DEEPSLEEP
  ESP.deepSleep(minutes * 60 * 1000000);
#endif
}

bool startWPS() {
  if (true == debug) {
    Serial.println("[WiFi] WPS configuration started");
  }
  bool wpsSuccess = WiFi.beginWPSConfig();

  if (wpsSuccess) {
    String newSSID = WiFi.SSID();
    if (0 < newSSID.length()) {
      if (true == debug) {
        Serial.printf("[WiFi] WPS ready. Connected to '%s'\n", newSSID.c_str());
      }
    } else {
      wpsSuccess = false;
    }
  }

  return wpsSuccess;
}

/**
   Establish WiFi-Connection
*/
void startWIFI() {
  if (true == debug) {
    if (WiFi.SSID().length() > 0)
    {
      Serial.printf("\n[WiFi] Try connection to saved SSID '%s'\n", WiFi.SSID().c_str());
    } else {
      Serial.println("\n[WiFi] No saved SSID found. Init WPS ... ");
    }
  }

  pinMode(WPS, INPUT_PULLUP); // acivate button input

  WiFi.mode(WIFI_STA);
  WiFi.hostname(getSensorHostname());

  // last saved login data
  WiFi.begin(WiFi.SSID().c_str(), WiFi.psk().c_str());
  int cnt = 0;
  // try login
  while ((WiFi.status() == WL_DISCONNECTED) && (cnt < 10)) {
    delay(500);
    if (true == debug) {
      Serial.print(".");
    }
    cnt++;
  }

  wl_status_t status = WiFi.status();
  if (status == WL_CONNECTED) {
    Serial.printf("\n[WiFi] Successfully logged in to SSID '%s'\n", WiFi.SSID().c_str());
    blinkBuildInLed(5, 100); // blink 5 times fast on connection success
  } else {
    // We were not successful launch therefore WPS
    Serial.println("[WiFi] Press the WPS button on the router.");
    Serial.println("[WiFi] Press the WPS button on the ESP!");

    while (digitalRead(WPS) != 0) {
      blinkBuildInLed(1, 1000); // blink every second until wps button activated
      yield();
    }
    if (!startWPS()) {
      Serial.println("[WiFi] No connection can be established via WPS!");
    }
  }
}

bool sendSensorData()
{
  HTTPClient http;
  int httpCode = -1;
  String payload = "temp=" + String(temperature) + "&humidity=" + String(humidity) + "&pressure=" + String(pressure) + "&dewpoint=" + String(dewpoint) + "&heatindex=" + String(heatindex);

  if (true == debug)
  {
    Serial.println("[HTTP] Start connection info request...");
  }

  http.begin(apiUrl);
  http.setUserAgent(userAgent + " " + clientVer);
  http.addHeader("Content-Type", apiContentType);
  httpCode = http.POST(payload);

  if (true == debug) {
    Serial.printf("[HTTP] GET... code: %d\n", httpCode);
    Serial.println("[HTTP] End connection.");
  }

  if (httpCode > 0) {
    if (httpCode == HTTP_CODE_OK) {
      return true;
    }
  }

  return false;
}

String getSensorHostname()
{
  char buffer[20];
  sprintf(buffer, "WWC-ESP8266-%08X", ESP.getChipId());

  return buffer;
}

void blinkBuildInLed(int flashtimes, int delaySeconds)
{
  pinMode(LED_BUILTIN, OUTPUT);     // define LED as output
  digitalWrite(LED_BUILTIN, HIGH);  // disable

  for (int i = 0; i < flashtimes; i++)
  {
    digitalWrite(LED_BUILTIN, LOW);
    delay(delaySeconds);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(delaySeconds);
  }
}
