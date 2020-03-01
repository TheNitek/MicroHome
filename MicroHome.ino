#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>     
#include <ArduinoJson.h>
#include <UniversalTelegramBot.h>
#include <HTTPClient.h>
#include "BLEDevice.h"
#include "time.h"
#include "Config.h"

#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  30        /* Time ESP32 will go to sleep (in seconds) */

static const uint8_t LED = 2;

WiFiManager wifiManager;

const char* ntpServer = "fritz.box";
const char* timeZone = "CET-1CEST,M3.5.0,M10.5.0/3";

// Initialize Telegram BOT
//#define BOTtoken "asdasdasd"  // your Bot Token (Get from Botfather)

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

const uint16_t botCheckInterval = 5; //mean time between scan messages
RTC_DATA_ATTR time_t botLastUpdate;   //last time messages' scan has been done
const char *chatIds[] = {"302777211", "1070698297"};

static BLEUUID uuidFlowerService("00001204-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidFlowerVersionBattery("00001a02-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidFlowerMode("00001a00-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidFlowerData("00001a01-0000-1000-8000-00805f9b34fb");

enum ConnectionMode {
  WIFI,
  BLE
} currentMode;

BLEAddress bleGuava = BLEAddress("c4:7c:8d:65:e9:92");
typedef struct  {
  float temperature;
  uint8_t moisture;
  uint32_t light;
  uint16_t conductivity;
  uint8_t battery;
  time_t lastUpdate;
} FlowerData;
RTC_DATA_ATTR FlowerData flowerData;


static BLEUUID uuidThermoService("ebe0ccb0-7a0a-4b0c-8a1a-6ff2997da3a6");
static BLEUUID uuidThermoData("ebe0ccc1-7a0a-4b0c-8a1a-6ff2997da3a6");
static BLEUUID uuidThermoAuthService("0000fe95-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidThermoAuth("00000010-0000-1000-8000-00805f9b34fb");
static BLEUUID uuidThermoBatteryService("180f");
static BLEUUID uuidThermoBattery("00002A19-0000-1000-8000-00805f9b34fb");

struct ThermoData {
  ThermoData(const char* deviceName, std::string stringAddress) : name(deviceName), address(stringAddress){};
  const char* name;
  BLEAddress address;
  float temperature;
  uint8_t humidity;
  uint8_t battery;
  time_t lastUpdate;
  volatile bool notified = false;

  void thermoNotifyCallback(uint8_t* pData, size_t length, bool isNotify) {
    if(notified || !isNotify) {
      return;
    }
    
    Serial.println("- got thermo data");

    if(length < 3) {
      Serial.println("- Invalid data");
      notified = true;
      return;
    }

    temperature = (pData[0] + (pData[1] << 8)) / 100.0f;
    Serial.printf("-- %f\n", temperature);

    humidity = pData[2];
    Serial.printf("-- %d\n", humidity);

    notified = true;
  }

  void toString(char* output, uint8_t outputSize) {
    snprintf(output, outputSize,
      "%s: %.1f°C, %d%%, (%d%% Batterie)\n",
      name,
      temperature,
      humidity,
      battery
    );
  }
};

RTC_DATA_ATTR ThermoData thermometers[] = {
  ThermoData("Wohnzimmer", "a4:c1:38:46:e6:72"),
  ThermoData("Schlafzimmer", "a4:c1:38:9a:f6:a1"),
  ThermoData("Bad", "a4:c1:38:18:93:8a"),
  ThermoData("Kinderzimmer", "a4:c1:38:0f:0d:43")
};

ThermoData* currentThermo;
static void thermoNotifyCallback(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  currentThermo->thermoNotifyCallback(pData, length, isNotify);
};

// Do not update BLE infos if this many seconds did not pass since last update
const uint16_t minTimeBetweenUpdates = 2*60;

const uint16_t minTimeBetweenAlerts = 60*60;

const uint16_t bleUpdateInterval = 30*60;
RTC_DATA_ATTR time_t bleLastUpdate = 0;
boolean bleForceUpdate = false;

BLEScan* pBLEScan;

//static BLEAdvertisedDevice* myDevice;

char statusRequestor[20] = "";

const uint8_t noAlarmBefore = 6;
const uint8_t noAlarmBeforeWeekend = 8;
const uint8_t noAlarmAfter = 22;

const uint8_t plantOnHour = 6;
const uint8_t plantOffHour = 19;
const uint32_t plantMinLux = 1000;
#define PLANT_ON_URL "http://192.168.1.181/cm?cmnd=Power%20On"
#define PLANT_OFF_URL "http://192.168.1.181/cm?cmnd=Power%20Off"
#define PLANT_STATUS_URL "http://192.168.1.181/cm?cmnd=Power"
boolean plantPowerStatus = false;

RTC_DATA_ATTR struct {
  time_t guavaTemperatureLow = 0;
  time_t guavaTemperatureHigh = 0;
  time_t guavaWaterLow = 0;
  time_t guavaWaterHigh = 0;
  time_t vent = 0;
} notificationStatus;

const int wdtTimeout = 60 * 1000;
hw_timer_t *watchdogTimer = NULL;

void IRAM_ATTR resetModule() {
  Serial.println("watchdog reset");
  esp_sleep_enable_timer_wakeup(1);
  esp_deep_sleep_start();
}

bool readFlower(FlowerData* flowerData) {
  timerWrite(watchdogTimer, 0);

  if(difftime(time(NULL), flowerData->lastUpdate) <= minTimeBetweenUpdates) {
    Serial.println("Flower data up to date - Skipping");
    return true;
  }

  Serial.print("Forming a connection to ");
  Serial.println(bleGuava.toString().c_str());

  BLEClient *pClient  = BLEDevice::createClient();
  //pClient->setClientCallbacks(bleClientCallback);
  BLEDevice::setPower(ESP_PWR_LVL_P7);
  Serial.println("- Created client");

  if(!pClient->connect(bleGuava)) {
    Serial.println("- Failed to connect to flower sensor");
    pClient->disconnect();
    time_t start = time(NULL);
    while(pClient->isConnected() && (difftime(time(NULL), start) < 60)) yield();
    delete pClient;
    return false;
  }
  Serial.println("- Connected to flower sensor");

  uint8_t magicValueRaw[] = {0xA0, 0x1F};
  pClient->getService(uuidFlowerService)->getCharacteristic(uuidFlowerMode)->writeValue(magicValueRaw, sizeof(magicValueRaw), true);
  Serial.println("- Set magic value");

  const char *val = pClient->getValue(uuidFlowerService, uuidFlowerData).c_str();

  Serial.println("- Read flower data");

  uint16_t* temp_raw = (uint16_t*)val;
  flowerData->temperature = (*temp_raw) / ((float)10.0);
  Serial.print("-- Temperature: ");
  Serial.println(flowerData->temperature);

  flowerData->moisture = val[7];
  Serial.print("-- Moisture: ");
  Serial.println(flowerData->moisture);

  flowerData->light = val[3] + val[4] * 256;
  Serial.print("-- Light: ");
  Serial.println(flowerData->light);
 
  flowerData->conductivity = val[8] + val[9] * 256;
  Serial.print("-- Conductivity: ");
  Serial.println(flowerData->conductivity);

  val = pClient->getValue(uuidFlowerService, uuidFlowerVersionBattery).c_str();
  Serial.println("- Read version and battery");
  flowerData->battery = val[0];

  Serial.print("-- Battery: ");
  Serial.println(flowerData->battery);

  time(&flowerData->lastUpdate);

  pClient->disconnect();
  time_t start = time(NULL);
  while(pClient->isConnected() && (difftime(time(NULL), start) < 60)) yield();
  delete pClient;
  
  return true;
}

bool readThermo(ThermoData* thermoData) {
  timerWrite(watchdogTimer, 0);
  if(difftime(time(NULL), thermoData->lastUpdate) <= minTimeBetweenUpdates) {
    Serial.println("Data are up to date - Skipping");
    return true;
  }
  
  Serial.printf("Forming a connection to %s (%s)\n", thermoData->name, thermoData->address.toString().c_str());

  BLEClient *pClient  = BLEDevice::createClient();
  //pClient->setClientCallbacks(bleClientCallback);
  BLEDevice::setPower(ESP_PWR_LVL_P7);
  Serial.println("- Created client");

  if(!pClient->connect(thermoData->address)) {
    Serial.println("- Failed to connect to sensor");
    pClient->disconnect();
    time_t start = time(NULL);
    while(pClient->isConnected() && (difftime(time(NULL), start) < 60)) yield();
    delete pClient;
    return false;
  }
  Serial.println("- Connected to sensor");

  uint8_t magicValueRaw[] = {0x00, 0x38};
  pClient->getService(uuidThermoAuthService)->getCharacteristic(uuidThermoAuth)->writeValue(magicValueRaw, sizeof(magicValueRaw), true);
  Serial.println("- Set magic value");

  for(uint8_t i = 0; i < 3; i++) {
    const char* battery = pClient->getValue(uuidThermoBatteryService, uuidThermoBattery).c_str();
    thermoData->battery = battery[0];
    if(thermoData->battery != 0) {
      break;
    }
  }

  thermoData->notified = false;
  
  currentThermo = thermoData;
  pClient->getService(uuidThermoService)->getCharacteristic(uuidThermoData)->registerForNotify(thermoNotifyCallback);

  time_t start = time(NULL);
  while(!thermoData->notified && (difftime(time(NULL), start) < 60)) yield();

  time(&thermoData->lastUpdate);

  pClient->disconnect();
  start = time(NULL);
  while(pClient->isConnected() && (difftime(time(NULL), start) < 60)) yield();
  delete pClient;

  currentThermo = nullptr;
  
  return thermoData->notified;
}

void startWifi() {
  timerWrite(watchdogTimer, 0);

  Serial.println("Connecting Wifi");

  wifiManager.setEnableConfigPortal(false);
  wifiManager.setTimeout(60);
  uint8_t i = 0;
  while(!wifiManager.autoConnect() && i++ < 3) {
    Serial.println("Retry autoConnect");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }
  if(!WiFi.isConnected()) {
    timerStop(watchdogTimer);
    wifiManager.setEnableConfigPortal(true);
    wifiManager.autoConnect("NaeveBot", "botbotbot");
    timerStart(watchdogTimer);
  }

  Serial.print("WiFi connected with IP: "); Serial.println(WiFi.localIP());
}
void stopWifi() {
  wifiManager.disconnect();
  WiFi.mode(WIFI_OFF);
}

void startBLE() {
  BLEDevice::init("Collector");
}

void stopBLE() {
  BLEDevice::deinit();
}

void setMode(ConnectionMode mode) {
  if(mode == BLE && currentMode != BLE) {
    Serial.println("Switching to BLE mode");
    stopWifi();
    startBLE();
    currentMode = BLE;
  } else if(mode == WIFI && currentMode != WIFI) {
    Serial.println("Switching to WiFi mode");
    stopBLE();
    startWifi();
    currentMode = WIFI;
  }
}

/*void scanBLE() {
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(), true);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->start(0);
}*/

void updateFlowerStatus() {
  for(uint8_t i = 0; i < 3; i++) {
    if(readFlower(&flowerData)) {
      Serial.println("Got flower data");
      return;
    }
  }
  Serial.println("Failed to get flower data");
}

void updateThermoStatus() {
  for(uint8_t i = 0; i < 4; i++) {
    delay(1000);
    Serial.print("- "); Serial.println(ESP.getFreeHeap());
    for(uint8_t j = 0; j < 3; j++) {
      if(readThermo(&thermometers[i])) {
        Serial.printf("Got %s thermo data\n", thermometers[i].name);
        break;
      }
    }
    Serial.print("- "); Serial.println(ESP.getFreeHeap());
  }
}

void handleTelegram() {
  timerWrite(watchdogTimer, 0);

  if (difftime(time(NULL), botLastUpdate) > botCheckInterval)  {
    Serial.println(ESP.getFreeHeap());
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      Serial.println("got message");
      for (int i = 0; i < numNewMessages; i++) {
        Serial.print("ChatId: "); Serial.println(bot.messages[i].chat_id);
        if (bot.messages[i].text == "/status") {
          bleForceUpdate = true;
          strcpy(statusRequestor, bot.messages[i].chat_id.c_str());
          bot.sendChatAction(statusRequestor, "typing");
          return;
        }
      }

      numNewMessages = bot.getUpdates(bot.last_message_received + 1);
    }

    time(&botLastUpdate);
    Serial.println(ESP.getFreeHeap());
  }
}

void printLocalTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

boolean isQuietTime() {
  tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  if(timeinfo.tm_hour >= noAlarmAfter) {
    return true;
  }
  switch(timeinfo.tm_wday) {
    case 0:
    case 6:
      // Weekend
      return (timeinfo.tm_hour < noAlarmBeforeWeekend);
    default:
      // Not weekend
      return (timeinfo.tm_hour < noAlarmBefore);
  }
}

boolean setPlantPower(boolean power) {
  timerWrite(watchdogTimer, 0);

  if(power == plantPowerStatus) {
    Serial.println("Power status already set");
    return true;
  }
  if(power) {
    return httpGetPlantPower(PLANT_ON_URL, &plantPowerStatus);
  } else {
    return httpGetPlantPower(PLANT_OFF_URL, &plantPowerStatus);
  }
}

boolean updatePlantPower() {
  return httpGetPlantPower(PLANT_STATUS_URL, &plantPowerStatus);
}

boolean httpGetPlantPower(const char* url, const boolean *status) {
  HTTPClient http;
  http.begin(url);
  Serial.printf("Executing get: %s\n", url);
  int httpCode = http.GET();

  if(httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, response);

    if (!error) {
      const char* powerStatus = doc["POWER"];
      plantPowerStatus = (strcmp(powerStatus, "ON") == 0);
      http.end();
      return true;
    }
    Serial.print("deserializeJson() failed: "); Serial.println(error.c_str());
  } else if(httpCode > 0) {
    Serial.printf("Unexpected HTTP response: %d\n", httpCode);
  } else {
    Serial.printf("Error during HTTP call: %s\n", http.errorToString(httpCode).c_str());
  }

  http.end();
  return false;
}

void updateSensorData() {
  setMode(BLE);
  Serial.println(ESP.getFreeHeap());
  updateFlowerStatus();
  updateThermoStatus();
  //scanBLE();
  Serial.println(ESP.getFreeHeap());
  setMode(WIFI);
  time(&bleLastUpdate);
}

void applyRules() {
    tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    } else if(timeinfo.tm_hour >= plantOnHour && timeinfo.tm_hour < plantOffHour && flowerData.light < plantMinLux) {
      setPlantPower(true);
    } else {
      setPlantPower(false);
    }

    // Only notify if data have been received at least once
    if(flowerData.lastUpdate > 0) {
      if(flowerData.moisture < 15) {
        if(difftime(time(NULL), notificationStatus.guavaWaterLow) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Die Guave hat durst! (Feuchtigkeit: %d%%)", flowerData.moisture);
          if(sendNotification(msg)) {
            time(&notificationStatus.guavaWaterLow);
          }
        }
      } else {
        if(notificationStatus.guavaWaterLow != 0) {
          sendNotification("Der Guave hat genug zu trinken");
          notificationStatus.guavaWaterLow = 0;
        }
      }
      if(flowerData.moisture > 60) {
        if(difftime(time(NULL), notificationStatus.guavaWaterHigh) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Die Guave ertrinkt! (Feuchtigkeit: %d%%)", flowerData.moisture);
          if(sendNotification(msg)) {
            time(&notificationStatus.guavaWaterHigh);
          }
        }
      } else {
        if(notificationStatus.guavaWaterHigh != 0) {
          sendNotification("Der Guave ertrinkt nicht mehr");
          notificationStatus.guavaWaterHigh = 0;
        }
      }
      if(flowerData.temperature < 5) {
        if(difftime(time(NULL), notificationStatus.guavaTemperatureLow) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Der Guave ist kalt! (Temperatur: %.1f°C)", flowerData.temperature);
          if(sendNotification(msg)) {
            time(&notificationStatus.guavaTemperatureLow);
          }
        }
      } else {
        if(notificationStatus.guavaTemperatureLow != 0) {
          sendNotification("Der Guave ist nicht mehr kalt");
          notificationStatus.guavaTemperatureLow = 0;
        }
      }
      if(flowerData.temperature > 35) {
        if(difftime(time(NULL), notificationStatus.guavaTemperatureHigh) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Der Guave ist warm! (Temperatur: %.1f°C)", flowerData.temperature);
          if(sendNotification(msg)) {
            time(&notificationStatus.guavaTemperatureHigh);
          }
        }
      } else {
        if(notificationStatus.guavaTemperatureHigh != 0) {
          sendNotification("Der Guave ist nicht mehr warm");
          notificationStatus.guavaTemperatureHigh = 0;
        }
      }
    }
    boolean vent = false;
    char msgBuffer[400] = "Bitte lüften!\n";
    char thermoBuffer[100];
    for(uint8_t i = 0; i < 4; i++) {
      if(thermometers[i].humidity > 60) {
        vent = true;
        thermometers[i].toString(thermoBuffer, sizeof(thermoBuffer));
        strcat(msgBuffer, thermoBuffer);
      }
    }
    if(vent) {
      if(difftime(time(NULL), notificationStatus.vent) > minTimeBetweenAlerts) {
        if(sendNotification(msgBuffer)) {
          time(&notificationStatus.vent);
        }
      }
    } else {
      if(notificationStatus.vent != 0) {
        sendNotification("Erfolgreich gelüftet!");
        notificationStatus.vent = 0;
      }
    }
}

boolean sendNotification(const char *msg) {
  if(isQuietTime()) {
    return false;
  }

  uint8_t chatCount = sizeof(chatIds)/sizeof(chatIds[0]);
  boolean sent = false;
  for(uint8_t i = 0; i < chatCount; i++) {
    if(bot.sendMessage(chatIds[i], String(msg), "")) {
      sent = true;
    } else {
      Serial.printf("Failed to send status to %s\n", chatIds[i]);
    }
  }
  return sent;
}

boolean isBLEUpdateRequired() {
  time_t now = time(NULL);
  if(difftime(now, flowerData.lastUpdate) > minTimeBetweenUpdates) {
    return true;
  }
  for(uint8_t i = 0; i < 4; i++) {
    if(difftime(now, thermometers[i].lastUpdate) > minTimeBetweenUpdates) {
      return true;
    }
  }
  Serial.println("No update required");
  return false;
}

boolean sendStatus(const char* recipient) {
    char msgBuffer[800];
    
    snprintf(msgBuffer, sizeof(msgBuffer), 
      "Guave: %d%% Feuchtigkeit, %dlux, %.1f°C, %dmS/cm, %d%% Batterie\nPflanzenlampe: %s\n",
      flowerData.moisture,
      flowerData.light,
      flowerData.temperature,
      flowerData.conductivity,
      flowerData.battery,
      plantPowerStatus ? "an" : "aus"
    );
    char thermoBuffer[100];
    for(uint8_t i = 0; i < 4; i++) {
      thermometers[i].toString(thermoBuffer, sizeof(thermoBuffer));
      strcat(msgBuffer, thermoBuffer);
    }
    return bot.sendMessage(statusRequestor, String(msgBuffer), "");
}

void setup() {
  Serial.begin(115200);
  // Free up some heap by unloading bluetooth classic stuff
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  // Light up the LED to indicate that we are awake
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  watchdogTimer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(watchdogTimer, &resetModule, true);  //attach callback
  timerAlarmWrite(watchdogTimer, wdtTimeout * 1000, false); //set time in us
  timerAlarmEnable(watchdogTimer);                          //enable interrupt

  //esp_log_level_set("wifi", ESP_LOG_INFO); 

  wifiManager.setDebugOutput(false);
  currentMode = WIFI;
  startWifi();

  configTzTime(timeZone, ntpServer);
  printLocalTime();
  
  Serial.printf("Is quiet time: %d\n", isQuietTime());
}

void loop() {
  timerWrite(watchdogTimer, 0);
  if(((difftime(time(NULL), bleLastUpdate) > bleUpdateInterval && !isQuietTime()) || bleForceUpdate) && isBLEUpdateRequired()) {
    bleForceUpdate = false;
    Serial.print("---------- ");
    printLocalTime();

    updateSensorData();
    updatePlantPower();

    applyRules();
  } 

  if(strcmp(statusRequestor, "") != 0) {
    if(sendStatus(statusRequestor)) {
      strcpy(statusRequestor, "");
    } else {
      Serial.println("Could not send status");
    }
  }

  handleTelegram();

  if(strcmp(statusRequestor, "") == 0) {
    Serial.println("Going to sleep");
    digitalWrite(LED, HIGH);
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
  }
}