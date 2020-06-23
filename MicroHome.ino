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
#include "mbedtls/ccm.h"

static const uint8_t LED = 2;

const char* ntpServer = "fritz.box";
const char* timeZone = "CET-1CEST,M3.5.0,M10.5.0/3";

// Initialize Telegram BOT
//#define BOTtoken "asdasdasd"  // your Bot Token (Get from Botfather)

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

BLEScan* pBLEScan;

const uint16_t minTimeBetweenAlerts = 60*60;

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

const uint16_t botCheckInterval = 5; //mean time between scan messages
RTC_DATA_ATTR time_t botLastUpdate;   //last time messages' scan has been done
const char *chatIds[] = {"302777211", "1070698297"};


class Sensor {
  public:
    Sensor(const char* deviceName, std::string stringAddress, const char* stringKey = nullptr) : name(deviceName), address(stringAddress) {
      if(stringKey != nullptr && strlen(stringKey) == 32) {
        char tmpByte[3] = "";
        bindkey.reserve(16);
        for(int i = 0; i < 16; i++) {
          strncpy(tmpByte, &(stringKey[i * 2]), 2);
           bindkey.push_back(std::strtoul(tmpByte, NULL, 16));
        }
      }
    };

    virtual void toString(char* output, uint16_t outputSize) {};

    const char* name;
    BLEAddress address;
    float temperature = std::numeric_limits<float>::max();
    uint8_t humidity = std::numeric_limits<uint8_t>::max();
    uint8_t moisture = std::numeric_limits<uint8_t>::max();
    uint32_t light = std::numeric_limits<uint32_t>::max();
    uint16_t conductivity = std::numeric_limits<uint16_t>::max();
    uint8_t battery = std::numeric_limits<uint8_t>::max();
    std::vector<uint8_t> bindkey; 
    uint8_t lastFrameCount = 0;
    time_t lastUpdate = 0;
};

class FlowerData: public Sensor {
  public:
    using Sensor::Sensor;

    void toString(char* output, uint16_t outputSize) {
      snprintf(output, outputSize, 
        "Guave: %d%% Feuchtigkeit, %dlux, %.1f°C, %dmS/cm, %d%% Batterie\nPflanzenlampe: %s\n",
        moisture,
        light,
        temperature,
        conductivity,
        battery,
        plantPowerStatus ? "an" : "aus"
      );
    }
};

class ThermoData: public Sensor {
  public:
    using Sensor::Sensor;

    void toString(char* output, uint16_t outputSize) {
      snprintf(output, outputSize,
        "%s: %.1f°C, %d%%, (%d%% Batterie)\n",
        name,
        temperature,
        humidity,
        battery
      );
    }
};

RTC_DATA_ATTR FlowerData guava("Guave", "c4:7c:8d:65:e9:92");

RTC_DATA_ATTR ThermoData thermometers[] = {
  ThermoData("Wohnzimmer", "a4:c1:38:46:e6:72", BIND_LIVING),
  ThermoData("Wohnzimmer2", "e7:2e:01:42:a1:1f"),
  ThermoData("Schlafzimmer", "a4:c1:38:9a:f6:a1", BIND_BED),
  ThermoData("Bad", "a4:c1:38:18:93:8a", BIND_BATH),
  ThermoData("Kinderzimmer", "a4:c1:38:0f:0d:43", BIND_KID)
};

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

void startWifi() {
  timerWrite(watchdogTimer, 0);

  Serial.println("Connecting Wifi");

  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
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

class MiDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      timerWrite(watchdogTimer, 0);

      if(!advertisedDevice.haveServiceData()) {
        return;
      }

      if(advertisedDevice.getServiceData().length() <= 12) {
        return;
      }

      if(!(advertisedDevice.getServiceData().at(2) == 0x98 && advertisedDevice.getServiceData().at(3) == 0x00) &&   // FLORA
          !(advertisedDevice.getServiceData().at(2) == 0x5b && advertisedDevice.getServiceData().at(3) == 0x04) &&  // LYWSD02MMC
          !(advertisedDevice.getServiceData().at(2) == 0x5b && advertisedDevice.getServiceData().at(3) == 0x05)) { // LYWSD03MMC
        //Serial.println("Unknown device?!");
        return;
      }

      // [0] Flags
      // [4] Frame counter
      // [12] Data type
      // [14] Data length
      std::string payloadString = advertisedDevice.getServiceData();
      uint8_t* payload = (uint8_t*) payloadString.c_str();
      size_t payloadLength = payloadString.length();

      Sensor* sensor = nullptr;
      for(uint8_t i = 0; i < 5; i++) {
        if(thermometers[i].address.equals(advertisedDevice.getAddress())) {
          sensor = &thermometers[i];
          break;
        }
      }

      if(sensor == nullptr && guava.address.equals(advertisedDevice.getAddress())) {
        sensor = &guava;
      }

      if(sensor == nullptr) {
        Serial.printf("Unknown device: %s\n", advertisedDevice.getAddress().toString().c_str());
        return;
      }

      if(sensor->lastFrameCount == payload[4]) {
        return;
      }

      Serial.printf("%s ", sensor->name);

      // Encrypted?
      if(payload[0] & 0x08) {
        // Encrypted

        if(sensor->bindkey.empty()) {
          Serial.printf("Got encrypted payload but no key for %s", sensor->name);
          return;
        }

        if (!((payloadLength == 19) || ((payloadLength >= 22) && (payloadLength <= 24)))) {
          Serial.printf("wrong size (%d)\n", payloadLength);
          return;
        }

        size_t datasize = (payloadLength == 19) ? payloadLength - 12 : payloadLength - 18;
        uint8_t cipherPos = (payloadLength == 19) ? 5 : 11;

        uint8_t iv[16] = {0};
        memcpy(iv, payload + 5, 6);               // MAC in reverse
        memcpy(iv + 6, payload + 2, 3);             // sensor type (2) + packet id (1)
        memcpy(iv + 9, payload + payloadLength - 7, 3); // payload counter

        mbedtls_ccm_context ctx;
        mbedtls_ccm_init(&ctx);

        if(mbedtls_ccm_setkey(&ctx, MBEDTLS_CIPHER_ID_AES, sensor->bindkey.data(), 16 * 8)) {
          Serial.println("setkey failed");
          mbedtls_ccm_free(&ctx);
          return;
        }


        const uint8_t add[16] = {0x11};
        uint8_t plainText[16] = {0};
        const uint8_t tagsize = 4;

        uint32_t ret = mbedtls_ccm_auth_decrypt(&ctx, datasize, iv, 12, add, 1,
                                      payload + cipherPos, plainText, payload + payloadLength - tagsize, tagsize);
        if(ret) {
          Serial.print("decrypt failed: ");
          Serial.println(ret);
          mbedtls_ccm_free(&ctx);
          return;
        }

        mbedtls_ccm_free(&ctx);

        memcpy(payload+cipherPos+1, plainText, datasize);
      }

      /*for(uint8_t i=0; i<6; i++) {
        Serial.printf("%02x ", payload[i]);
      }*/

      uint8_t dataType = payload[12];
      uint8_t dataLength = payload[14];

      if ((dataType == 0x04) && (dataLength == 2)) {
        sensor->temperature = (uint16_t(payload[15]) | (uint16_t(payload[16]) << 8)) / 10.0f;
        Serial.printf("Temp: %2.1f ", sensor->temperature);
      } else if ((dataType == 0x06) && (dataLength == 2)) {
        sensor->humidity = (uint16_t(payload[15]) | (uint16_t(payload[16]) << 8)) / 10;
        Serial.printf("Humid: %d ", sensor->humidity);
      } else if ((dataType == 0x07) && (dataLength == 3)) {
        sensor->light = uint32_t(payload[15]) | (uint32_t(payload[16]) << 8) | (uint32_t(payload[17]) << 16);
        Serial.print("Light: "); Serial.print(sensor->light); Serial.print(" ");
      }
      else if ((dataType == 0x08) && (dataLength == 1)) {
        sensor->moisture = payload[15];
        Serial.printf("Moisture: %d%% ", sensor->moisture);
      }
      else if ((dataType == 0x09) && (dataLength == 2)) {
        sensor->conductivity = uint16_t(payload[15]) | (uint16_t(payload[16]) << 8);
        Serial.printf("Conductivity: %d ", sensor->moisture);
      } else if ((dataType == 0x0A) && (dataLength == 1)) {
        sensor->battery = payload[15];
        Serial.printf("Battery: %d%% ", sensor->battery);
      } else {
        Serial.printf("unknown: %02x ", dataType);
      }
      Serial.println(ESP.getFreeHeap());

      sensor->lastFrameCount = payload[4];
      time(&(sensor->lastUpdate));
      pBLEScan->clearResults();

    }
};

void scanBLE(void * parameter) {
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MiDeviceCallbacks(), true);
  pBLEScan->setActiveScan(false);
  pBLEScan->setInterval(100);
  pBLEScan->setWindow(99);
  pBLEScan->start(0);
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
          sendStatus(bot.messages[i].chat_id);
          //strcpy(statusRequestor, bot.messages[i].chat_id.c_str());
          //bot.sendChatAction(statusRequestor, "typing");
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

void applyRules() {
    tm timeinfo;
    if(!getLocalTime(&timeinfo)){
      Serial.println("Failed to obtain time");
    } else if(timeinfo.tm_hour >= plantOnHour && timeinfo.tm_hour < plantOffHour && guava.light < plantMinLux) {
      setPlantPower(true);
    } else {
      setPlantPower(false);
    }

    // Only notify if data have been received at least once
    if(guava.moisture != std::numeric_limits<uint8_t>::max()) {
      if(guava.moisture < 15) {
        if(difftime(time(NULL), notificationStatus.guavaWaterLow) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Die Guave hat durst! (Feuchtigkeit: %d%%)", guava.moisture);
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
      if(guava.moisture > 60) {
        if(difftime(time(NULL), notificationStatus.guavaWaterHigh) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Die Guave ertrinkt! (Feuchtigkeit: %d%%)", guava.moisture);
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
    }
    if(guava.temperature != std::numeric_limits<float>::max()) {
      if(guava.temperature < 5) {
        if(difftime(time(NULL), notificationStatus.guavaTemperatureLow) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Der Guave ist kalt! (Temperatur: %.1f°C)", guava.temperature);
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
      if(guava.temperature > 35) {
        if(difftime(time(NULL), notificationStatus.guavaTemperatureHigh) > minTimeBetweenAlerts) {
          char msg[50];
          snprintf(msg, sizeof(msg), "Der Guave ist warm! (Temperatur: %.1f°C)", guava.temperature);
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
      if(thermometers[i].humidity == std::numeric_limits<uint8_t>::max()) {
        continue;
      }
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
      Serial.printf("Failed to send notification to %s\n", chatIds[i]);
    }
  }
  return sent;
}

boolean sendStatus(String recipient) {
    char msgBuffer[800];
    
    guava.toString(msgBuffer, sizeof(msgBuffer));

    char thermoBuffer[100];
    for(uint8_t i = 0; i < 4; i++) {
      thermometers[i].toString(thermoBuffer, sizeof(thermoBuffer));
      strcat(msgBuffer, thermoBuffer);
    }
    return bot.sendMessage(recipient, String(msgBuffer), "");
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

  startWifi();

  configTzTime(timeZone, ntpServer);
  printLocalTime();
  
  Serial.printf("Is quiet time: %d\n", isQuietTime());

  BLEDevice::init("Collector");
  xTaskCreate(&scanBLE, "BLETask", 10000, NULL, 1, NULL); 
}

void loop() {
  timerWrite(watchdogTimer, 0);
  if(!isQuietTime()) {
    Serial.print("---------- ");
    printLocalTime();

    applyRules();
  } 

  //handleTelegram();
  delay(5 * 60 * 1000);
}