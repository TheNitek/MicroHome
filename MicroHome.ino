#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>     
#include <ArduinoJson.h>
#include <UniversalTelegramBot.h>
#include <HTTPClient.h>
#include <NimBLEDevice.h>
//#include "BLEDevice.h"
#include "time.h"
#include "Config.h"
#include "mbedtls/ccm.h"

static const uint8_t LED = 2;

const char* NTP_SERVER = "fritz.box";
const char* TIME_ZONE = "CET-1CEST,M3.5.0,M10.5.0/3";

// Initialize Telegram BOT
//#define BOTtoken "asdasdasd"  // your Bot Token (Get from Botfather)

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

BLEScan* pBLEScan;

const uint16_t MIN_TIME_BETWEEN_ALERTS = 60*60;

const uint8_t NO_ALARM_BEFORE = 6;
const uint8_t NO_ALARM_BEFORE_WEEKEND = 8;
const uint8_t NO_ALARM_AFTER = 22;

const uint16_t SENSOR_TIMEOUT = 60*60;
const uint16_t MIN_TIME_BETWEEN_TIMEOUT_ALERTS = 60*60;

const int BLE_WDT_TIMEOUT = 5*60;
hw_timer_t *bleWatchdogTimer = nullptr;

const uint8_t HTTP_MAX_ERRORS = 5;
volatile uint8_t httpErrors = 0;

const uint16_t BOT_CHECK_INTERVAL = 5; //mean time between scan messages
RTC_DATA_ATTR time_t botLastUpdate = 0;   //last time messages' scan has been done
const char *CHAT_IDS[] = {"302777211", "1070698297"};

const uint16_t POWER_CHECK_INTERVAL = 5*60;

enum maybe_t { M_TRUE, M_MAYBE, M_FALSE};

class PowerSwitch {
  public:
    PowerSwitch(const char* _name, const char* _ip, const uint8_t _socket = 1) : name(_name), IP(_ip), SOCKET(_socket) {

    }

    const char* name;
    boolean powerStatus = false;
    time_t lastUpdate = 0;


    boolean setPower(boolean power) {
      const char* urlTemplate = (power ? ON_URL : OFF_URL);

      char url[100] = "";
      snprintf(url, sizeof(url), urlTemplate, IP, SOCKET);

      return httpGetPower(url);
    }

    boolean togglePower() {
      char url[100] = "";
      snprintf(url, sizeof(url), TOGGLE_URL, IP, SOCKET);

      return httpGetPower(url);
    }

    boolean getPower() {
      if(difftime(time(NULL), lastUpdate) < POWER_CHECK_INTERVAL) {
        return powerStatus;
      }
      
      char url[100] = "";
      snprintf(url, sizeof(url), STATUS_URL, IP, SOCKET);

      httpGetPower(url);

      return powerStatus;
    }

    void toString(char* output, uint16_t outputSize) {
      getPower();
      snprintf(output, outputSize, 
        "%s: %s\n",
        name,
        (lastUpdate != 0) ? (powerStatus ? "an" : "aus") : "-"
      );
    }

  private:  
    const char ON_URL[32] = "http://%s/cm?cmnd=Power%d%%20On";
    const char OFF_URL[33] = "http://%s/cm?cmnd=Power%d%%20Off";
    const char TOGGLE_URL[36] = "http://%s/cm?cmnd=Power%d%%20Toggle";
    const char STATUS_URL[26] = "http://%s/cm?cmnd=Power%d";
    const char* IP;
    const uint8_t SOCKET;

    boolean httpGetPower(const char* url) {
      WiFiClient client;
      HTTPClient http;
      if(!http.begin(client, url)) {
        Serial.println("Could not begin HTTPClient");
        return false;
      }

      Serial.printf("Executing get: %s\n", url);
      int httpCode = http.GET();

      if(httpCode == HTTP_CODE_OK) {
        httpErrors = 0;

        String response = http.getString();
        StaticJsonDocument<200> doc;
        DeserializationError error = deserializeJson(doc, response);

        if (!error) {
          const char* power = doc["POWER"];
          if(!power) {
            char nodeName[10] = "";
            snprintf(nodeName, sizeof(nodeName), "POWER%d", SOCKET);
            power = doc[nodeName];
          }
          if(power) {
            powerStatus = (strcmp(power, "ON") == 0);
            http.end();
            time(&lastUpdate);
            return true;
          } else {
            Serial.print("Could not find power status: "); Serial.println(response);
          }
        } else {
          Serial.print("deserializeJson() failed: "); Serial.println(error.c_str());
        }
      } else if(httpCode > 0) {
        Serial.printf("Unexpected HTTP response: %d\n", httpCode);
      } else {
        Serial.printf("Error during HTTP call: %s\n", http.errorToString(httpCode).c_str());
      }

      http.end();
      return false;
    }
};

class Sensor {
  public:
    Sensor(const char* deviceName, std::string stringAddress, const char* stringKey = nullptr) : NAME(deviceName), ADDRESS(stringAddress) {
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

    const char* NAME;
    const BLEAddress ADDRESS;
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
      char moist[5] = "-";
      char lux[5] = "-";
      char temp[5] = "-";
      char conduct[5] = "-";
      char time[6] = "-";

      if(lastUpdate != 0) {
        struct tm* tm = localtime(&lastUpdate);
        strftime(time, sizeof(time), "%H:%M", tm);
      }

      if(moisture != std::numeric_limits<uint8_t>::max()) {
        snprintf(moist, sizeof(moist), "%d", moisture);
      }

      if(light != std::numeric_limits<uint32_t>::max()) {
        snprintf(lux, sizeof(lux), "%d", light);
      }

      if(temperature != std::numeric_limits<float>::max()) {
        snprintf(temp, sizeof(temp), "%.1f", temperature);
      }

      if(conductivity != std::numeric_limits<uint16_t>::max()) {
        snprintf(conduct, sizeof(conduct), "%d", conductivity);
      }

      snprintf(output, outputSize, 
        "Guave (%s): %s%% Feuchtigkeit, %slux, %s°C, %smS/cm\n",
        time,
        moist,
        lux,
        temp,
        conduct
      );
    }
};

class ThermoData: public Sensor {
  public:
    using Sensor::Sensor;

    void toString(char* output, uint16_t outputSize) {
      char temp[5] = "-";
      char humid[5] = "-";
      char bat[4] = "-";
      char time[6] = "-";

      if(lastUpdate != 0) {
        struct tm* tm = localtime(&lastUpdate);
        strftime(time, sizeof(time), "%H:%M", tm);
      }

      if(temperature != std::numeric_limits<float>::max()) {
        snprintf(temp, sizeof(temp), "%.1f", temperature);
      }

      if(humidity != std::numeric_limits<uint8_t>::max()) {
        snprintf(humid, sizeof(humid), "%d", humidity);
      }

      if(battery != std::numeric_limits<uint8_t>::max()) {
        snprintf(bat, sizeof(bat), "%d", battery);
      }

      snprintf(output, outputSize,
        "%s (%s): %s°C, %s%%, (%s%%)\n",
        NAME,
        time,
        temp,
        humid,
        bat
      );
    }
};

PowerSwitch plantLight("PlantLight", "192.168.1.181");
PowerSwitch outdoor1("Outdoor1", "192.168.1.195", 1);
PowerSwitch outdoor2("Outdoor2", "192.168.1.195", 2);

RTC_DATA_ATTR FlowerData guava("Guave", "c4:7c:8d:65:e9:92");

RTC_DATA_ATTR ThermoData thermometers[] = {
  ThermoData("Küche", "a4:c1:38:46:e6:72"),
  ThermoData("Wohnzimmer", "e7:2e:01:42:a1:1f"),
  ThermoData("Schlafzimmer", "a4:c1:38:9a:f6:a1"),
  ThermoData("Bad", "a4:c1:38:18:93:8a"),
  ThermoData("Klo", "a4:c1:38:f7:53:53"),
  ThermoData("Kinderzimmer", "a4:c1:38:22:6a:22")
};
uint8_t thermometerCount = sizeof(thermometers)/(sizeof(thermometers[0]));

RTC_DATA_ATTR struct {
  time_t guavaTemperatureLow = 0;
  time_t guavaTemperatureHigh = 0;
  time_t guavaWaterLow = 0;
  time_t guavaWaterHigh = 0;
  time_t vent = 0;
  time_t noData = 0;
} notificationStatus;

void IRAM_ATTR resetModule() {
  Serial.println("watchdog reset");
  esp_sleep_enable_timer_wakeup(1);
  esp_deep_sleep_start();
}

void startWifi() {
  timerWrite(bleWatchdogTimer, 0);

  Serial.println("Connecting Wifi");

  WiFiManager wifiManager;
  wifiManager.setDebugOutput(false);
  wifiManager.setEnableConfigPortal(false);
  wifiManager.setTimeout(60);
  wifiManager.setConnectRetries(3);
  uint8_t i = 0;
  while(!wifiManager.autoConnect() && i++ < 3) {
    Serial.println("Retry autoConnect");
    WiFi.disconnect();
    WiFi.mode(WIFI_OFF);
  }
  if(!WiFi.isConnected()) {
    timerStop(bleWatchdogTimer);
    wifiManager.setEnableConfigPortal(true);
    wifiManager.autoConnect("NaeveBot", "botbotbot");
    timerStart(bleWatchdogTimer);
  }

  Serial.print("WiFi connected with IP: "); Serial.println(WiFi.localIP());
}

class MiDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice* advertisedDevice) {
    //void onResult(BLEAdvertisedDevice advertisedDevice1) {
      //BLEAdvertisedDevice* advertisedDevice = &advertisedDevice1;
      timerWrite(bleWatchdogTimer, 0);

      if(!advertisedDevice->haveServiceData()) {
        return;
      }

      if(advertisedDevice->getServiceData().length() <= 12) {
        return;
      }

      // [0] Flags
      // [4] Frame counter
      // [12] Data type
      // [14] Data length
      std::string payloadString = advertisedDevice->getServiceData();
      uint8_t* payload = (uint8_t*) payloadString.c_str();
      size_t payloadLength = payloadString.length();

      Sensor* sensor = nullptr;
      for(uint8_t i = 0; i < thermometerCount; i++) {
        if(thermometers[i].ADDRESS.equals(advertisedDevice->getAddress())) {
          sensor = &thermometers[i];
          break;
        }
      }

      if(sensor == nullptr && guava.ADDRESS.equals(advertisedDevice->getAddress())) {
        sensor = &guava;
      }

      if(sensor == nullptr) {
        return;
      }

      if(sensor->lastFrameCount == payload[4]) {
        return;
      }

      printLocalTime();
      Serial.printf("%s ", sensor->NAME);


      if(!(advertisedDevice->getServiceData().at(2) == 0x98 && advertisedDevice->getServiceData().at(3) == 0x00) &&  // FLORA
          !(advertisedDevice->getServiceData().at(2) == 0x5b && advertisedDevice->getServiceData().at(3) == 0x04) && // LYWSD02MMC
          !(advertisedDevice->getServiceData().at(2) == 0x5b && advertisedDevice->getServiceData().at(3) == 0x05)) { // LYWSD03MMC stock
        /*for(uint8_t i=0; i<payloadLength; i++) {
          Serial.printf("%02x ", payload[i]);
          Serial.println();
        }*/
        if(advertisedDevice->getServiceData().at(2) == 0x1A && advertisedDevice->getServiceData().at(3) == 0x18) { // LYWSD03MMC custom firmware
          if(payloadLength != 18) {
            Serial.println("Invalid payload length from ATC firmware");
            return;
          }
          /* Byte 11-12 Temperature in int16
            Byte 13 Humidity in percent
            Byte 14 Battery in percent
            Byte 15-16 Battery in mV uint16_t
            Byte 17 frame packet counter
          */
          int16_t temp = (int16_t(payload[11]) | (int16_t(payload[12]) << 8));
          uint8_t humidity = payload[13];
          uint8_t battery = payload[14];
          Serial.print(temp);
          Serial.printf(" %d %d\n", humidity, battery);
          return;
        }
        Serial.println();
        return;
      }

      // Encrypted?
      if(payload[0] & 0x08) {
        // Encrypted

        if(sensor->bindkey.empty()) {
          Serial.printf("Got encrypted payload but no key for %s\n", sensor->NAME);
          return;
        }

        if (!((payloadLength == 19) || ((payloadLength >= 22) && (payloadLength <= 24)))) {
          Serial.printf("wrong size (%d)\n", payloadLength);
          return;
        }

        size_t datasize = (payloadLength == 19) ? payloadLength - 12 : payloadLength - 18;
        uint8_t cipherPos = (payloadLength == 19) ? 5 : 11;

        uint8_t iv[16] = {0};
        memcpy(iv, payload + 5, 6);                     // MAC in reverse
        memcpy(iv + 6, payload + 2, 3);                 // sensor type (2) + packet id (1)
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

      // Not sure about the second one, but it works
      uint8_t offset = (payload[0] & 0x20 || payload[0] & 0x08) ?  12 : 11;
      uint8_t dataType = payload[offset];
      uint8_t dataLength = payload[offset+2];

      if ((dataType == 0x04) && (dataLength == 2)) {
        sensor->temperature = (uint16_t(payload[offset+3]) | (uint16_t(payload[offset+4]) << 8)) / 10.0f;
        Serial.printf("Temp: %2.1f° ", sensor->temperature);
      } else if ((dataType == 0x06) && (dataLength == 2)) {
        sensor->humidity = (uint16_t(payload[offset+3]) | (uint16_t(payload[offset+4]) << 8)) / 10;
        Serial.printf("Humid: %d%% ", sensor->humidity);
      } else if ((dataType == 0x07) && (dataLength == 3)) {
        sensor->light = uint32_t(payload[offset+3]) | (uint32_t(payload[offset+4]) << 8) | (uint32_t(payload[offset+5]) << 16);
        Serial.print("Light: "); Serial.print(sensor->light); Serial.print(" ");
      } else if ((dataType == 0x08) && (dataLength == 1)) {
        sensor->moisture = payload[offset+3];
        Serial.printf("Moisture: %d%% ", sensor->moisture);
      } else if ((dataType == 0x09) && (dataLength == 2)) {
        sensor->conductivity = uint16_t(payload[offset+3]) | (uint16_t(payload[offset+4]) << 8);
        Serial.printf("Conductivity: %d ", sensor->moisture);
      } else if ((dataType == 0x0A) && (dataLength == 1)) {
        sensor->battery = payload[offset+3];
        Serial.printf("Battery: %d%% ", sensor->battery);
      } else if ((dataType == 0x0D) && (dataLength == 4)) {
        sensor->temperature = (uint16_t(payload[offset+3]) | (uint16_t(payload[offset+4]) << 8)) / 10.0f;
        sensor->humidity = (uint16_t(payload[offset+5]) | (uint16_t(payload[offset+6]) << 8)) / 10.0;
        Serial.printf("Temp: %2.1f° ", sensor->temperature);
        Serial.printf("Humid: %d%% ", sensor->humidity);
      } else {
        Serial.printf("unknown: %02x: ", dataType);
        for(uint8_t i=0; i<payloadLength; i++) {
          Serial.printf("%02x ", payload[i]);
        }
      }
      Serial.println();

      sensor->lastFrameCount = payload[4];
      time(&(sensor->lastUpdate));
      pBLEScan->clearResults();
    }
};

void handleTelegram() {
  if (difftime(time(NULL), botLastUpdate) > BOT_CHECK_INTERVAL)  {
    int numNewMessages = bot.getUpdates(bot.last_message_received + 1);

    while (numNewMessages) {
      Serial.println("got message");
      httpErrors = 0;
      for (int i = 0; i < numNewMessages; i++) {
        Serial.print("ChatId: "); Serial.println(bot.messages[i].chat_id);
        if (bot.messages[i].text.startsWith("/status")) {
          sendStatus(bot.messages[i].chat_id);
          //strcpy(statusRequestor, bot.messages[i].chat_id.c_str());
          //bot.sendChatAction(statusRequestor, "typing");
          //return;
        } else if (bot.messages[i].text == "/power toggle outdoor1") {
          outdoor1.togglePower();
          sendStatus(bot.messages[i].chat_id);
        } else if (bot.messages[i].text == "/power toggle outdoor2") {
          outdoor2.togglePower();
          sendStatus(bot.messages[i].chat_id);
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
  Serial.print(&timeinfo, "%A, %B %d %Y %H:%M:%S: ");
}

boolean isQuietTime() {
  tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return false;
  }
  if(timeinfo.tm_hour >= NO_ALARM_AFTER) {
    return true;
  }
  switch(timeinfo.tm_wday) {
    case 0:
    case 6:
      // Weekend
      return (timeinfo.tm_hour < NO_ALARM_BEFORE_WEEKEND);
    default:
      // Not weekend
      return (timeinfo.tm_hour < NO_ALARM_BEFORE);
  }
}

void applyRules() {
  if(isQuietTime()) {
    return;
  }

  // Only notify if data have been received at least once
  if(guava.moisture != std::numeric_limits<uint8_t>::max()) {
    if(guava.moisture < 20) {
      if(difftime(time(NULL), notificationStatus.guavaWaterLow) > MIN_TIME_BETWEEN_ALERTS) {
        char msg[50];
        snprintf(msg, sizeof(msg), "Die Guave hat Durst! (Feuchtigkeit: %d%%)", guava.moisture);
        if(sendNotification(msg)) {
          time(&notificationStatus.guavaWaterLow);
        }
      }
    // Checking for 25% to avoid toggling and watering will increase moisture way more
    } else if((notificationStatus.guavaWaterLow != 0) && ((difftime(time(NULL), notificationStatus.guavaWaterLow) > MIN_TIME_BETWEEN_ALERTS) || (guava.moisture >= 25))) {
      sendNotification("Der Guave hat genug zu trinken");
      notificationStatus.guavaWaterLow = 0;
    }
    if(guava.moisture > 60) {
      if(difftime(time(NULL), notificationStatus.guavaWaterHigh) > MIN_TIME_BETWEEN_ALERTS) {
        char msg[50];
        snprintf(msg, sizeof(msg), "Die Guave ertrinkt! (Feuchtigkeit: %d%%)", guava.moisture);
        if(sendNotification(msg)) {
          time(&notificationStatus.guavaWaterHigh);
        }
      }
    } else if(notificationStatus.guavaWaterHigh != 0 && ((difftime(time(NULL), notificationStatus.guavaWaterHigh) > MIN_TIME_BETWEEN_ALERTS) || (guava.moisture <= 56))) {
      sendNotification("Der Guave ertrinkt nicht mehr");
      notificationStatus.guavaWaterHigh = 0;
    }
  }
  if(guava.temperature != std::numeric_limits<float>::max()) {
    if(guava.temperature < 5) {
      if(difftime(time(NULL), notificationStatus.guavaTemperatureLow) > MIN_TIME_BETWEEN_ALERTS) {
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
      if(difftime(time(NULL), notificationStatus.guavaTemperatureHigh) > MIN_TIME_BETWEEN_ALERTS) {
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
  if((guava.lastUpdate > 0) && (difftime(time(NULL), guava.lastUpdate) > SENSOR_TIMEOUT) && (difftime(time(NULL), notificationStatus.vent) > MIN_TIME_BETWEEN_TIMEOUT_ALERTS)) {
    if(sendNotification("Die Guave liefert keine Daten")) {
      time(&notificationStatus.noData);
    }
  }


  maybe_t vent = M_FALSE;
  char msgBuffer[400] = "Bitte lüften!\n";
  char sensorBuffer[100];
  for(uint8_t i = 0; i < thermometerCount; i++) {
    if(thermometers[i].humidity == std::numeric_limits<uint8_t>::max()) {
      continue;
    }
    if(thermometers[i].humidity > 60) {
      vent = M_TRUE;
      thermometers[i].toString(sensorBuffer, sizeof(sensorBuffer));
      strcat(msgBuffer, sensorBuffer);
    } else if(thermometers[i].humidity > 58) {
      // Avoid toggling by adding a 2% tollerance
      vent = M_MAYBE;
    }
    if((thermometers[i].lastUpdate > 0) && (difftime(time(NULL), thermometers[i].lastUpdate) > SENSOR_TIMEOUT) && (difftime(time(NULL), notificationStatus.vent) > MIN_TIME_BETWEEN_TIMEOUT_ALERTS)) {
      snprintf(sensorBuffer, sizeof(sensorBuffer), "%s liefert keine Daten",thermometers[i].NAME);
      if(sendNotification(sensorBuffer)) {
        time(&notificationStatus.noData);
      }
    }
  }

  if(vent == M_TRUE) {
    if(difftime(time(NULL), notificationStatus.vent) > MIN_TIME_BETWEEN_ALERTS) {
      if(sendNotification(msgBuffer)) {
        time(&notificationStatus.vent);
      }
    }
  } else {
    if(notificationStatus.vent != 0 && (vent == M_FALSE || difftime(time(NULL), notificationStatus.vent) > MIN_TIME_BETWEEN_ALERTS)) {
      sendNotification("Erfolgreich gelüftet!");
      notificationStatus.vent = 0;
    }
  }
}

boolean sendNotification(const char *msg) {
  if(isQuietTime()) {
    return false;
  }

  uint8_t chatCount = sizeof(CHAT_IDS)/sizeof(CHAT_IDS[0]);
  boolean sent = false;
  for(uint8_t i = 0; i < chatCount; i++) {
    if(bot.sendMessage(CHAT_IDS[i], String(msg))) {
      httpErrors = 0;
      sent = true;
    } else {
      httpErrors++;
      Serial.printf("Failed to send notification to %s\n", CHAT_IDS[i]);
    }
  }
  return sent;
}

boolean sendStatus(String recipient) {
    char msgBuffer[1000];
    
    guava.toString(msgBuffer, sizeof(msgBuffer));

    char tmpBuffer[100];
    for(uint8_t i = 0; i < thermometerCount; i++) {
      thermometers[i].toString(tmpBuffer, sizeof(tmpBuffer));
      strcat(msgBuffer, tmpBuffer);
    }
    plantLight.toString(tmpBuffer, sizeof(tmpBuffer));
    strcat(msgBuffer, tmpBuffer);

    /*outdoor1.toString(tmpBuffer, sizeof(tmpBuffer));
    strcat(msgBuffer, tmpBuffer);

    outdoor2.toString(tmpBuffer, sizeof(tmpBuffer));
    strcat(msgBuffer, tmpBuffer);*/

    snprintf(tmpBuffer, sizeof(tmpBuffer), "Heap: %d\n", ESP.getFreeHeap());
    strcat(msgBuffer, tmpBuffer);
    if(bot.sendMessage(recipient, String(msgBuffer), "")) {
      httpErrors = 0;
      return true;
    } else {
      httpErrors++;
      Serial.println("Failed to send status");
      return false;
    }
}

void scanBLE(void * parameter) {
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MiDeviceCallbacks());
  pBLEScan->setActiveScan(false);
  pBLEScan->start(0);
}

void setup() {
  Serial.begin(115200);
  // Free up some heap by unloading bluetooth classic stuff
  esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  // Light up the LED to indicate that we are awake
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  bleWatchdogTimer = timerBegin(0, 80, true);                  //timer 0, div 80
  timerAttachInterrupt(bleWatchdogTimer, &resetModule, true);  //attach callback
  timerAlarmWrite(bleWatchdogTimer, BLE_WDT_TIMEOUT * 1000000, false); //set time in us
  timerAlarmEnable(bleWatchdogTimer);                          //enable interrupt

  //esp_log_level_set("wifi", ESP_LOG_INFO); 
  client.setInsecure();

  startWifi();

  configTzTime(TIME_ZONE, NTP_SERVER);
  printLocalTime();

  bot.sendMessage(CHAT_IDS[0], "Reboot", "");
  
  Serial.printf("Is quiet time: %d\n", isQuietTime());

  bot.longPoll = 30;

  BLEDevice::init("Collector");

  xTaskCreate(&scanBLE, "BLETask", 10000, NULL, 1, NULL); 
}

void loop() {
  applyRules();

  handleTelegram();

  if(httpErrors > HTTP_MAX_ERRORS) {
    Serial.println("Too many http errors. Resetting");
    esp_sleep_enable_timer_wakeup(1);
    esp_deep_sleep_start();
  }

  delay((std::min(BOT_CHECK_INTERVAL, POWER_CHECK_INTERVAL)));
}