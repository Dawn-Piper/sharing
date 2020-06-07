/* MUST  INCLUDE EVERYTHING IN THE SKETCH FILE */
#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <ESP8266WebServer.h>
#include <ArduinoJson.h>
#include <FS.h> // Include the SPIFFS library
#include <ESP8266HTTPClient.h>
#include <PubSubClient.h>
#include <time.h>
#include <Ticker.h> //Ticker Library
#include <NTPClient.h>
#include <WiFiUdp.h>
#include "buffer.h"
#include "debug.h"
#include "emon.h"
#include "error.h"
#include "FileSystem.h"
#include "fs.h"
#include "Header.h"
#include "http.h"
#include "mqttclient.h"
#include "ntp.h"
#include "Sensor.h"
#include "submit.h"
#include "timing.h"
#include "updater.h"
#include "util.h"
#include "wifi.h"

extern Updater OTAUpdater;

int hardwareVersion = 2;
String meterID;
char cfgString[CFG_DOC_SIZE];
bool isConfigured = false;
struct device_config config;
unsigned int lastSaveCheck = 0;

String getChipId()
{
  char id[10];
  sprintf(id, "%08X", ESP.getChipId());
  String chipid = id;

  return chipid;
}

void ICACHE_RAM_ATTR factoryReset() {
  static uint32_t count = 0;
  static uint32_t lastPress = 0;
  if (count >= 3) {
    if (getTimeSeconds() < lastPress + 2) {
      wipeAndDumpFS();
      ESP.reset();
    } else {
      count = 0;
    }
  } else {
    lastPress = getTimeSeconds();
    count++;
  }
}

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LOW);  // Turn the LED on by making the voltage LOW
  // FLASH BUTTON ISR
  pinMode(D3, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(D3), factoryReset, RISING);

  meterID = getChipId();
  bool ntpSynced = false;
  char errorStateString[16];

#ifdef AUTO_CONFIG
  strcpy(config.connectionServer, "http://data.amplemeter.com:5000/api/submit"); // default server path, can be configured from App
  strcpy(config.connectionWiFiSsid, "TheNet");
  strcpy(config.connectionWiFiPassword, "shiRuying");
  isConfigured = true; // for speeding up test, will be removed
#endif
  strcpy(config.connectionMqttServer, MQTT_SERVER);

  delay(1000);
  Wire.begin();
  Wire.setClock(400000L);
  Serial.begin(115200);
  Serial.flush();

  logger(INFO, String("[MAIN] Version: ") + FIRMWARE_VERSION);
  logger(INFO, "[MAIN] Meter ID (chip ID) is " + meterID);

  if (SPIFFS.begin())
  {
    logger(INFO, "[MAIN] SPIFFS successfully initiated");
  }
  else
  {
    logger(INFO, "[MAIN] failed to initiate SPIFFS");
    return;
  }

  if (!isConfigured)
  {
    readConfig();
  }

  if (!isConfigured)
  {
    configure();
  }

  connectWifi(false);
#ifndef DISABLE_OTA
  OTAUpdater.check(0);
#endif

  /* *************************************** BE WARY OF ADDING ANY CODE ABOVE THIS LINE */

  ntpSynced = setupTiming();
  if (!ntpSynced) {
    logger(INFO, "[NTP] Rebooting device");
    ESP.reset();
  }

  if (!writeLineToFile("TEST", "/write_test")) {
    char fileName[MAX_FILE_NAME_LENGTH];
    while (getNextFile(fileName, "/data", 5, 0)) {
      removeFile(fileName);
      logger(INFO, "Removed File:" + String(fileName));
    }
    while (getNextFile(fileName, "/log", 4, 0)) {
      removeFile(fileName);
      logger(INFO, "Removed File:" + String(fileName));
    }
    logger(ERROR, "[ERR] FS full, all data and log files removed");
  }

  removeFile("/write_test");

  // Can't be logged before NTP sync as needs timestamp
  logger(ERROR, "[ERR][MAIN] Device has rebooted");

  setupBuffers();
  mqttSetup();

  if (getLineFromFile(errorStateString, "/state", 0)) {
    Serial.println("GOT THE LEVEL: " + String(errorStateString));
    if (!strcmp(errorStateString, "ORANGE")) {
      Serial.println("SETTING ORANGE");
      errorState = ORANGE;
    } else if (!strcmp(errorStateString, "RED")) {
      Serial.println("SETTING RED");
      errorState = RED;
    }
  }

  setupEmon(false);

  if (errorState != GREEN) {
    digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
    return; // Skip tests
  }


  logger(INFO, "<<<<<<<<<<<<<<<<<<<<<<<<<<< STARTING DEVICE TESTS >>>>>>>>>>>>>>>>>>>>>>>>>>>");
  logger(INFO, "-----------------------------------------------------------------------------");
  logger(INFO, "[TEST] 1/4 TESTING WIFI");
  if(testWifi()) {
    logger(INFO, "[TEST] 1/4 WIFI TEST PASSED");
  } else {
    logger(INFO, "[TEST] 1/4 WIFI TEST FAILED");
  }
  logger(INFO, "-----------------------------------------------------------------------------");
  logger(INFO, "[TEST] 2/4 TESTING CURRENT READING");
  int startbuffer = currentBuf;
  int startPointer = bufferPosition;
  emonSetVisible(true);
  // The current prints come from the emon module
  delay(10000); //TODO incorperate sample frequency here
  emonSetVisible(false);
  int endbuffer = currentBuf;
  int endPointer = bufferPosition;
  logger(INFO, "[TEST] 2/4 Datarate is " + String((float)(((endbuffer*SAMPLE_BUFFER_SIZE)+endPointer) - ((startbuffer*SAMPLE_BUFFER_SIZE)+startPointer))/10) + "Hz, expected " + String(getSampleFrequency()) + "Hz");
  logger(INFO, "[TEST] 2/4 YOU SHOULD SEE 10 SECONDS OF CURRENT DATA ABOVE");
  logger(INFO, "-----------------------------------------------------------------------------");
  logger(INFO, "[TEST] 3/4 TESTING TIME");
  logger(INFO, "[TEST] 3/4 Time in seconds: " + String(getTimeSeconds()));
  logger(INFO, "[TEST] 3/4 Time in milliseconds: " + uint64ToString(getTimeMilliSeconds()));
  logger(INFO, "[TEST] 3/4 TIME SHOULD MATCH CURRENT EPOCH TIME");
  logger(INFO, "[TEST] 3/4 OR");
  logger(INFO, "[TEST] 3/4 Time: " + getHumanTime());
  logger(INFO, "[TEST] 3/4 Time should match time in GMT timezone");
  logger(INFO, "-----------------------------------------------------------------------------");
  logger(INFO, "[TEST] 4/4 TESTING DATA SUBMIT");
  logger(INFO, "[TEST] 4/4 WAITING FOR ONE BUFFER TO COMPLETE");
  while(currentBuf == 0 && !sampleBufs[0].isFull) {
    logger(INFO, ".");
    delay(1000);
  }
  updateTiming();
  setupBuffers();
  init_dataFilesInOrder();
  logger(INFO, "");
  bufferDump(false);
  connect_and_submitData(config.connectionMqttServer, meterID, config.isOnBattery);
  logger(INFO, "[TEST] 4/4 CHECK GRAFANA FOR DATA");
  logger(INFO, "<<<<<<<<<<<<<<<<<<<<<<<<<<< SUBMIT TEST COMPLETE >>>>>>>>>>>>>>>>>>>>>>>>>>>>");


  digitalWrite(LED_BUILTIN, HIGH);  // Turn the LED off by making the voltage HIGH
  logger(INFO, "");
  logger(INFO, "=============================================================================");
  logger(INFO, "||                             SETUP COMPLETE                              ||");
  logger(INFO, "=============================================================================");
  logger(INFO, "");

  if (config.isOnBattery) {
    Serial.println("On battery");
  } else {
    Serial.println("NOT on battery");
    Serial.printf("Buffer period %u (s)\n", getBufferPeriodSeconds());
  }

  //Flush any remaining data from previous runs
  lastSubmitCheck = getTimeSeconds() - SUBMIT_INTERVAL;
  connect_and_submitData(config.connectionMqttServer, meterID, config.isOnBattery);
}

void configure()
{
  startAccessPoint();
  startServer();

  listenForRequests();
}

void loop()
{
  float fsUsage = getFSUsage();
  bool doLogs = (sendLogs);
  bool doData = (getTimeSeconds() > lastSubmitCheck + SUBMIT_INTERVAL);
  bool doSave = (getTimeSeconds() > lastSaveCheck + SAVE_INTERVAL);
  bool doOTA = (getTimeSeconds() > lastUpdateCheck + UPDATE_CHECK_INTERVAL);
  bool doTiming = ((getTimeSeconds() - lastUpdate) > TIMING_UPDATE_INTERVAL);
  bool doErrorOrange = (errorState == GREEN && fsUsage > ORANGE_ERROR_PERCENT);
  bool doErrorRed = (errorState == ORANGE && fsUsage > RED_ERROR_PERCENT);
  bool doErrorReboot = ((errorState == ORANGE || errorState == RED) && (millis()/1000) > ERROR_REBOOT_INTERVAL);
  bool doDebugLog = (getTimeSeconds() > lastDebugLog + DEBUG_LOG_FEQUENCY);

  if (!config.isOnBattery) {
    doData = (getTimeSeconds() > lastSubmitCheck + getBufferPeriodSeconds());
  }

  bufferDump(false);

  if (doOTA
      || doTiming
      || doErrorOrange
      || doErrorRed
      || doErrorReboot
      || doLogs
      || doData
      || doSave
      || doDebugLog) {

    connectWifi(true);
    if (doOTA) {
      logger(INFO, "[OTA] Checking OTA from MAIN loop");
      if(connectWifi(true)) {
        OTAUpdater.check(getTimeSeconds());
      } else {
        logger(ERROR, "[ERR][OTA] OTA check failed");
      }
    }
    else if (doDebugLog) {
      debugLog();
    }
    else if (doTiming) {
      updateTiming();
    }
    else if (doSave) {
      bool didDump = dumpBuffersToFile();
      if (didDump) {
        lastSaveCheck = getTimeSeconds();
      }
    }
    else if (doErrorOrange) {
      removeFile("/state");
      writeLineToFile("ORANGE", "/state");
      logger(ERROR, "[ERR] FS usage = " + String(fsUsage));
      logger(ERROR, "[ERR] Hit ORANGE error state, rebooting.");
      bufferDump(true);
      ESP.reset();
    }
    else if (doErrorRed) {
      removeFile("/state");
      writeLineToFile("RED", "/state");
      logger(ERROR, "[ERR] FS usage = " + String(fsUsage));
      logger(ERROR, "[ERR] Hit RED error state, rebooting.");
      bufferDump(true);
      ESP.reset();
    }
    else if (doErrorReboot)
    {
      logger(ERROR, "[ERR] Error state reboot.");
      bufferDump(true);
      ESP.reset();
    }
    else if (doLogs || doData) {
      if (doLogs) {
        submitLogs(config.connectionMqttServer, meterID);
      }
      if (doData){
        connect_and_submitData(config.connectionMqttServer, meterID, config.isOnBattery);
        dataBench();
        cleanLogs();
      }
    }
  }
  delay(10000);
}
