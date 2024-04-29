/**************************************************************************************
 * 
 *  Sixfab Pico LTE Tracker Example
 *  Based on the Lilygo SIM7000G Example
 *  Version 1.0
 *  Last updated 2024-04-28
 *  Written by Scott C. Lemon
 *  Based on Lilygo Sample Code - https://github.com/Wovyn/lilygo-t-sim7000g-asset-tracker-example
 * 
 * v1.0 - Initial port of the Lilygo code
 * 
 * TODO
 *      - Test modem reset more - if carrier is lost - will is properly recover with alternate carrier?
 *      - don't wait forever for the first GPS fix?
 *      - Modem Power down and Module deep sleep still not complete 
 * 
 **************************************************************************************/

/*
 * Set serial for debug console (to the Serial Monitor, default speed 115200)
 */
#define SerialMon Serial

/*
 * Set serial for AT commands (to the module)
 * Use Hardware Serial on Mega, Leonardo, Micro
 */
#define SerialAT Serial1

/*
 * See all AT commands, if wanted
 */
// #define DUMP_AT_COMMANDS

/*
 * Define the serial console for debug prints, if needed
 */
#define TINY_GSM_DEBUG SerialMon

/*
 * powerdown modem after tests?
 * 
 * NOTE: this is not yet properly implemented
 * 
 */
#define TINY_GSM_POWERDOWN false

/*
 * set GSM PIN, if any
 */
#define GSM_PIN ""

/*
 * Your cellular definitions
 *   - You must provide your GPRS/Cellular APN and credentials, if any
 *   
 *   - for the Sixfab Pico LTE the APN is: super
 *   - no authentication
 */
const char apn[]  = "super";
const char gprsUser[] = "";
const char gprsPass[] = "";

// include your specific configuration
#include "configuration.h"

/*
 *  this will select the correct driver to use
 *  
 *  NOTE: currently the BG96 driver is being used with the Sixfab Pico LTE
 *        even though the Pico has the BG95-M3
 */
#if (ENABLE_HTTP && USE_HTTPS) || (ENABLE_MQTT && USE_MQTTS)
  #define TINY_GSM_MODEM_BG96   // provides both SSL and non-SSL connections
#else
  #define TINY_GSM_MODEM_BG96
#endif

#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// include all necessary libraries
#include <ArduinoMqttClient.h>
#include <TinyGsmClient.h>
#include <ArduinoHttpClient.h>
#include <SPI.h>

#ifdef DUMP_AT_COMMANDS
  #include <StreamDebugger.h>
  StreamDebugger debugger(SerialAT, SerialMon);
  TinyGsm modem(debugger);
#else
  TinyGsm modem(SerialAT);
#endif

#if ENABLE_HTTP
  // NOTE: These HTTP errors are 0 and negatives, and so abs() must be used to print correctly
  String httpConnectError[] = {
    "HTTP_SUCCESS",
    "HTTP_ERROR_CONNECTION_FAILED",
    "HTTP_ERROR_API",
    "HTTP_ERROR_TIMED_OUT",
    "HTTP_ERROR_INVALID_RESPONSE"
    };

  // HTTP is using modem connection 0
  //
  // Does BG95 support multiple connections?
  //
  #if USE_HTTPS
    TinyGsmClientSecure http_client(modem,0);
    HttpClient http(http_client, server, port);
  #else
    TinyGsmClient http_client(modem,0);
    HttpClient http(http_client, server, port);
  #endif
#endif

#if ENABLE_MQTT
  // NOTE: These MQTT connect errors start at -2 and so must be offset by 2 to print correctly
  String mqttConnectError[] = {
    "MQTT_CONNECTION_REFUSED",
    "MQTT_CONNECTION_TIMEOUT",
    "MQTT_SUCCESS",
    "MQTT_UNACCEPTABLE_PROTOCOL_VERSION",
    "MQTT_IDENTIFIER_REJECTED",
    "MQTT_SERVER_UNAVAILABLE",
    "MQTT_BAD_USER_NAME_OR_PASSWORD",
    "MQTT_NOT_AUTHORIZED"
    };

  // MQTT is using modem connection 1
  #if USE_MQTTS
    TinyGsmClientSecure mqtt_client(modem,1);
    MqttClient mqtt(mqtt_client);
  #else
    TinyGsmClient mqtt_client(modem,1);
    MqttClient mqtt(mqtt_client);
  #endif
#endif

#define uS_TO_S_FACTOR 1000000ULL   /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP       600     /* Time ESP32 will go to sleep (in seconds) */
#define POWER_DOWN_DELAY    180     /* Time ESP32 will delay before power down (in seconds) */

#define TRANSMIT_INTERVAL   120     /* How often should the tracker transmit the current location */
#define WAITING_MESSAGE     15      /* How long between debug waiting messages displayed */

#define GPS_NO_LOCK_DELAY   5       /* How long to wait, looping to regain GPS lock, in seconds */
#define GPS_NO_LOCK_TIMEOUT 30      /* How long to wait, total, before resetting the GPS power */

// pins for the modem connection - specific to the Sixfab Pico LTE board
#define UART_BAUD   115200
#define PIN_DTR     25
#define PIN_RX      1
#define PIN_TX      0
#define PWR_PIN     26
#define PWR_KEY     17
#define LED_PIN     22

// Modem RAT Types
String ratType[] = {"GSM","GSM Compact","UTRAN","GSM w/EGPRS","UTRAN w/HSDPA","UTRAN w/HSUPA","UTRAN w/HSDPA and HSUPA","E-UTRAN","CAT-M","NB-IoT"};

// GPS related variables ...
float gps_lat      = 0;
float gps_lon      = 0;
float gps_speed    = 0;
float gps_alt      = 0;
int   gps_vsat     = 0;
int   gps_usat     = 0;
float gps_accuracy = 0;
int   gps_year     = 0;
int   gps_month    = 0;
int   gps_day      = 0;
int   gps_hour     = 0;
int   gps_min      = 0;
int   gps_sec      = 0;

uint32_t  gotNetwork = millis();
uint32_t  lostNetwork = millis();
uint32_t  secondsWithoutNetwork = -1;
uint32_t  lastTransmitTimestamp = 0;
boolean   networkConnected = false;
boolean   networkConnectedChanged = false;
uint32_t  waitingStarted = 0;
uint32_t  lastWaitingMessage = 0;
boolean   ledState;

/**************************************************************************************
 * 
 * Setup
 * 
 **************************************************************************************/
void setup()
{
    // Set console baud rate
    SerialMon.begin(115200);
    delay(10000);

    Serial.println("Beginning Asset Tracker setup() ...");

    // Set LED OFF
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, HIGH);
    ledState = true;

    // Reset Modem
    pinMode(PWR_PIN, OUTPUT);
    digitalWrite(PWR_PIN, HIGH);
    delay(1000);
    digitalWrite(PWR_PIN, LOW);

    // Lilygo had this in their examples
    DBG("Wait 1 seconds for modem to boot ...");
    delay(1000);

    // Configure modem serial interface
    SerialAT.setRX(PIN_RX);
    SerialAT.setTX(PIN_TX);
    SerialAT.begin(UART_BAUD);

    // Restart takes quite some time
    // To skip it, call init() instead of restart()
    DBG("Initializing modem ...");
    DBG("Toggle PWR_KEY ...");
    pinMode(PWR_KEY, OUTPUT);
    digitalWrite(PWR_KEY, LOW);
    delay(1000);
    digitalWrite(PWR_KEY, HIGH);
    delay(1000);
    digitalWrite(PWR_KEY, LOW);
    while (!modem.init()) {
        DBG("Failed to init modem? Toggle PWR_KEY ...");
        pinMode(PWR_KEY, OUTPUT);
        digitalWrite(PWR_KEY, LOW);
        delay(1000);
        digitalWrite(PWR_KEY, HIGH);
        delay(1000);
        digitalWrite(PWR_KEY, LOW);
        // return;
    }
    DBG("Init Successful!");

    // Display modem name and information
    String name = modem.getModemName();
    delay(1000);
    DBG("Modem Name:", name);
    
    String modemInfo = modem.getModemInfo();
    delay(1000);
    DBG("Modem Info:", modemInfo);

    String firmware;
    modem.sendAT("+CGMR");
    if (modem.waitResponse(10000L, firmware) != 1) {
      DBG("Firmware: FAILED");
    } else {
      firmware.replace("\r\n" "OK" "\r\n", "");
      firmware.replace("_", " ");
      firmware.trim();
      DBG("Firmware: ", firmware);
    }

    // Unlock your SIM card with a PIN if needed
    if ( GSM_PIN && modem.getSimStatus() != 3 ) {
        modem.simUnlock(GSM_PIN);
        DBG("SIM Check ...");
    }
    DBG("... SIM Good!");

/**************************************************************************************
 * 
 *  Enable the GPS
 *  
 **************************************************************************************/

  DBG("Enabling the GPS ...");
  modem.enableGPS();
  DBG("... done!");

  // we want the device to transmit immediately upon boot so set the last TX in the past
  lastTransmitTimestamp = millis() - TRANSMIT_INTERVAL * 1000;

  // initiate a connection
  //modem.gprsConnect(apn, gprsUser, gprsPass);
  //modem.sendAT("+COPS=0");
 
  // entering the loop!
  DBG("Entering main loop!");

}
/**************************************************************************************
 * 
 * Loop
 * 
 **************************************************************************************/
void loop()
{
  /**************************************************************************************
   * 
   *  On each loop we want to detect if we are connected or not.
   *  This will be used to log how long we were out of coverage
   *  
   **************************************************************************************/
    // toggle the User LED
    if (ledState) {
      digitalWrite(LED_PIN, LOW);
      ledState = false;
    } else {
      digitalWrite(LED_PIN, HIGH);
      ledState = true;      
    }

    if (modem.isNetworkConnected()) {
        if (!networkConnected) {
          DBG("Network changed to connected!");
          gotNetwork = millis();
          if (secondsWithoutNetwork == -1) {
            // this is to deal with the first run of the loop
            secondsWithoutNetwork = 0;
          } else {
            // this is for any time after the first run of the loop
            secondsWithoutNetwork = lostNetwork - gotNetwork;            
          }
          networkConnected = true;
          networkConnectedChanged = true;
        } else {
          //DBG("Network still connected!");
          gotNetwork = millis();
          lostNetwork = millis();
          //secondsWithoutNetwork = lostNetwork - gotNetwork;
          networkConnectedChanged = false;          
        }
    } else {
        if (networkConnected) {
          DBG("Network changed to disconnected!");
          lostNetwork = millis();
          //secondsWithoutNetwork = lostNetwork - gotNetwork;
          networkConnected = false;
          networkConnectedChanged = true;
        } else {
          //DBG("Network still disconnected!");
          networkConnectedChanged = false;          
          lostNetwork = millis();
          secondsWithoutNetwork = lostNetwork - gotNetwork;
        }
        //delay(1000);
        //return;      
    }

    /*
     * if we lost the cellular connection try to force a reconnect
     * 
     * NOTE:  This is a TODO item.  I have not yet found the correct combination of commands
     */
    if (!networkConnected && networkConnectedChanged) {
      DBG("Attempting to reconnect!");
      //modem.sendAT("+COPS=0");
      //modem.restart();
      //modem.gprsConnect(apn, gprsUser, gprsPass);
    }

    /*
     * Check to see if we have network:
     *   Delay and return if we don't
     *   Print waiting to send messages if we do
     */
    if (!modem.waitForNetwork()) {
        DBG("!waitForNetwork");
        delay(1000);
        return;
    } else {
      // delay here just to slow loop and print dots ...
      if (waitingStarted == 0){
        waitingStarted = millis();
        lastWaitingMessage = waitingStarted;
      }

      if (((millis() - lastWaitingMessage) / 1000) >= WAITING_MESSAGE) {
        Serial.println("Waited " + String((millis() - waitingStarted) / 1000) + " seconds of " + String(TRANSMIT_INTERVAL) + " second transmit interval ...");
        lastWaitingMessage = millis();

#if ENABLE_MQTT
        if(mqtt.connected()) {
          Serial.println("Poll MQTT Broker to keep connection alive!");
          mqtt.poll();
        }
#endif

      }
      delay(1000);
    }

    if (networkConnectedChanged) {
      IPAddress local = modem.localIP();
      DBG("Local IP:", local);
    }

    /*
     * is it time to transmit?
     */
    if ((millis() - lastTransmitTimestamp) >= TRANSMIT_INTERVAL * 1000) {

      DBG(">> Time to transmit!");
      DBG("Connecting to APN: ", apn);
      if (!modem.gprsConnect(apn, gprsUser, gprsPass)) {
          DBG("!gprsConnect");
          delay(1000);
          return;      
      }
  
      bool res = modem.isGprsConnected();
      DBG("GPRS status:", res ? "connected" : "not connected");
      
// TODO: what do we do here if it failed above?
  
      // display the SIM and connection information
      String ccid = modem.getSimCCID();
      DBG("CCID:", ccid);
  
      String imsi = modem.getIMSI();
      DBG("IMSI:", imsi);
    
      String imei = modem.getIMEI();
      DBG("IMEI:", imei);
  
      String cop = modem.getOperator();
      DBG("Operator:", cop);
  
      IPAddress local = modem.localIP();
      DBG("Local IP:", local);

      String networkTime = modem.getGSMDateTime(DATE_FULL);
      DBG("Current Network Time:", networkTime);
      
      /************************************************************************
       * 
       * Display the current Radio Access Technology:
       * 
       *  0 GSM
       *  1 GSM Compact
       *  2 UTRAN
       *  3 GSM w/EGPRS
       *  4 UTRAN w/HSDPA
       *  5 UTRAN w/HSUPA
       *  6 UTRAN w/HSDPA and HSUPA
       *  7 E-UTRAN
       *  8 CAT-M
       *  9 NB-IoT
       *  
       ************************************************************************/
      String copsString;
      String currentRatType;
      modem.sendAT("+COPS?");
      if (modem.waitResponse(10000L, copsString) != 1) {
        DBG("copsString: FAILED");
      } else {
        copsString.replace("\r\n" "OK" "\r\n", "");
        copsString.replace("_", " ");
        copsString.trim();
        DBG("copsString: ", copsString);
        currentRatType = ratType[String(copsString.charAt(copsString.length() - 1)).toInt()];
        DBG("RAT Type: ", currentRatType);
      }

      int csq = modem.getSignalQuality();
      DBG("Signal quality:", csq);
  
      /*
       * check GPS and get our GPS data
       * 
       * NOTE:  We currently wait forever for the first fix
       * 
       */
      int noLockCount = 0;
      while (1) {
        if (modem.getGPS(&gps_lat, &gps_lon, &gps_speed, &gps_alt, &gps_vsat, &gps_usat, &gps_accuracy,
                      &gps_year, &gps_month, &gps_day, &gps_hour, &gps_min, &gps_sec)) {
          DBG("Latitude:", String(gps_lat, 8), "\tLongitude:", String(gps_lon, 8));
          DBG("Speed:", gps_speed, "\tAltitude:", gps_alt);
          DBG("Visible Satellites:", gps_vsat, "\tUsed Satellites:", gps_usat);
          DBG("Accuracy:", gps_accuracy);
          DBG("Year:", gps_year, "\tMonth:", gps_month, "\tDay:", gps_day);
          DBG("Hour:", gps_hour, "\tMinute:", gps_min, "\tSecond:", gps_sec);
          break;
        } else {
          DBG("Waiting for GPS lock ...");
          noLockCount += GPS_NO_LOCK_DELAY;
          DBG("Delay in seconds = ", noLockCount);
          // if it's been GPS_NO_LOCK_TIMEOUT ... reset the GPS!
          if (noLockCount >= GPS_NO_LOCK_TIMEOUT) {
                noLockCount = 0;
                DBG("Reset GPS ...");
                modem.disableGPS();
                delay(1000);
                modem.enableGPS();
                DBG("... done!");
          } // end if (noLockCount > GPS_NO_LOCK_TIMEOUT)
        } // else wait or reset GPS
        delay(GPS_NO_LOCK_DELAY * 1000);
      } // while forever waiting for GPS lock

      /*
       * transform and adjust data values
       * 
       * NOTE: speed is sometimes negative for some reason, set it to zero
       */
      if (gps_speed < 0) {
        gps_speed = 0;
      }
      //  metric -> imperial
      //  kph -> mph
      gps_speed = gps_speed * 0.621371;
      //  meters -> feet
      gps_alt = gps_alt * 3.28084;

#if ENABLE_HTTP
      // post to HTTP endpoint ...
      if (POST_TO_ENDPOINT && ENABLE_HTTP) {
        modem.disableGPS();
        DBG("POSTing data to endpoint ... ");
        String contentType = "application/json";
  
        String postData = "{"
          "\"version\":1.1,"
          "\"sensorType\":\"Wovyn SF-Pico-LTE\","
          "\"emitterGUID\":\"" + imei + "\","
          "\"iccid\":\"" + ccid + "\","
          "\"secondsWithoutNetwork\":" + String(secondsWithoutNetwork / 1000) + ","
          "\"imsi\":\"" + imsi + "\","
          "\"operator\":\"" + cop + "\","
          "\"signal\":" + String(csq,8) + ","
          "\"ratType\":\"" + currentRatType + "\","
          "\"networkTime\":\"" + networkTime + "\","
          "\"lat\":" + String(gps_lat,8) + ","
          "\"lon\":" + String(gps_lon,8) + ","
          "\"speed\":" + String(gps_speed,3) + ","
          "\"alt\":" + String(gps_alt,3) + ","
          "\"vSat\":" + String(gps_vsat) + ","
          "\"uSat\":" + String(gps_usat) + ","
          "\"accuracy\":" + String(gps_accuracy,3) + ""
          "}";
      
        http.post(resource, contentType, postData);
      
        // read the status code and body of the response
        int statusCode = http.responseStatusCode();
        String response = http.responseBody();
      
        Serial.print("Status code: ");
        Serial.println(statusCode);
        if (statusCode < 0) {
          Serial.println(httpConnectError[abs(statusCode)]);
        }
        Serial.print("Response: ");
        Serial.println(response);
        DBG("... end POST!");
    
        // stop the http client
        http.stop();

        // reset variables after successful POST
        secondsWithoutNetwork = 0;

        modem.enableGPS();

      } // end http post to endpoint
#endif

#if ENABLE_MQTT
      // update mqtt ...
      if (POST_TO_ENDPOINT && ENABLE_MQTT) {
        DBG("Updating MQTT ... ");

        modem.disableGPS();

        if (!mqtt.connected()) {
          Serial.print("Attempting to connect to the MQTT broker: ");
          Serial.println(broker);
          
          if (!mqtt.connect(broker, mqttPort)) {
            Serial.print("MQTT connection failed! Error code = ");
            Serial.println(mqtt.connectError());
            Serial.println(mqttConnectError[mqtt.connectError()+2]);
            //mqtt.stop();
          
            return;
          }
        }
        
        Serial.println("Connected to the MQTT broker!");

        mqtt.poll();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/iccid");
        Serial.print("  Value: ");
        Serial.println(ccid);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/iccid");
        mqtt.print(ccid);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/imsi");
        Serial.print("  Value: ");
        Serial.println(imsi);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/imsi");
        mqtt.print(imsi);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/operator");
        Serial.print("  Value: ");
        Serial.println(cop);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/operator");
        mqtt.print(cop);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/ratType");
        Serial.print("  Value: ");
        Serial.println(currentRatType);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/ratType");
        mqtt.print(currentRatType);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/signal");
        Serial.print("  Value: ");
        Serial.println(String(csq,8));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/signal");
        mqtt.print(String(csq,8));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/networkTime");
        Serial.print("  Value: ");
        Serial.println(networkTime);

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/networkTime");
        mqtt.print(networkTime);
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/secondWithoutNetwork");
        Serial.print("  Value: ");
        Serial.println(String(secondsWithoutNetwork / 1000));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/secondsWithoutNetwork");
        mqtt.print(String(secondsWithoutNetwork / 1000));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/latitude");
        Serial.print("  Value: ");
        Serial.println(String(gps_lat,8));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/latitude");
        mqtt.print(String(gps_lat,8));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/longitude");
        Serial.print("  Value: ");
        Serial.println(String(gps_lon,8));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/longitude");
        mqtt.print(String(gps_lon,8));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/speed");
        Serial.print("  Value: ");
        Serial.println(String(gps_speed,3));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/speed");
        mqtt.print(String(gps_speed,3));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/altitude");
        Serial.print("  Value: ");
        Serial.println(String(gps_alt,3));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/altitude");
        mqtt.print(String(gps_alt,3));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/vSat");
        Serial.print("  Value: ");
        Serial.println(String(gps_vsat));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/vSat");
        mqtt.print(String(gps_vsat));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/uSat");
        Serial.print("  Value: ");
        Serial.println(String(gps_usat));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/uSat");
        mqtt.print(String(gps_usat));
        mqtt.endMessage();
        mqtt.flush();

        Serial.print("Updating MQTT topic: ");
        Serial.print("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/accuracy");
        Serial.print("  Value: ");
        Serial.println(String(gps_accuracy,3));

        // Update MQTT using print interface to set the message contents
        mqtt.beginMessage("Wovyn/SF-Pico-LTE/AssetTracker/" + imei + "/accuracy");
        mqtt.print(String(gps_accuracy,3));
        mqtt.endMessage();
        mqtt.flush();

        mqtt.stop();
        Serial.println("Stopped MQTT Connection");

        modem.enableGPS();

      } // end mqtt updates
#endif

      Serial.println("... end of sending to endpoints!");

      // reset the last transmit timestamp and messages variables
      lastTransmitTimestamp = millis();
      waitingStarted = 0;
      lastWaitingMessage = 0;

      Serial.println("<< Waiting " + String(TRANSMIT_INTERVAL) + " seconds until next send!");

    } // end if it's time to transmit

// at this time we do NOT want to disconnect the cellular connection
#if FALSE
    DBG("Disconnecting GPRS ...");
    modem.gprsDisconnect();
    if (!modem.isGprsConnected()) {
        DBG("GPRS disconnected");
    } else {
        DBG("GPRS disconnect: Failed.");
    }
#endif

// we also do NOT want to power down the modem
#if TINY_GSM_POWERDOWN
    // Try to power-off (modem may decide to restart automatically)
    // To turn off modem completely, please use Reset/Enable pins
    modem.poweroff();
    DBG("Poweroff modem.");
#endif

// ... and loop!
}
