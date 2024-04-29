# sixfab-pico-lte-asset-tracker-example
This is a simple example for using the Sixfab Pico LTE as an asset tracker.

The sketch is using the following libraries that will have to be installed:

- TinyGSM for Cellular Modem support - https://github.com/vshymanskyy/TinyGSM
- ArduinoMQTTClient for MQTT Broker support - https://github.com/arduino-libraries/ArduinoMqttClient
- ArduinoHTTPClient for HTTP support - https://github.com/arduino-libraries/ArduinoHttpClient

You can search for these libraries in the Arduino IDE Library Manager.

To use this code you also need to set several parameters in the ```configuration.h``` file:

```
/*
 * do we want to post to our defined endpoints?
 */
#define POST_TO_ENDPOINT  true

/*
 * Your endpoint configuration(s)
 *   - You must set the defined variables for your endpoints
 *   - Enable HTTP and/or MQTT
 *   - Indicate if you want SSL/TLS used for either
 */
// Your HTTP(S) server details
#define ENABLE_HTTP true
#define USE_HTTPS   false
const char server[] = "myHttpHost.mydomain.com";
const char resource[] = "/mypath";
const int  port = 0;

// Your MQTT(S) broker details
#define ENABLE_MQTT true
#define USE_MQTTS   false
const char broker[] = "test.mosquitto.org";
const int  mqttPort = 1883;

```
There are several other parameters defined in the main code that can easily be tweaked and adjusted for your application.
