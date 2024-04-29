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
#define ENABLE_HTTP false
#define USE_HTTPS   false
const char server[] = "myHttpHost.mydomain.com";
const char resource[] = "/mypath";
const int  port = 0;

// Your MQTT(S) broker details
#define ENABLE_MQTT false
#define USE_MQTTS   false
const char broker[] = "test.mosquitto.org";
const int  mqttPort = 1883;
