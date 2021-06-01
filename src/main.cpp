#include <WiFi.h>
#include <PubSubClient.h>
#include <Arduino.h>
#include <Wire.h>

#include "SparkFunCCS811.h" //Click here to get the library: http://librarymanager/All#SparkFun_CCS811
#include <FastLED.h>

#define NUM_LEDS 8
#define DATA_PIN 26
#define CCS811_ADDR 0x5A
// Replace the next variables with your SSID/Password combination
const char *ssid = "REPLACE_WITH_YOUR_SSID";
const char *password = "REPLACE_WITH_YOUR_PASSWORD";
const char *mqtt_server = "192.168.1.144";

const char *mqtt_server_user = "user";
const char *mqtt_server_password = "password";
const char *mqtt_topic = "esp32/1";

CRGBArray<NUM_LEDS> leds;

//#define CCS811_ADDR 0x5B //Default I2C Address
//Alternate I2C Address

CCS811 mySensor(CCS811_ADDR);

WiFiClient espClient;
PubSubClient client(espClient);
long lastMsg = 0;
char msg[50];
int value = 0;

void callback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print(". Message: ");
  String messageTemp;

  for (int i = 0; i < length; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  // Feel free to add more if statements to control more GPIOs with MQTT
}

void setup()
{
  Serial.begin(115200);
  Serial.println("CCS811 Basic Example");

  Wire.begin(21, 22); //Inialize I2C Hardware

  if (mySensor.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing...");
    while (1)
      ;
  }
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);

  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void reconnect()
{
  // Loop until we're reconnected
  while (!client.connected())
  {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP32Client", mqtt_server_user, mqtt_server_password))
    {
      Serial.println("connected");
      // Subscribe
      client.subscribe("esp32/output");
    }
    else
    {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void loop()
{
  if (!client.connected())
  {
    reconnect();
  }
  client.loop();

  //Check to see if data is ready with .dataAvailable()
  if (mySensor.dataAvailable())
  {
    //If so, have the sensor read and calculate the results.
    //Get them later
    mySensor.readAlgorithmResults();

    Serial.print("CO2[");
    //Returns calculated CO2 reading
    uint16_t co2 = mySensor.getCO2();
    Serial.print(co2);
    Serial.print("] tVOC[");
    //Returns calculated TVOC reading
    Serial.print(mySensor.getTVOC());
    Serial.print("] millis[");
    //Display the time since program start
    Serial.print(millis());
    Serial.print("]");
    Serial.println();
    CRGB col = co2 < 600 ? CRGB::Green : co2 < 1000 ? CRGB::Yellow
                                                    : CRGB::Red;
    for (int i = 0; i < NUM_LEDS; i++)
    {
      leds[i] = col;
    }
    FastLED.show();
    char co2String[8];
    dtostrf(co2, 1, 2, co2String);
    client.publish("esp32/humidity", co2String);
  }

  delay(500); //Don't spam the I2C bus
}