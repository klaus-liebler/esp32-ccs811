#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <Arduino.h>
#include <Wire.h>
#include <AudioFileSourcePROGMEM.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>
#include "Alarm.mp3.h"

#include <SparkFunCCS811.h>
#include "SparkFunBME280.h"
#include <FastLED.h>
#include "secrets.hh"

constexpr int32_t NUM_LEDS = 8;
constexpr uint8_t DATA_PIN = 26;
constexpr uint8_t CCS811_ADDR = 0x5A; //or 0x5B

//Managementobjekt für die RGB-LEDs
CRGBArray<NUM_LEDS> leds;

//Managementobjekt für den CO2-Sensor
CCS811 ccs811(CCS811_ADDR);

//Managementobjekt für den Temperatur/Luftdruck/Luftfeuchtigkeitssensor
BME280 bme280;

//Managementobjekte für die Sound-Wiedergabe
AudioGeneratorMP3 *gen;
AudioFileSourcePROGMEM *file;
AudioOutputI2S *out;

//Kommunikationsobjekt für WLAN
WiFiClient wifiClient;

//Kommunikationsobjekt für MQTT; nutzt WLAN
PubSubClient mqttClient(wifiClient);

//Kommunikationsobjekt Webserver
WebServer httpServer(80);

//"Datenmodell" durch einfache globale Variablen
float temperature, humidity, pressure, co2;

char jsonBuffer[300];

enum class SoundState
{
  IDLE,
  REQUEST_TO_PLAY,
  PLAYING,
  FINISHED,
};

SoundState soundState = SoundState::IDLE;

long lastMsg = 0;
char msg[50];
int value = 0;

//Funktion erzeugt dynamisches HTML - letztlich die Website, die darzustellen ist
void handle_OnConnect()
{
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr += "<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr += "<title>Barrierefreie Behaglichskeitsampel</title>\n";
  ptr += "<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr += "body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;}\n";
  ptr += "p {font-size: 24px;color: #444444;margin-bottom: 10px;}\n";
  ptr += "</style>\n";
  ptr += "</head>\n";
  ptr += "<body>\n";
  ptr += "<div id=\"webpage\">\n";
  ptr += "<h1>Barrierefreie Behaglichskeitsampel</h1>\n";
  ptr += "<p>Temperatur: ";
  ptr += temperature; //<<<<<================Hier wird zum Beispiel die aktuelle Temperatur dynamische ins HTML eingefügt
  ptr += "&deg;C</p>";
  ptr += "<p>Luftfreuchtigkeit: ";
  ptr += humidity;
  ptr += "%</p>";
  ptr += "<p>Luftdruck: ";
  ptr += pressure;
  ptr += "hPa</p>";
  ptr += "<p>Luftfreuchtigkeit: ";
  ptr += humidity;
  ptr += "%</p>";
  ptr += "<p>CO2-Konzentration: ";
  ptr += co2;
  ptr += "ppm</p>";
  ptr += "</div>\n";
  ptr += "</body>\n";
  ptr += "</html>\n";
  httpServer.send(200, "text/html", ptr);
}

//Generiert eine einfache Fehlerseite
void handle_NotFound()
{
  httpServer.send(404, "text/plain", "Not found");
}

//Funktion wird immer dann aufgerufen, wenn lab@home eine Nachricht über MQTT erhält - wird hier eigentlich nicht benötigt und ist nur "der Vollständigkeit halber" implementiert
void mqttCallback(char *topic, byte *message, unsigned int length)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.println();
}

//Funktion wird automatisch vom Framework einmalig beim einschalten bzw nach Reset aufgerufen
void setup()
{
  //Richtet serielle Kommunikationsschnittstelle ein, damit die ganzen Meldungen am PC angezeigt werden können
  Serial.begin(115200);
  Serial.println("W-HS IoT Barrierefreie Behaglichskeitsampel");

  //Legt fest, über welche Schnittstelle und welche Pins des ESP32 die Sound-Wiedergabe laufen soll
  out = new AudioOutputI2S(0, 0);
  out->SetPinout(27, 4, 25);

  //Legt fest, über welche Pins die sog. I2C-Schnittstelle zur Anbindung der beiden verwendete Sensoren laufen soll
  Wire.begin(21, 22);

  //Legt die Bus-Adresse des BME280-Sensors fest
  bme280.setI2CAddress(0x76);

  //Baut die Verbindung mit dem CCS811 auf
  if (ccs811.begin() == false)
  {
    Serial.print("CCS811 error. Please check wiring. Freezing...");
    while (1)
      ;
  }

  //Baut die Verbindung mit dem BME280 auf
  if (bme280.beginI2C() == false)
  {
    Serial.print("BME280 error. Please check wiring. Freezing...");
    while (1)
      ;
  }

  //Konfiguriert die RGB-Leds
  FastLED.addLeds<WS2812B, DATA_PIN, RGB>(leds, NUM_LEDS);

  //Baut die Verbindung mit dem WLAN auf
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(WIFI_SSID);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");

  //Es wird hinterlegt, welche Funktionen aufzurufen sind, wenn ein Browser sich verbindet
  //Wenn die "Hauptseite", also einfach "/" aufgerufen wird, dann soll handle_OnConnect aufgerufen werden
  httpServer.on("/", handle_OnConnect);
  //wenn etwas anderes aufgerufen wird, dann soll eine einfache Fehlerseite dargestellt werden
  httpServer.onNotFound(handle_NotFound);

  //Ab der nächsten Zeile ist der ESP32 für einen Webbrowser erreichbar, weil der sog "HTTP-Server" gestartet wird
  httpServer.begin();
  Serial.print("HTTP server started. Open http://");
  Serial.print(WiFi.localIP());
  Serial.println(" in your browser");

  //Baut die Verbindung zum MQTT-Server auf
  mqttClient.setServer(MQTT_SERVER, 1883);
  mqttClient.setCallback(mqttCallback);
  if (!mqttClient.connect("lab@home", MQTT_USER, MQTT_PASS))
  {
    Serial.print("MQTT connection failed. Freezing...");
    while (1)
      ;
  }
}

//Management-Routine für die Sound-Wiedergabe
void soundLoop()
{
  switch (soundState)
  {
  case SoundState::REQUEST_TO_PLAY:
    file = new AudioFileSourcePROGMEM(Alarm_mp3, sizeof(Alarm_mp3));
    gen = new AudioGeneratorMP3();
    gen->begin(file, out);
    soundState = SoundState::PLAYING;
    //break;
  case SoundState::PLAYING:
    if (gen->isRunning() && !gen->loop())
    {
      gen->stop();
      file->close();
      delete gen;
      delete file;
      soundState = SoundState::FINISHED;
    }
    break;
  default:
    break;
  }
}

uint32_t lastSensorUpdate = 0;
uint32_t lastMQTTUpdate = 0;

//Diese Funktion wird vom Framework immer und immer wieder aufgerufen
void loop()
{
  //Sorge dafür, dass der mqttClient, der httpClient und die Sound-Wiedergabe ihren Aktivitäten nachgehen können
  mqttClient.loop();
  httpServer.handleClient();
  soundLoop();
  //Hole die aktuelle Zeit
  int now = millis();

  //Hole alle 5 Sekunden Messdaten von den Sensoren
  if (now - lastSensorUpdate > 5000)
  {
    //Check to see if data is ready with .dataAvailable()
    if (ccs811.dataAvailable())
    {
      ccs811.readAlgorithmResults();
      co2 = ccs811.getCO2();
    }
    humidity = bme280.readFloatHumidity();
    temperature = bme280.readTempC();
    pressure = bme280.readFloatPressure();
    //...und gebe die aktuellen Messdaten auf der Console aus
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"temperature\":\"%f\",\"humidity\":\"%f\", \"pressure\":\"%f\", \"co2\":\"%f\"}", temperature, humidity, pressure, co2);
    Serial.println(jsonBuffer);

    //...und führe die Lampen-Sound-Logik aus

    //Errechne ausgehend vom CO2-Messwert die anzuzeigende Farbe
    CRGB col = CRGB::Green;
    if (co2 > 1000)
      col = CRGB::Red;
    if (co2 > 600)
      col = CRGB::Yellow;
    //Setze alle acht Lampen auf den Farbwert
    for (int i = 0; i < NUM_LEDS; i++)
      leds[i] = col;
    //und schreibe diesen Farbwert in die LEDs raus
    FastLED.show();

    //gebe außerdem der Sound-Wiedergabe vor, was Sie zu machen hat (Sound Starten bzw zurücksetzen)
    if (co2 > 1000)
    {
      if (soundState == SoundState::IDLE)
        soundState = SoundState::REQUEST_TO_PLAY;
    }
    else
    {
      if (soundState == SoundState::FINISHED)
        soundState = SoundState::IDLE;
    }

    lastSensorUpdate = now;
  }

  //Schreibe alle 20 Sekunden die aktuellen Messwerte per MQTT raus
  if (now - lastMQTTUpdate > 20000)
  {
    mqttClient.publish("esp32/humidity", jsonBuffer);
    lastMQTTUpdate = now;
  }
}