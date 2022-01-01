#include <WiFi.h>
#include <PubSubClient.h>
#include <WebServer.h>
#include <Arduino.h>
#include <Wire.h>
#include <AudioFileSourcePROGMEM.h>
#include <AudioGeneratorMP3.h>
#include <AudioOutputI2S.h>
#include "Alarm.mp3.h"
#include <HTTPClient.h>

#include <SparkFunCCS811.h>
#include <SparkFunBME280.h>
#include "WS2812.hh"
#include "owb.h"
#include "owb_rmt.h"
#include "ds18b20.h"
#include "secrets.hh"


//Diverse konstante Werte werden gesetzt
constexpr gpio_num_t PIN_LED_STRIP = GPIO_NUM_26;
constexpr gpio_num_t PIN_ONEWIRE = GPIO_NUM_14;

constexpr size_t LED_NUMBER = 8;
constexpr uint32_t TIMEOUT_FOR_LED_STRIP = 1000;
constexpr rmt_channel_t CHANNEL_WS2812 = RMT_CHANNEL_0;
constexpr rmt_channel_t CHANNEL_ONEWIRE_TX = RMT_CHANNEL_1;
constexpr rmt_channel_t CHANNEL_ONEWIRE_RX = RMT_CHANNEL_2;

constexpr uint8_t CCS811_ADDR = 0x5A; //or 0x5B
constexpr uint8_t BME280_ADDR = 0x76;

//Managementobjekt für die RGB-LEDs
WS2812_Strip<LED_NUMBER> *strip = NULL;

//Managementobjekt für den CO2-Sensor
CCS811 ccs811(CCS811_ADDR);
bool css811_ok = false;

//Managementobjekt für den Temperatur/Luftdruck/Luftfeuchtigkeitssensor
BME280 bme280;
bool bme280_ok = false;

//Managementobjekte für den präzisen Temperatursensor, der an Pin 14 (GPIO_NUM_14) hängt
DS18B20_Info *ds18b20_info = NULL;
owb_rmt_driver_info rmt_driver_info;
OneWireBus *owb;

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

#include "index.html"

void handle_httpGetRoot()
{
  httpServer.send(200, "text/html", index_html);
}

void handle_httpGetData()
{
  httpServer.send(200, "application/json", jsonBuffer);
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

// Called when a metadata event occurs (i.e. an ID3 tag, an ICY block, etc.
void MDCallback(void *cbData, const char *type, bool isUnicode, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  (void)isUnicode; // Punt this ball for now
  // Note that the type and string may be in PROGMEM, so copy them to RAM for printf
  char s1[32], s2[64];
  strncpy_P(s1, type, sizeof(s1));
  s1[sizeof(s1) - 1] = 0;
  strncpy_P(s2, string, sizeof(s2));
  s2[sizeof(s2) - 1] = 0;
  Serial.printf("METADATA(%s) '%s' = '%s'\n", ptr, s1, s2);
  Serial.flush();
}

// Called when there's a warning or error (like a buffer underflow or decode hiccup)
void StatusCallback(void *cbData, int code, const char *string)
{
  const char *ptr = reinterpret_cast<const char *>(cbData);
  // Note that the string may be in PROGMEM, so copy it to RAM for printf
  char s1[64];
  strncpy_P(s1, string, sizeof(s1));
  s1[sizeof(s1) - 1] = 0;
  Serial.printf("STATUS(%s) '%d' = '%s'\n", ptr, code, s1);
  Serial.flush();
}

//Funktion wird automatisch vom Framework einmalig beim einschalten bzw nach Reset aufgerufen
void setup()
{
  //Richtet serielle Kommunikationsschnittstelle ein, damit die ganzen Meldungen am PC angezeigt werden können
  Serial.begin(115200);
  Serial.println("W-HS IoT BeHampel");

  //Legt fest, über welche Schnittstelle und welche Pins des ESP32 die Sound-Wiedergabe laufen soll
#ifdef EXTERNAL_I2S_DAC
  out = new AudioOutputI2S(0, 0);
  out->SetPinout(27, 4, 25);
#else
  out = new AudioOutputI2S(0, 1);
#endif

  //Legt fest, über welche Pins die sog. I2C-Schnittstelle zur Anbindung der beiden verwendete Sensoren laufen soll
  Wire.begin(22, 21);

  //Legt die Bus-Adresse des BME280-Sensors fest
  bme280.setI2CAddress(BME280_ADDR);

  //Baut die Verbindung mit dem CCS811 auf
  css811_ok = ccs811.begin();
  if (css811_ok)
  {
    Serial.println("CCS811 found and initialized.");
  }
  else
  {
    Serial.println("CCS811 error. Please check wiring!");
  }

  //Baut die Verbindung mit dem BME280 auf
  bme280_ok = bme280.beginI2C();
  if (bme280_ok)
  {
    Serial.println("BME280 found and initialized.");
  }
  else
  {
    Serial.println("BME280 error. Please check wiring!");
  }

  //Starte die OneWire-Bibliothek
  Serial.println("Start DS18B20:");
  owb=owb_rmt_initialize(&rmt_driver_info, PIN_ONEWIRE, CHANNEL_ONEWIRE_TX, CHANNEL_ONEWIRE_RX);
  owb_use_crc(owb, true); // enable CRC check for ROM code
  OneWireBus_ROMCode rom_code;
  owb_status status = owb_read_rom(owb, &rom_code);
  if (status == OWB_STATUS_OK)
  {
      ds18b20_info = ds18b20_malloc();                                 // heap allocation
      ds18b20_init_solo(ds18b20_info, owb);                            // only one device on bus
      ds18b20_use_crc(ds18b20_info, true);                             // enable CRC check on all reads
      ds18b20_set_resolution(ds18b20_info, DS18B20_RESOLUTION_10_BIT); //10bit -->187ms Conversion time
      ds18b20_convert_all(owb);
  }
  else
  {
    Serial.printf("An error occurred reading DS18B20 ROM code: %d", status);
  }
  Serial.println("DS18B20 started successfully");
  
  //Konfiguriert den 8fach-RGB-LED-Strip
  strip = new WS2812_Strip<LED_NUMBER>(CHANNEL_WS2812);
  ESP_ERROR_CHECK(strip->Init(PIN_LED_STRIP));
  ESP_ERROR_CHECK(strip->Clear(TIMEOUT_FOR_LED_STRIP));
  strip->SetPixel(0,CRGB::Red);
  strip->SetPixel(1,CRGB::Green);
  strip->SetPixel(2,CRGB::Yellow);
  strip->Refresh(TIMEOUT_FOR_LED_STRIP);

  soundState = SoundState::REQUEST_TO_PLAY;

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
  httpServer.on("/", handle_httpGetRoot);
  //Die anzuzeigenden Daten werden nach dem erstmaligen Laden sekündlich aktualisiert. Dabei nutzt der Browser eine Technik namens "AJAX"
  httpServer.on("/data", handle_httpGetData);
  //wenn etwas anderes aufgerufen wird, dann soll eine einfache Fehlerseite dargestellt werden
  httpServer.onNotFound(handle_NotFound);

  //Ab der nächsten Zeile ist der ESP32 für einen Webbrowser erreichbar, weil der sog "HTTP-Server" gestartet wird
  httpServer.begin();
  Serial.print("HTTP server started. Open http://");
  Serial.print(WiFi.localIP());
  Serial.println(" in your browser");

  //Baut die Verbindung zum MQTT-Server auf
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
  if (!mqttClient.connect("BeHampel-lab@home", MQTT_USER, MQTT_PASS))
  {
    Serial.print("MQTT connection failed. Freezing...");
    while (1)
      ;
  }
  soundState = SoundState::REQUEST_TO_PLAY;
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
    if (css811_ok && ccs811.dataAvailable())
    {
      ccs811.readAlgorithmResults();
      co2 = ccs811.getCO2();
    }
    humidity = bme280_ok ? bme280.readFloatHumidity() : 0.0;
    pressure = bme280_ok ? bme280.readFloatPressure() : 0.0;
    pressure = pressure / 100;

    //Hole die Temperatur vom präzisen OneWire-Sensor und starte direkt den nächsten Messzyklus
    ds18b20_read_temp(ds18b20_info, &(temperature));
    ds18b20_convert_all(owb);

    const char *state = co2 < 800.0 ? "Gut" : "Schlecht";
    //...und gebe die aktuellen Messdaten auf der Console aus
    snprintf(jsonBuffer, sizeof(jsonBuffer), "{\"state\":\"%s\",\"temperature\":%.1f,\"humidity\":%.0f, \"pressure\":%.0f, \"co2\":%.0f}", state, temperature, humidity, pressure, co2);
    Serial.println(jsonBuffer);

    //...und führe die Lampen-Sound-Logik aus

    //Errechne ausgehend vom CO2-Messwert die anzuzeigende Farbe (RGB-Darstellung), https://www.rapidtables.com/web/color/RGB_Color.html
    //#####################
    //BITTE AB HIER VERÄNDERN
    //#####################

    ESP_ERROR_CHECK(strip->Clear(TIMEOUT_FOR_LED_STRIP));
    strip->SetPixel(0, CRGB::Blue);
    strip->SetPixel(1, CRGB::Blue);
    strip->SetPixel(2, CRGB::Blue);
    ESP_ERROR_CHECK(strip->Refresh(TIMEOUT_FOR_LED_STRIP));

    //#####################
    //BITTE AB HIER NICHTS MEHR VERÄNDERN
    //#####################

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
    if (mqttClient.publish(MQTT_TOPIC, jsonBuffer))
    {
      Serial.println("Successfully published to MQTT Broker");
    }
    else
    {
      Serial.println("Error while publishing to MQTT Broker");
    }
    lastMQTTUpdate = now;
  }
}