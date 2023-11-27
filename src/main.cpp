#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>

// *********************************************************************************************************;
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADS1X15.h>

Adafruit_ADS1115 ads; /* Use this for the 16-bit version */

#define TdsSensorPin 2
#define VREF 3.3          // analog reference voltage(Volt) of the ADC
#define SCOUNT 30         // sum of sample point
int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0, copyIndex = 0;
float averageVoltage = 0, tdsValue = 0, temperature = 25;

// *********************************************************************************************************;
int getMedianNum(int bArray[], int iFilterLen);
void loop2(void *parameter);
bool checkBound(float newValue, float prevValue, float maxDiff);

//*********************************************** OLED ***********************************************************
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

//*********************************************** Wifi and MQTT ***********************************************************
// WiFi
// const char *ssid = "DIGIFIBRA-cF5T";
// const char *password = "P92sKt3FGfsy";

const char *ssid = "Armadillo";
const char *password = "armadillolan";

// MQTT Broker
const char *mqtt_broker = "192.168.1.43";
const char *topic = "casa/aquarium";
const char *topicLevel = "casa/aquarium/level";
const char *topicTemp = "casa/aquarium/temp";
const char *topicTds = "casa/aquarium/tds";
const char *topicHum = "casa/aquarium/hum";

const char *mqtt_username = "root";
const char *mqtt_password = "orangepi.hass";
const int mqtt_port = 1883;

String client_id = "aquarium-";

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void displayMQTTerror(int mqtterror);

//* *********************************************** Sensors ***********************************************************

#define CapacitiveLEVELPIN 3

// define led on pin 15
#define LED 15

#define DHTPIN 1
#define DHTTYPE DHT11
// Inicializamos el sensor DHT11
DHT dht(DHTPIN, DHTTYPE);

// *********************************************************************************************************;
// DS18B20 sensor
// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;
// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);
// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// int adc_vref_to_gpio(11);
//  variables

float temperatureC = 0;
float temperatureF = 0;
float humidity = 0;
float heatIndexC = 0;
float heatIndexF = 0;
float heatIndex = 0;

int correction = 0;

bool stateScreen = 0;
float tdsValueCorrected = 0;
int level = 0;

float h = 0;
// Leemos la temperatura en grados centígrados (por defecto)
float t = 0;
// create a variable to use checkBound function
float prevTemp = 0;
float prevHum = 0;
float prevHeatIndex = 0;
float prevTds = 0;
float prevLevel = 0;
float prevTempBox = 0;
float prevHumBox = 0;

void setup()
{
  Serial.begin(115200);
  pinMode(TdsSensorPin, INPUT);
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);
  pinMode(CapacitiveLEVELPIN, INPUT);

  // ADS
  //  Descomentar el que interese
  //  ads.setGain(GAIN_TWOTHIRDS);  +/- 6.144V  1 bit = 0.1875mV (default)
  //  ads.setGain(GAIN_ONE);        +/- 4.096V  1 bit = 0.125mV
  //  ads.setGain(GAIN_TWO);        +/- 2.048V  1 bit = 0.0625mV
  //  ads.setGain(GAIN_FOUR);       +/- 1.024V  1 bit = 0.03125mV
  //  ads.setGain(GAIN_EIGHT);      +/- 0.512V  1 bit = 0.015625mV
  //  ads.setGain(GAIN_SIXTEEN);    +/- 0.256V  1 bit = 0.0078125mV
  ads.begin();
  // Comenzamos el sensor DHT
  dht.begin();
  Serial.println("Iniciando...");
  // Connecting to a WiFi network
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  // Start the DS18B20 sensor
  sensors.begin();

  // Start the OLED display
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 10);
  display.println("Aquarium");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(15, 10);
  display.println("V0.0.1");
  display.display();
  delay(1000);
  // Start the OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();
  // // WiFi
  // // WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  // WiFi.setTxPower(WIFI_POWER_8_5dBm);
  Serial.println("\nConnecting");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    digitalWrite(LED, HIGH);
    delay(500);
    Serial.println("Connecting to WiFi..");
    // show in display oled
    digitalWrite(LED, LOW);
    display.setCursor(0, 20);
    display.println("Connecting to WiFi..");
    display.display();
  }

  Serial.println("Connected to the Wi-Fi network");
  // print the local IP address to serial monitor
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
  // print mac address
  Serial.print("MAC address: ");
  Serial.println(WiFi.macAddress());
  delay(5000);

  client.setKeepAlive(20860);
  client.setServer(mqtt_broker, mqtt_port);
  client.setCallback(callback);

  client_id = client_id + String(WiFi.macAddress());

  while (!client.connected())
  {
    Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
    Serial.println();
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("Conectado a Casa MQTT Broker!");
    }
    else
    {
      Serial.print("failed with state ");
      Serial.println(client.state());
      Serial.println("Trying again in 5 seconds");
    }
    delay(2000);
  }

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi: " + WiFi.localIP().toString());
  display.setCursor(0, 20);
  display.println("MQTT: " + String(mqtt_broker));
  display.display();
  // Publish and subscribe
  client.publish(topic, "Hi, I'm AQUARIUM ^^");
  delay(3000);
  stateScreen = HIGH;

  xTaskCreatePinnedToCore(
      loop2,  /* Function to implement the task */
      "loop", /* Name of the task */
      1000,   /* Stack size in words */
      NULL,   /* Task input parameter */
      0,      /* Priority of the task */
      NULL,   /* Task handle. */
      1);
}
void loop2(void *parameter)
{
  while (true)
  {
    client.loop();
    if (stateScreen == HIGH)
    {

      display.clearDisplay();
      display.setTextSize(1);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("TDS: " + String((tdsValue / 100), 0) + " ppm"); // 1015 correction
      display.setCursor(0, 10);
      display.println("Temp: " + String(temperatureC) + " C");
      display.setCursor(0, 20);
      display.println("Level: " + String(level));
      display.display();
      delay(1000);
    }
  }
}
void loop()
{
  int16_t adc0;
  adc0 = ads.readADC_SingleEnded(0);

  static unsigned long analogSampleTimepoint = millis();

  if (millis() - analogSampleTimepoint > 40U) // every 40 milliseconds,read the analog value from the ADC
  {
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = adc0; // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
      analogBufferIndex = 0;
  }
  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];
    averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;                                                                                                                   // read the analog value more stable by the median filtering algorithm, and convert to voltage value
    float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);                                                                                                                                // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
    float compensationVolatge = averageVoltage / compensationCoefficient;                                                                                                                             // temperature compensation
    tdsValue = (((133.42 * compensationVolatge * compensationVolatge * compensationVolatge - 255.86 * compensationVolatge * compensationVolatge + 857.39 * compensationVolatge) * 0.5) - correction); // convert voltage value to tds value
                                                                                                                                                                                                      // Serial.print("voltage:");
                                                                                                                                                                                                      // Serial.print(averageVoltage,2);
                                                                                                                                                                                                      // Serial.print("V ");
    // if (tdsValue < 0)
    // {
    //   tdsValue = 0;
    // }

    Serial.println("***********************************");
    Serial.print("TDS Value:");
    Serial.print(tdsValue / 100, 0);
    Serial.println("ppm");
  }

  static unsigned long sondaTimepoint = millis();
  if (millis() - sondaTimepoint > 1000)
  {
    sondaTimepoint = millis();
    // Leemos la humedad relativa
    h = dht.readHumidity();
    // Leemos la temperatura en grados centígrados (por defecto)
    t = dht.readTemperature();
    // Leemos la temperatura en grados Fahrenheit
    float f = dht.readTemperature(true);

    // Comprobamos si ha habido algún error en la lectura
    if (isnan(h) || isnan(t) || isnan(f))
    {
      Serial.println("Error obteniendo los datos del sensor DHT11");
      return;
    }

    sensors.requestTemperatures();
    temperatureC = sensors.getTempCByIndex(0);
    temperatureF = sensors.getTempFByIndex(0);

    Serial.print("Temperatura del Acuario: ");
    Serial.print(temperatureC);
    Serial.println("ºC");
    // Calcular el índice de calor en Fahrenheit
    float hif = dht.computeHeatIndex(f, h);
    // Calcular el índice de calor en grados centígrados
    float hic = dht.computeHeatIndex(t, h, false);

    Serial.print("Temperatura de la Caja: ");
    Serial.print(t);
    Serial.println(" ºC ");
    Serial.print("Humedad de la Caja: ");
    Serial.print(h);
    Serial.println(" %");
    Serial.print("Indice de Calor: ");
    Serial.print(hic);
    Serial.println(" ºC ");
    Serial.println("***********************************");

    // read capacitive level
    level = digitalRead(CapacitiveLEVELPIN);
    Serial.print("Capacitive Level: ");
    Serial.println(level);
  }

  static unsigned long dataPublishSampleTimepoint = millis();
  if (millis() - dataPublishSampleTimepoint > 5000)
  {
    dataPublishSampleTimepoint = millis();

    // Using checkBound function to check the data
    if (checkBound(temperatureC, prevTemp, 0.5))
    {
      prevTemp = temperatureC;
      client.publish(topicTemp, String(temperatureC).c_str());
    }
    if (checkBound(tdsValue, prevTds, 1))
    {
      prevTds = tdsValue;
      client.publish(topicTds, String((int)(tdsValue / 100)).c_str());
    }
    if (checkBound(level, prevLevel, 0.1))
    {
      prevLevel = level;
      client.publish(topicLevel, String(level).c_str());
    }
    // send box temperature and humidity
    if (checkBound(t, prevTempBox, 0.1))
    {
      if (t < 40)
      {
        prevTempBox = t;
        client.publish("casa/aquarium/box/temp", String((int)(t)).c_str());
      }
    }
    if (checkBound(h, prevHumBox, 0.1))
    {
      if (h < 100 && h > 0)
      {
        prevHumBox = h;
        client.publish("casa/aquarium/box/hum", String((int)(h)).c_str());
      }
    }
  }
}
int getMedianNum(int bArray[], int iFilterLen)
{
  int bTab[iFilterLen];
  for (byte i = 0; i < iFilterLen; i++)
    bTab[i] = bArray[i];
  int i, j, bTemp;
  for (j = 0; j < iFilterLen - 1; j++)
  {
    for (i = 0; i < iFilterLen - j - 1; i++)
    {
      if (bTab[i] > bTab[i + 1])
      {
        bTemp = bTab[i];
        bTab[i] = bTab[i + 1];
        bTab[i + 1] = bTemp;
      }
    }
  }
  if ((iFilterLen & 1) > 0)
    bTemp = bTab[(iFilterLen - 1) / 2];
  else
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  return bTemp;
}
bool checkBound(float newValue, float prevValue, float maxDiff)
{
  return !isnan(newValue) && (newValue < prevValue - maxDiff || newValue > prevValue + maxDiff);
}
void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived in topic: ");
  Serial.println(topic);
  Serial.print("Message:");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  Serial.println("-----------------------");
}
void displayMQTTerror(int mqtterror)
{
  display.clearDisplay();
  display.setCursor(0, 5);
  display.println("MQTT: ");
  display.setCursor(0, 20);
  display.println("Error: " + String(mqtterror));
  display.setCursor(0, 30);
  display.println("Reintentando...");
  display.display();
  delay(2000);
}