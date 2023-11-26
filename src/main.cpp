#include <Arduino.h>

#include <WiFi.h>
#include <PubSubClient.h>

// *********************************************************************************************************;
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <OneWire.h>
#include <DallasTemperature.h>

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
const char *mqtt_username = "root";
const char *mqtt_password = "orangepi.hass";
const int mqtt_port = 1883;

String client_id = "aquarium-";

// define led on pin 15
#define LED 15
// *********************************************************************************************************;

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char *topic, byte *payload, unsigned int length);
void reconnect();
void displayMQTTerror(int mqtterror);
void loop2(void *parameter);

// *********************************************************************************************************;
// tds sensor
#define TdsSensorPin A0
#define VREF 5.0  // analog reference voltage(Volt) of the ADC
#define SCOUNT 30 // sum of sample point

int analogBuffer[SCOUNT]; // store the analog value in the array, read from ADC
int analogBufferTemp[SCOUNT];
int analogBufferIndex = 0;
int copyIndex = 0;

float averageVoltage = 0;
float tdsValue = 0;
float temperature = 16; // current temperature for compensation

// median filtering algorithm
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
  {
    bTemp = bTab[(iFilterLen - 1) / 2];
  }
  else
  {
    bTemp = (bTab[iFilterLen / 2] + bTab[iFilterLen / 2 - 1]) / 2;
  }
  return bTemp;
}

// *********************************************************************************************************;
// DS18B20 sensor

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// *********************************************************************************************************;
void setup()
{
  // put your setup code here, to run once:
  // Set software serial baud to 115200;
  // define led
  pinMode(LED, OUTPUT);
  digitalWrite(LED, LOW);

  Serial.begin(9600);
  Serial.println("Iniciando...");
  // Connecting to a WiFi network
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  { // Address 0x3D for 128x64
    Serial.println(F("SSD1306 allocation failed"));
    for (;;)
      ;
  }
  // Start the tds sensor
  delay(2000);
  pinMode(TdsSensorPin, INPUT);

  // Start the DS18B20 sensor
  sensors.begin();

  // Start the OLED display
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("Aquarium");
  display.display();
  delay(2000);
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 10);
  display.println("V 0.0.1");
  display.display();
  delay(1000);
  // Start the OLED display
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Iniciando...");
  display.display();
  // WiFi
  // WiFi.mode(WIFI_STA); // Optional
  WiFi.begin(ssid, password);
  WiFi.setTxPower(WIFI_POWER_8_5dBm);
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

  display.display();

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
    delay(1000);
  }
}

void loop()
{
  sensors.requestTemperatures();
  float temperatureC = sensors.getTempCByIndex(0);
  float temperatureF = sensors.getTempFByIndex(0);
  Serial.print(temperatureC);
  Serial.println("ºC");
  Serial.print(temperatureF);
  Serial.println("ºF");
  delay(5000);

  static unsigned long analogSampleTimepoint = millis();
  if (millis() - analogSampleTimepoint > 40U)
  { // every 40 milliseconds,read the analog value from the ADC
    analogSampleTimepoint = millis();
    analogBuffer[analogBufferIndex] = analogRead(TdsSensorPin); // read the analog value and store into the buffer
    analogBufferIndex++;
    if (analogBufferIndex == SCOUNT)
    {
      analogBufferIndex = 0;
    }
  }

  static unsigned long printTimepoint = millis();
  if (millis() - printTimepoint > 800U)
  {
    printTimepoint = millis();
    for (copyIndex = 0; copyIndex < SCOUNT; copyIndex++)
    {
      analogBufferTemp[copyIndex] = analogBuffer[copyIndex];

      // read the analog value more stable by the median filtering algorithm, and convert to voltage value
      averageVoltage = getMedianNum(analogBufferTemp, SCOUNT) * (float)VREF / 1024.0;

      // temperature compensation formula: fFinalResult(25^C) = fFinalResult(current)/(1.0+0.02*(fTP-25.0));
      float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
      // temperature compensation
      float compensationVoltage = averageVoltage / compensationCoefficient;

      // convert voltage value to tds value
      tdsValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage - 255.86 * compensationVoltage * compensationVoltage + 857.39 * compensationVoltage) * 0.5;

      // Serial.print("voltage:");
      // Serial.print(averageVoltage,2);
      // Serial.print("V   ");
    }
    Serial.print("TDS Value:");
    Serial.print(tdsValue, 0);
    Serial.println("ppm");
  }
}

void reconnect()
{
  int contador_error = 0;
  Serial.println("Bucle Reconectar");
  // Loop hasta que estemos reconectados
  while (!client.connected())
  {
    Serial.println("Intentando conexión MQTT...");
    // Intentar conectar
    if (client.connect(client_id.c_str(), mqtt_username, mqtt_password))
    {
      Serial.println("conectado");
    }
    else
    {
      Serial.print("COmunicacion MQTT falló, rc=");
      Serial.println(client.state());
      Serial.println("Intentar de nuevo en 5 segundos");
      delay(500);

      // Esperar 5 segundos antes de volver a intentar
      displayMQTTerror(client.state());

      delay(5000);
      contador_error++;
      if (contador_error > 10)
      {
        Serial.println("Reiniciando ESP");

        ESP.restart();
      }
      Serial.println("Intento " + String(contador_error));
    }
  }
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