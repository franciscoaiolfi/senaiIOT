#include <DHTesp.h>
#include <LiquidCrystal_I2C.h>
#include <ThingSpeak.h>
#include <WiFi.h>
#include <Adafruit_Sensor.h>
#include <PubSubClient.h>

#define DHT_PIN 15
#define I2C_ADDR 0x27
#define LCD_COLUMNS 16
#define LCD_LINES 2

DHTesp dhtSensor;
WiFiClient espClient;
PubSubClient mqttClient(espClient);
LiquidCrystal_I2C lcd(I2C_ADDR, LCD_COLUMNS, LCD_LINES);

const char* ssid = "Wokwi-GUEST";
const char* pass = "";

const char* mqttBroker = "broker.hivemq.com";
const int mqttPort = 1883;

unsigned long lastTime = 0;
unsigned long timerDelay = 30000;

unsigned long myChannelNumber = 2;
const char* server = "api.thingspeak.com";
const char* myWriteApiKey = "RF0KEO1UUU6Q5TT1";

float tempC;
float umidadeT;

void callbackMQTT(char* topic, byte* payload, unsigned int length) {
  String msg;
  for (int i = 0; i < length; i++) {
    char c = (char)payload[i];
    msg += c;
  }
  
  if (msg.equals("1")) {
    digitalWrite(13, HIGH);
  }
  if (msg.equals("0")) {
    digitalWrite(13, LOW);
  }
}

void reconnectWiFi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.println(".");
  }
  Serial.println("....");
  Serial.println("Wifi Conected");
  WiFi.mode(WIFI_STA);
}

void reconnectMQTT() {
  while (!mqttClient.connected()) {
    Serial.print("* Trying to connect to MQTT broker: ");
    Serial.println(mqttBroker);
    if (mqttClient.connect("esp32_mqtt73866")) {
      Serial.println("Connected successfully to MQTT broker!");
      mqttClient.subscribe("topic_on_off_led");
    }
    else {
      Serial.println("Failed to reconnect to the broker.");
      Serial.println("Retry in 2 seconds.");
      delay(2000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  dhtSensor.getPin();
  delay(10);
  lcd.init();
  lcd.backlight();
  pinMode(13, OUTPUT);
  
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(100);
    Serial.print(".");
  }
  Serial.println("Wifi Connected");
  WiFi.mode(WIFI_STA);
  
  ThingSpeak.begin(espClient);
  
  mqttClient.setServer(mqttBroker, mqttPort);
  mqttClient.setCallback(callbackMQTT);
}

void loop() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  tempC = dhtSensor.getTemperature();
  umidadeT = dhtSensor.getHumidity();
  
  if (data.temperature > 35) {
    digitalWrite(13, HIGH);
  }
  else {
    digitalWrite(13, LOW);
  }
  
  if (data.humidity > 70) {
    digitalWrite(14, HIGH);
  }
  else {
    digitalWrite(14, LOW);
  }
  
  Serial.println("Temperature: " + String(data.temperature, 1) + "C");
  Serial.println("Humidity: " + String(data.humidity, 1) + "%");
  Serial.println("------");
  
  lcd.setCursor(0, 0);
  lcd.print("Temperature: " + String(data.temperature, 1) + "\xDF" + "C  ");
  lcd.setCursor(0, 1);
  lcd.print("Humidity: " + String(data.humidity, 1) + "%  ");
  
  ThingSpeak.setField(1, tempC);
  ThingSpeak.setField(2, umidadeT);
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteApiKey);
  
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  
  mqttClient.loop();

  String message = "Temperatura: " + String(tempC) + " Umidade: " + String(umidadeT);
  mqttClient.publish("senai66", message.c_str());
  
  delay(1000);
}
