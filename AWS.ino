#include <Arduino.h>
#include <UbidotsEsp32Mqtt.h>

//Weather Station
#define WEATHERSTATION_ADDRESS 0x08
#define WEATHERSTATION_READ 0x03

#define WEATHERSTATION_TIMEOUT 1000

#define WEATHERSTATION_DATA_HUMIDITY 0
#define WEATHERSTATION_DATA_TEMPERATURE 2
#define WEATHERSTATION_DATA_NOISE 4
#define WEATHERSTATION_DATA_ATMPRESS 10
#define WEATHERSTATION_DATA_LUX 12
#define WEATHERSTATION_DATA_LIGHT 16

#define WEATHERSTATION_DATA_LENGTH 18

float humidity, temperature, noise, atmPressure, lux, light;
float winddir = 0.0, windspeed = 0.0;

//Tipping Bucket Rainfall
#define RAINFALL_PIN 32
#define RAINFALL_TIPPING_VALUE 0.2

float rainfall = 0.0;

//0x08 0x03 0x01 0xF4 0x00 0x0D

//Output
#define OUTPUT_MILLIS 1000
uint32_t output_timer;

//Connectivity
#define WIFI_SSID "YOUR SSID"
#define WIFI_PASS "YOUR PASSWORD"

//Ubidots
#define PUBLISH_MILLIS 5000
const char *UBIDOTS_TOKEN = "YOUR UBIDOTS TOKEN";
const char *DEVICE_LABEL = "mappi32-weatherstation";
const char *WINDSPEED_LABEL = "windspeed";
const char *WINDDIR_LABEL = "winddir";
const char *HUM_LABEL = "humidity";
const char *TEMP_LABEL = "temperature";
const char *ATMPRES_LABEL = "atmpress";
const char *NOISE_LABEL = "noise";
const char *LUX_LABEL = "lux";
const char *RAINFALL_LABEL = "rainfall";
unsigned long ubidots_timer = 0;

char buffer[256] = {0};

#define RXD2 16
#define TXD2 17

Ubidots ubidots(UBIDOTS_TOKEN);

void IRAM_ATTR rainfall_it_handler();

void ubidots_callback(char *topic, byte *payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for(int i = 0; i < length; i++) {
    Serial.write((char)payload[i]);
  }

  Serial.println();

  return;
}

uint8_t WeatherStation_Read(char* output, uint16_t timeout) {
  const char send[] = {WEATHERSTATION_ADDRESS, WEATHERSTATION_READ, 0x01, 0xF8, 0x00, 0x09, 0x05, 0x58};
  char *pTemp = output;

  uint8_t dataLength = 0;
  uint8_t dataCount = 0;

  Serial2.write(send, 8);

  uint32_t previousMillis = millis();

  while(millis() - previousMillis < timeout) {
    if(Serial2.available() > 0) {
      char c = Serial2.read();

      *pTemp = c;
      pTemp++;

      dataCount++;

      if(dataCount == 3) dataLength = c;
      if(dataCount >= dataLength + 5) break;
    }
  }

  return dataCount;
}

void WeatherStation_GetRawData(char *input, char *output, uint8_t dataType) {
  *output = input[dataType + 1];
  *(output + 1) = input[dataType];

  return;
}

float WeatherStation_GetHumidity(char *input) {
  uint16_t tmp;

  WeatherStation_GetRawData(input, (char*)&tmp, WEATHERSTATION_DATA_HUMIDITY);

  return ((float)tmp) / 10;
}

float WeatherStation_GetTemperature(char *input) {
  uint16_t tmp;

  WeatherStation_GetRawData(input, (char*)&tmp, WEATHERSTATION_DATA_TEMPERATURE);

  return ((float)tmp) / 10;
}

float WeatherStation_GetNoise(char *input) {
  uint16_t tmp;

  WeatherStation_GetRawData(input, (char*)&tmp, WEATHERSTATION_DATA_NOISE);

  return ((float)tmp) / 10;
}

float WeatherStation_GetAtmPressure(char *input) {
  uint16_t tmp;

  WeatherStation_GetRawData(input, (char*)&tmp, WEATHERSTATION_DATA_ATMPRESS);

  return ((float)tmp) / 10;
}

float WeatherStation_GetLux(char *input) {
  uint16_t tmpArray[2];
  uint32_t tmp;

  WeatherStation_GetRawData(input, ((char*)&tmp) + 2, WEATHERSTATION_DATA_LUX);
  WeatherStation_GetRawData(input, ((char*)&tmp), WEATHERSTATION_DATA_LUX + 2);

  return ((float)tmp);
}

float WeatherStation_GetLight(char *input) {
  uint16_t tmp;

  WeatherStation_GetRawData(input, (char*)&tmp, WEATHERSTATION_DATA_LIGHT);

  return ((float)tmp) / 10;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);

  pinMode(RAINFALL_PIN, INPUT);

  attachInterrupt(RAINFALL_PIN, rainfall_it_handler, FALLING);

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(ubidots_callback);
  ubidots.setup();
  ubidots.reconnect();

  delay(2000);

  Serial.println("Automatic Weather Station by KMTek");

  output_timer = millis();
  ubidots_timer = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - output_timer >= OUTPUT_MILLIS) {
    if(WeatherStation_Read(buffer, WEATHERSTATION_TIMEOUT) > 0) {
      humidity = WeatherStation_GetHumidity(&buffer[3]);
      temperature = WeatherStation_GetTemperature(&buffer[3]);
      noise = WeatherStation_GetNoise(&buffer[3]);
      atmPressure = WeatherStation_GetAtmPressure(&buffer[3]);
      lux = WeatherStation_GetLux(&buffer[3]);
      light = WeatherStation_GetLight(&buffer[3]);
  
      Serial.print("Humidity: ");
      Serial.println(humidity);
  
      Serial.print("Temperature: ");
      Serial.println(temperature);

      Serial.print("Noise: ");
      Serial.println(noise);

      Serial.print("Atm Pressure: ");
      Serial.println(atmPressure);

      Serial.print("Lux: ");
      Serial.println(lux);

      Serial.print("Light: ");
      Serial.println(light);

      Serial.print("Rainfall: ");
      Serial.println(rainfall);
    }

    output_timer = millis();
  }

  if(!ubidots.connected()) ubidots.reconnect();

  if(millis() - ubidots_timer > PUBLISH_MILLIS) {
    ubidots.add(HUM_LABEL, humidity);
    ubidots.add(TEMP_LABEL, temperature);
    ubidots.add(ATMPRES_LABEL, atmPressure);
    ubidots.add(NOISE_LABEL, noise);
    ubidots.add(LUX_LABEL, lux);
    ubidots.add(RAINFALL_LABEL, rainfall);
    ubidots.add(WINDSPEED_LABEL, windspeed);
    ubidots.add(WINDDIR_LABEL, winddir);

    ubidots.publish(DEVICE_LABEL);

    Serial.println("Ubidots: the data have been published");

    ubidots_timer = millis();
  }
}

void IRAM_ATTR rainfall_it_handler() {
  static uint32_t lastMillis = millis();

  uint32_t delta = millis() - lastMillis;
  if(delta <= 10) return;
  Serial.print("Delta:");
  Serial.println(delta);
  rainfall = RAINFALL_TIPPING_VALUE / ((float)delta / 1000);
  lastMillis = millis();

  return;
}
