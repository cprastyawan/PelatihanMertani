#include <Arduino.h>
#include <DallasTemperature.h>
#include <UbidotsEsp32Mqtt.h>

//Temperature Sensor
#define DS18B20_PIN 32
#define TEMPERATURE_SAMPLE_MILLIS 1000

OneWire oneWire(DS18B20_PIN);
DallasTemperature TemperatureSensor(&oneWire);

float temperature_value = 0.0f;
unsigned long temperature_timer = 0;

//Water Level Sensor
#define RS485_PIN_RX 16
#define RS485_PIN_TX 17
#define RS485_PIN_DE 
#define RS485_SERIAL Serial2
#define RS485_RECEIVE LOW
#define RS485_TRANSMIT HIGH
#define WATERLEVEL_SAMPLE_MILLIS 1000

unsigned long waterlevel_timer = 0;
float waterlevel_value = 0.0f;
char *pTmp = NULL;

//Output
#define OUTPUT_MILLIS 1000
unsigned long output_timer = 0;

//Connectivity
#define WIFI_SSID "YOUR SSID"
#define WIFI_PASS "YOUR PASSWORD"

//Ubidots
#define PUBLISH_MILLIS 5000
const char *UBIDOTS_TOKEN = "UBIDOTS TOKEN";
const char *DEVICE_LABEL = "mappi32-waterlevel";
const char *TEMPERATURE_LABEL = "temperature";
const char *WATERLEVEL_LABEL = "water-level";
unsigned long ubidots_timer = 0;

Ubidots ubidots(UBIDOTS_TOKEN);

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

char* WaterLevelSensor_Read() {
  if(RS485_SERIAL.available()) {
    static unsigned char i;
    static bool startRead;
    static char buffer[4];

    char c = RS485_SERIAL.read();
    if(c == 0xFF) startRead = true;

    

    if(startRead == true) {
      buffer[i] = c;
      i++;
    }

    if(i >= 4) {
      i = 0;
      startRead = false;

      return buffer;
    }
  }

  return NULL;
}

unsigned char WaterLevelSensor_Process(char* buffer, float* output) {
  unsigned int sum = (buffer[0] + buffer[1] + buffer[2]) & 0x00FF;

  if(sum == buffer[3]) {
    *output = (float)((buffer[1] * 256) + buffer[2]);
    if(*output >= 220) {
      *output /= 10;

      return 1;
    } else *output = 0;
  }

  return 0;
}

unsigned int WaterLevelSensor_Sum(float input, float *output) {
  static float tmp;
  static unsigned int count;
  unsigned int tmpCount = count;

  if(output != NULL) {
    *output = tmp;
    tmp = 0.0f;
    count = 0;
  } else {
    tmp += input;
    count++;
  }

  return tmpCount;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial2.begin(9600);

  TemperatureSensor.begin();

  Serial.println("WATER LEVEL SENSOR By KMTek");

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(ubidots_callback);
  ubidots.setup();
  ubidots.reconnect();

  delay(2000);

  temperature_timer = millis();
  output_timer = millis();
  ubidots_timer = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  char *pWaterLevelSensorOutput = WaterLevelSensor_Read();

  if(pWaterLevelSensorOutput != NULL) {
    float temp;
    if(WaterLevelSensor_Process(pWaterLevelSensorOutput, &temp)) {
      WaterLevelSensor_Sum(temp, NULL);
    }
  }

  if(millis() - temperature_timer >= TEMPERATURE_SAMPLE_MILLIS) {
    TemperatureSensor.requestTemperatures();
    temperature_value = TemperatureSensor.getTempCByIndex(0);

    temperature_timer = millis();
  }

  if(millis() - output_timer >= OUTPUT_MILLIS) {
    unsigned int count = WaterLevelSensor_Sum(0, &waterlevel_value);

    waterlevel_value /= count;

    Serial.print("Water Level: ");
    Serial.print(waterlevel_value);
    Serial.println(" cm");

    Serial.print("Temperature: ");
    Serial.print(temperature_value);
    Serial.println(" C");

    output_timer = millis();
  }

  if(!ubidots.connected()) ubidots.reconnect();

  if(millis() - ubidots_timer > PUBLISH_MILLIS) {
    ubidots.add(TEMPERATURE_LABEL, temperature_value);
    ubidots.add(WATERLEVEL_LABEL, waterlevel_value);
    ubidots.publish(DEVICE_LABEL);

    Serial.println("Ubidots: the data have been published");

    ubidots_timer = millis();
  }
}
