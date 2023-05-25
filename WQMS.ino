#include <Arduino.h>
#include <UbidotsEsp32Mqtt.h>

//PH
#define PH_PIN 34
#define PH_SAMPLE_MILLIS 10
float PH_value;
float offset = -0.2;

//TDS
#define TDS_PIN 35
#define TDS_SAMPLE_MILLIS 10
float TDS_value;
float temperature = 25;

#define SAMPLING_COUNT_MAX 100
unsigned long sampling_timer;
unsigned int sampling_count = 0;

//Output
#define OUTPUT_MILLIS 1000
unsigned long output_timer = 0;

//Connectivity
#define WIFI_SSID "YOUR SSID"
#define WIFI_PASS "YOUR PASSWORD"

//Ubidots
#define PUBLISH_MILLIS 5000
const char *UBIDOTS_TOKEN = "UBIDOTS TOKEN";
const char *DEVICE_LABEL = "mappi32-waterquality";
const char *TDS_LABEL = "tds";
const char *PH_LABEL = "ph";
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

float convertTDS(float tdsVoltage) {
  return (133.42 * tdsVoltage * tdsVoltage * tdsVoltage - 255.86 * tdsVoltage * tdsVoltage + 857.39 * tdsVoltage) * 0.5;
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  ubidots.connectToWifi(WIFI_SSID, WIFI_PASS);
  ubidots.setCallback(ubidots_callback);
  ubidots.setup();
  ubidots.reconnect();

  Serial.println("Water Quality by KMTek");
  delay(2000);

  sampling_timer = millis();
  ubidots_timer = millis();
  output_timer = millis();
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - sampling_timer > TDS_SAMPLE_MILLIS) {
    TDS_value = convertTDS(3.30 * (float)analogRead(TDS_PIN) / 4096.0);
    PH_value = (((float)analogRead(PH_PIN) - 0.0) * (0.0 - 14.0) / (3722.0 - 0.0) + 14.0) + offset;

    sampling_timer = millis();
  }

  if(millis() - output_timer > OUTPUT_MILLIS) {
    Serial.print("TDS: ");
    Serial.println(TDS_value);

    Serial.print("pH: ");
    Serial.println(PH_value);

    output_timer = millis();
  }

  if(!ubidots.connected()) ubidots.reconnect();

  if(millis() - ubidots_timer > PUBLISH_MILLIS) {
    ubidots.add(TDS_LABEL, TDS_value);
    ubidots.add(PH_LABEL, PH_value);
    ubidots.publish(DEVICE_LABEL);

    Serial.println("Ubidots: the data have been published");

    ubidots_timer = millis();
  }
}
