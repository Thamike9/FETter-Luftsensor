#include <Arduino.h>
#include <WiFiManager.h>
#include "EspMQTTClient.h"
#include <EEPROM.h>
#include "FastLED.h"
#include "credentials.h"
#include <Wire.h>
#include <SparkFun_SCD30_Arduino_Library.h>
#include <PMserial.h>

#define LED_PIN 2
#define NUM_LEDS 11
#define PMS_RX D4
#define PMS_TX D3

CRGB leds[NUM_LEDS];

#define CO2_THRESHOLD 1000
#define CO2_CRITICAL 1500
#define PM2_5_THRESHOLD 35
#define PM2_5_CRITICAL 75
#define TEMP_LOW 20
#define TEMP_HIGH 26
#define TEMP_CRITICAL_LOW 15
#define TEMP_CRITICAL_HIGH 30
#define HUMIDITY_LOW 30
#define HUMIDITY_HIGH 60
#define HUMIDITY_CRITICAL_LOW 20
#define HUMIDITY_CRITICAL_HIGH 70

EspMQTTClient client{
    WIFI_SSID,
    WIFI_PASSWORD,
    MQTT_BROKER,
    MQTT_USER,
    MQTT_USER_PASSWORD,
    MQTT_DEVICE_ID,
    1883};

SCD30 scd30;
SerialPM pms(PMS5003, PMS_RX, PMS_TX); // PMSx003, RX, TX

void updateLEDs(float co2, float pm2_5, float temperature, float humidity);
String measurementsToString(float co2, float pm2_5, float temperature, float humidity);
float co2 = -1, temperature = -1, humidity = -1, pm2_5 = -1;

void setup()
{
  Serial.begin(115200);
  Serial1.begin(9600, SERIAL_8N1, SERIAL_FULL);
  Wire.begin();
  delay(500);
  Serial.printf("Hello!\n %s is awake!\n", MQTT_DEVICE_ID);
  WiFiManager wifiManager;
  wifiManager.autoConnect(MQTT_DEVICE_ID);

  if (!scd30.begin(true)) //activates autocalibration
  {
    Serial.printf("%ld: SCD Sensor was NOT found!\n", millis());
  }
  else
  {
    Serial.printf("%ld: SCD Sensor found!\n", millis());
  }

  pms.init();
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  FastLED.clear();
  FastLED.show();
  Serial.printf("%ld: Setup complete!\nAutocalibration: %d\n", millis(), scd30.getAutoSelfCalibration());
  
}

void onConnectionEstablished()
{
  Serial.printf("%ld: MQTT connected!\n", millis());
}
void loop()
{
  client.loop(); // Ensure the MQTT client stays connected
  static unsigned long lastMeasurement = 0;
  static unsigned long lastSerial = 0;

  if (millis() - lastMeasurement >= MEASUREMENT_INTERVAL)
  {
    lastMeasurement = millis();
    co2 = -1;
    temperature = -1;
    humidity = -1;
    pm2_5 = -1;

    if (scd30.dataAvailable())
    {
      co2 = scd30.getCO2();
      temperature = scd30.getTemperature();
      humidity = scd30.getHumidity();
      Serial.printf("%ld: SCD30 data acquired!\n", millis());
    }
    else
    {
      Serial.printf("%ld: SCD30 data unavailable, check sensor!\n", millis());
    }
    pms.wake();
    delay(3000);
    if (pms.read())
    {
      pm2_5 = pms.pm25;
      Serial.printf("%ld: PMS data acquired!\n", millis());
      Serial.printf("PM1.0 %2d, PM2.5 %2d, PM10 %2d [ug/m3]\n",
                    pms.pm01, pms.pm25, pms.pm10);
    }
    else
    {
      Serial.printf("%ld: PMS data unavailable, check sensor!\n", millis());
    }
    // pms.sleep();

    if (client.isConnected())
    {
      if (co2 >= 0)
        client.publish(String(MQTT_DEVICE_ID) + "/CO2", String(co2));
      if (temperature >= 0)
        client.publish(String(MQTT_DEVICE_ID) + "/TEMP", String(temperature));
      if (humidity >= 0)
        client.publish(String(MQTT_DEVICE_ID) + "/RH", String(humidity));
      if (pm2_5 >= 0)
        client.publish(String(MQTT_DEVICE_ID) + "/PM25", String(pm2_5));
      Serial.printf("%ld: Payload from %s sent!\n", millis(), MQTT_DEVICE_ID);
    }
    else
    {
      Serial.printf("%ld: MQTT not connected, skipping publish!\n", millis());
    }

    Serial.printf("%ld: LEDs updated!\n", millis());
    updateLEDs(co2, pm2_5, temperature, humidity);
  }

  if (millis() - lastSerial >= SERIAL_INTERVAL)
  {
    Serial.printf("%ld: %s\n", millis(), measurementsToString(co2, pm2_5, temperature, humidity).c_str());
    unsigned long remainingTime = MEASUREMENT_INTERVAL - (millis() - lastMeasurement);
    unsigned long minutes = remainingTime / 60000;
    unsigned long seconds = (remainingTime % 60000) / 1000;
    Serial.printf("%ld: Next update in: %lu min %lu sec\n", millis(), minutes, seconds);
    lastSerial = millis();
  }

  yield(); // Allow ESP8266 background tasks
}

void updateLEDs(float co2, float pm2_5, float temperature, float humidity)
{
  CRGB co2Color = (co2 < CO2_THRESHOLD) ? CRGB::Green : (co2 < CO2_CRITICAL) ? CRGB::Yellow
                                                                             : CRGB::Red;
  leds[0] = leds[1] = leds[2] = co2Color;
  if (co2 >= CO2_CRITICAL)
  {
    leds[0] = leds[1] = leds[2] = (millis() / 500) % 2 ? CRGB::Red : CRGB::Black;
  }

  CRGB pmColor = (pm2_5 < PM2_5_THRESHOLD) ? CRGB::Green : (pm2_5 < PM2_5_CRITICAL) ? CRGB::Yellow
                                                                                    : CRGB::Red;
  leds[3] = leds[4] = leds[5] = leds[6] = pmColor;
  if (pm2_5 >= PM2_5_CRITICAL)
  {
    leds[3] = leds[4] = leds[5] = leds[6] = (millis() / 500) % 2 ? CRGB::Red : CRGB::Black;
  }

  CRGB tempColor = (temperature >= TEMP_LOW && temperature <= TEMP_HIGH) ? CRGB::Green : (temperature >= TEMP_CRITICAL_LOW && temperature <= TEMP_CRITICAL_HIGH) ? CRGB::Yellow
                                                                                     : (temperature < TEMP_CRITICAL_LOW)                                         ? CRGB::Blue
                                                                                                                                                                 : CRGB::Red;
  leds[7] = leds[8] = tempColor;

  CRGB humidityColor = (humidity >= HUMIDITY_LOW && humidity <= HUMIDITY_HIGH) ? CRGB::Green : (humidity >= HUMIDITY_CRITICAL_LOW && humidity <= HUMIDITY_CRITICAL_HIGH) ? CRGB::Yellow
                                                                                           : (humidity < HUMIDITY_CRITICAL_LOW)                                          ? CRGB::Blue
                                                                                                                                                                         : CRGB::Red;
  leds[9] = leds[10] = humidityColor;

  FastLED.show();
}

String measurementsToString(float co2, float pm2_5, float temperature, float humidity)
{
  return "CO2: " + String(co2) + " ppm, " +
         "PM2.5: " + String(pm2_5) + " µg/m³, " +
         "Temperature: " + String(temperature) + " °C, " +
         "Humidity: " + String(humidity) + " %";
}
