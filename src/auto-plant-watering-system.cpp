/* 
 * Project: Automatic Plant Watering System
 * Author: Marlon McPherson
 * Date: 20 MAR 2025
 */

#include "Particle.h"
#include "credentials.h"
#include "Adafruit_BME280.h"
#include "Adafruit_SSD1306.h"
#include "Adafruit_GFX.h"
#include <Adafruit_MQTT.h>
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h"
#include "Adafruit_MQTT/Adafruit_MQTT.h"

SYSTEM_MODE(SEMI_AUTOMATIC);
// SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Subscribe emailSubFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/email"); 
Adafruit_MQTT_Publish temperaturePubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/midterm2-temperature");
Adafruit_MQTT_Publish humidityPubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/midterm2-humidity");
Adafruit_MQTT_Publish pressurePubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/midterm2-pressure");
Adafruit_MQTT_Publish moisturePubFeed = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/midterm2-moisture");

const byte BME_ADDRESS = 0x76;
Adafruit_BME280 BME;
byte status;
byte degreeSymbol = 248;
float BMEData[3];

const byte OLED_ADDRESS = 0x3C;

const int OLED_RESET=-1;
Adafruit_SSD1306 display(OLED_RESET);

const int MOISTPIN = A0;
const int DUSTPIN = A2;

unsigned int lastTime;

void bmeRead();
void bmeDisplay();
void MQTT_connect();
bool MQTT_ping();

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  // Connect to Internet but not Particle Cloud
  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  status = BME.begin(BME_ADDRESS);
  if (status == false) {
    Serial.printf("BME280 at address 0x%02X failed to start", BME_ADDRESS); 
  }

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.clearDisplay();
}

void loop() {
  MQTT_connect();
  MQTT_ping();

  // publish to Adafruit.io every 10 seconds
  if((millis() - lastTime > 10000)) {
    if(mqtt.Update()) {
      bmeRead();
      temperaturePubFeed.publish(BMEData[0]);
      pressurePubFeed.publish(BMEData[1]);
      humidityPubFeed.publish(BMEData[2]);
      // moisturePubFeed.publish(moisture);
      } 
    lastTime = millis();
  }
}

void bmeRead() {
  // temp (F)
  BMEData[0] = ((9.0/5.0) * BME.readTemperature()) + 32.0;
  // pressure (kPa)
  BMEData[1] = BME.readPressure() / 1000;
  // humidity (%RH)
  BMEData[2] = BME.readHumidity();
  bmeDisplay();
}

void bmeDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.printf("Temp:     %0.2f %cF\nHumidity: %0.2f %%RH\nPressure: %0.2f kPa", BMEData[0], degreeSymbol, BMEData[2], BMEData[1]);
  display.display();
}

// Function to connect and reconnect as necessary to the MQTT server.
// Should be called in the loop function and it will take care if connecting.
void MQTT_connect() {
  int8_t ret;
 
  // Return if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("Error Code %s\n",mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds...\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds and try again
  }
  Serial.printf("MQTT Connected!\n");
}

bool MQTT_ping() {
  static unsigned int last;
  bool pingStatus;

  if ((millis()-last)>120000) {
      Serial.printf("Pinging MQTT \n");
      pingStatus = mqtt.ping();
      if(!pingStatus) {
        Serial.printf("Disconnecting \n");
        mqtt.disconnect();
      }
      last = millis();
  }
  return pingStatus;
}