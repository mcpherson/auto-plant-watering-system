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

SYSTEM_MODE(AUTOMATIC);
// SYSTEM_THREAD(ENABLED);
SerialLogHandler logHandler(LOG_LEVEL_INFO);

TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 

Adafruit_MQTT_Subscribe buttonSubFeed = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/button-on-off"); 
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
const int PUMPPIN = D16;
// const int DUSTPIN = A2;

int lastWatered;
unsigned int lastTime;
int buttonValue;
int moistMid = 2500;
int dryness, dryMagnitude, moisture;
String moistureState;
String dateTime, m, d, timeOnly, yr;

// dust sensor
// unsigned long duration;
// unsigned long dustStartTime;
// unsigned long dustSampleTime = 30000;
// unsigned long lowPulseOccupancy = 0;
// float ratio = 0;
float concentration = 0;

String getTime();
void bmeRead();
void getMoisture();
void getDustLevel();
void displayData(String timestamp);
void publishData();
void waterPlant();
void MQTT_connect();
bool MQTT_ping();

void setup() {
  Serial.begin(9600);
  waitFor(Serial.isConnected, 10000);

  WiFi.on();
  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }
  Serial.printf("\n\n");

  // Setup MQTT subscription
  // mqtt.subscribe(&emailSubFeed);
  mqtt.subscribe(&buttonSubFeed);

  Time.zone(-6);
  Particle.syncTime();

  status = BME.begin(BME_ADDRESS);
  if (status == false) {
    Serial.printf("BME280 at address 0x%02X failed to start", BME_ADDRESS); 
  }

  display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS);
  display.clearDisplay();

  pinMode(MOISTPIN, INPUT);
  pinMode(PUMPPIN, OUTPUT);

  // pinMode(DUSTPIN, INPUT);
  // dustStartTime = millis();
}

void loop() {
  MQTT_connect();
  MQTT_ping();

  // this is our 'wait for incoming subscription packets' busy subloop
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt.readSubscription(100))) {
    if (subscription == &buttonSubFeed) {
      buttonValue = atoi((char *)buttonSubFeed.lastread);
      Serial.printf("Toggle: %i\n", buttonValue);
      if (buttonValue == 1) {
        Serial.printf("ON\n");
        digitalWrite(PUMPPIN, HIGH);
        delay(500);
      }
      else {
        Serial.printf("OFF\n");
        digitalWrite(PUMPPIN, LOW);
      }
    }
    // if (subscription == &brightnessSubFeed) {
    //   brightnessValue = atoi((char *)brightnessSubFeed.lastread);
    //   Serial.printf("Setting brightness to %i\n", brightnessValue);
    //   analogWrite(LEDPIN2, brightnessValue);
    // }
  }

  // getDustLevel();

  // publish to Adafruit.io every 10 seconds
  if((millis() - lastTime > 10000)) {
    if(mqtt.Update()) {
      String timestamp = getTime();
      bmeRead();
      getMoisture();
      displayData(timestamp);
      publishData();
      Serial.printf("DRY: %i, WET: %i\n", dryness, moisture);
      } 
    lastTime = millis();
  }
}

String getTime() {
  String shortTime;
  dateTime = Time.timeStr();
  m = dateTime.substring(4, 7).toUpperCase();
  d = dateTime.substring(8, 10);
  // h:m
  timeOnly = dateTime.substring(11, 16);
  yr = dateTime.substring(20, 24);
  shortTime.concat(d);
  shortTime.concat(" ");
  shortTime.concat(m);
  shortTime.concat(" ");
  shortTime.concat(yr);
  shortTime.concat(" ");
  shortTime.concat(timeOnly);
  return shortTime;
}

void bmeRead() {
  // temp (F)
  BMEData[0] = ((9.0/5.0) * BME.readTemperature()) + 32.0;
  // pressure (kPa)
  BMEData[1] = BME.readPressure() / 1000;
  // humidity (%RH)
  BMEData[2] = BME.readHumidity();
}

void displayData(String timestamp) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(0, 0);
  display.printf("  %s\n\nTemp:      %0.2f %cF\nHumidity:  %0.2f %%RH\nPressure:  %0.2f kPa\nDust:      %0.0fcf\nMoisture:  %s", timestamp.c_str(), BMEData[0], degreeSymbol, BMEData[2], BMEData[1], concentration, moistureState.c_str());
  display.display();
}

// get reading from moisture sensor and convert to a value that represents moisture, not dryness.
void getMoisture() {
  dryness = analogRead(MOISTPIN);
  dryMagnitude = abs(moistMid - dryness);
  dryness > moistMid 
  ? moisture = moistMid - dryMagnitude 
  : moisture = moistMid + dryMagnitude;
  // NOTE: not all c++ compilers can handle ranges in switch cases.
  switch (moisture) {
    case 3400 ... 4000:
      moistureState = "SATURATED";
      break;
    case 3000 ... 3399:
      moistureState = "WET";
      break;
    case 2500 ... 2999:
      moistureState = "MOIST";
      break;
    case 2000 ... 2499:
      moistureState = "DAMP";
      break;
    case 1600 ... 1999:
      moistureState = "PARCHED";
      break;
    case 0 ... 1599:
      moistureState = "DRY";
      break;
    default:
      break;
  }
}

// void getDustLevel() {
//   duration = pulseIn(DUSTPIN, LOW);
//   lowPulseOccupancy = lowPulseOccupancy + duration;
  
//   if ((millis() - dustStartTime) > dustSampleTime) {
//     ratio = lowPulseOccupancy / (dustSampleTime * 10.0);
//     // using spec sheet curve
//     concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62; 
//     Serial.printf("DUST - LPO: %i, Ratio: %0.3f, Concentration: %0.3f\n", lowPulseOccupancy, ratio, concentration);
//     lowPulseOccupancy = 0;
//     dustStartTime = millis();
//   }
// }

void publishData() {
  temperaturePubFeed.publish(BMEData[0]);
  pressurePubFeed.publish(BMEData[1]);
  humidityPubFeed.publish(BMEData[2]);
  moisturePubFeed.publish(moisture);
}

void waterPlant() {
  // only water if moisture is low and more than 10s have passed since last watering
  if (moisture < 3000 && lastWatered + 10000 > millis()) {
    digitalWrite(PUMPPIN, HIGH);
    delay(500);
    lastWatered = millis();
  }
  else {
    digitalWrite(PUMPPIN, LOW);
  }
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