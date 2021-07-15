/*
 * Project smartWateringSystem
 * Description: Midterm 2
 * Author: Vanessa Benavidez  
 * Date: July 12 2021 
 */

#include "Adafruit_SSD1306.h"
#include "Adafruit_BME280.h"
#include "Wire.h"
#include "Adafruit_MQTT.h"
#include "Adafruit_MQTT/Adafruit_MQTT_SPARK.h" 
#include "Adafruit_MQTT/Adafruit_MQTT.h" 
#include "credentials.h"
#include "Air_Quality_Sensor.h"

AirQualitySensor sensor (A3);
int current_quality;


#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET 4
#define SCREEN_ADDRESS 0x3c
#define BME_ADDRESS 0x76

float tempC;
float pressPA;
float humidRH;


const int WATERPUMP = D11;
const int READMOISTURE = A2; 


int moistureread;
int moisturevalues;

Adafruit_BME280 bme;

Adafruit_SSD1306 display(OLED_RESET);



TCPClient TheClient; 
Adafruit_MQTT_SPARK mqtt(&TheClient,AIO_SERVER,AIO_SERVERPORT,AIO_USERNAME,AIO_KEY); 
Adafruit_MQTT_Publish mqttpublishTemperature = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/tempc");
Adafruit_MQTT_Publish mqttpublishPressure = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/pressPA");
Adafruit_MQTT_Publish mqttpublishHumidity = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/humidRH");
Adafruit_MQTT_Publish mqttpublishMoisture = Adafruit_MQTT_Publish(&mqtt, AIO_USERNAME "/feeds/moistureread");
Adafruit_MQTT_Subscribe mqttwaterpump = Adafruit_MQTT_Subscribe(&mqtt, AIO_USERNAME "/feeds/waterpump");

unsigned long last, lastTime;
int waterpump;

const int DUSTSENSOR = D12;
unsigned long duration;
unsigned long starttime;
unsigned long sampletime_ms = 30000;
unsigned long lowpulseoccupancy = 0; 
float ratio = 0;
float concentration = 0;


void setup() {
 pinMode (WATERPUMP, OUTPUT);
 pinMode (READMOISTURE, INPUT);
 Serial.begin(9600);
 bme.begin(0x76);
 display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
 display.display();
 delay(5000);
 display.clearDisplay();
 display.setTextSize(1);
 display.setTextColor(WHITE);
 display.display();

  WiFi.connect();
  while(WiFi.connecting()) {
    Serial.printf(".");
  }

  // Setup MQTT subscription for onoff feed.
  mqtt.subscribe(&mqttwaterpump);

  pinMode(WATERPUMP, OUTPUT);
  pinMode(D12, INPUT);
  starttime = millis();

  sensor.init();



} 


void loop() {
moistureread = analogRead(READMOISTURE);

if (moistureread>2000) {
digitalWrite(WATERPUMP, HIGH);
    delay(1000);
   }
 else {
     digitalWrite(D11, LOW);
 }
Serial.println(moistureread);
    
tempC = bme.readTemperature();
pressPA = bme.readPressure();
humidRH = bme.readHumidity();

display.setCursor(0,0);
display.clearDisplay();
display.printf("Temp is %0.1f\n", tempC);
display.printf("Pressure is %0.1f\n", pressPA);
display.printf("Humidity is %0.1f\n", humidRH);
display.display();


MQTT_connect();


Adafruit_MQTT_Subscribe *subscription;
   while ((subscription = mqtt.readSubscription(1000))) {
    if (subscription == &mqttwaterpump) {
       waterpump = atoi((char *)mqttwaterpump.lastread);
           Serial.printf("Received %i from Adafruit.io feed mqttSmartPlant \n",waterpump);
      
    }
    if (waterpump >= 1) {
      digitalWrite(WATERPUMP, HIGH);
      delay(500);
      digitalWrite(WATERPUMP, LOW);
      waterpump = 0;
    }
   }

  
  if((millis()-lastTime > 30000)) {
   if(mqtt.Update()) {
  mqttpublishTemperature.publish(tempC);
  Serial.printf("Publishing %0.2f \n", tempC);
  mqttpublishPressure.publish(pressPA); 
  Serial.printf("Publishing %0.2f \n", pressPA);
  mqttpublishHumidity.publish(humidRH); 
  Serial.printf("Publishing %0.2f \n", humidRH);
  moistureread = analogRead(READMOISTURE);
  mqttpublishMoisture.publish(moistureread); 
  Serial.printf("Publishing %i \n", moistureread);

     } 
 lastTime = millis();
  }
 
 duration = pulseIn(D12, LOW);
 lowpulseoccupancy = lowpulseoccupancy+duration;
 
 if ((millis()-starttime) > sampletime_ms)
 {
   ratio = lowpulseoccupancy/(sampletime_ms*10.0);
   concentration = 1.1*pow(ratio,3)-3.8*pow(ratio,2)+520*ratio+0.62;
   Serial.print(lowpulseoccupancy);
   Serial.print(",");
   Serial.print(ratio);
   Serial.print(",");
   Serial.println(concentration);
   lowpulseoccupancy = 0;
   starttime = millis();
 }
 current_quality=sensor.slope();
 if (current_quality >= 0)
 {
   if (current_quality==0)
      Serial.println("High pollution! Force signal active");
    else if (current_quality==1)
       Serial.println("High pollution!");
    else if (current_quality==2)
       Serial.println("Low pollution!");
    else if (current_quality==3)
       Serial.println("Fresh air!");
}


 }

void MQTT_connect() {
 int8_t ret;
 
  // Stop if already connected.
  if (mqtt.connected()) {
    return;
  }
 
  Serial.print("Connecting to MQTT... ");
 
  while ((ret = mqtt.connect()) != 0) { // connect will return 0 for connected
       Serial.printf("%s\n",(char *)mqtt.connectErrorString(ret));
       Serial.printf("Retrying MQTT connection in 5 seconds..\n");
       mqtt.disconnect();
       delay(5000);  // wait 5 seconds
  }
  Serial.printf("MQTT Connected!\n");


}



