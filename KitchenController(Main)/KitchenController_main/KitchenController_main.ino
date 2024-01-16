#include <ESP8266WiFi.h>
#include <PubSubClient.h>  // Allows us to connect to, and publish to the MQTT broker
#include "HX711.h"
HX711 scale;

///////// Temp& Humidity Pressure Sensor /////////////////

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include "Adafruit_CCS811.h"

Adafruit_BME280 bme;  // I2C
Adafruit_CCS811 ccs;
///////// Temp& Humidity Pressure Sensor /////////////////

#ifndef STASSID
#define STASSID "Hemlodge"
#define STAPSK "psw#ofhemlodgeAP"
#endif
#define ExFanPin 13 //D7
#define LtPin 4   //D2
#define GVPin 1 //TX
#define SDA_Pin 0 //d3
#define SCL_Pin 2  //D4
#define MDPin 15 //D8
#define WSDtPin 14 //d5
#define WSClPin 12 //d6
#define GsPin 3  //RX
unsigned int AllData_Update_drtn = 800, MQTT_Connection_Check_drtn=3000;
unsigned long Prev_AllData_Update_t = 0, Prev_MQTT_Connection_Check_t=0;
float cRH = 0, cTemp = 0, prvRH = 0, prvTemp = 0, cPr = 0, prvPr = 0, cTVOC=0, cCO2=0, cw1 = 0;
const char* ssid = STASSID;
const char* password = STAPSK;


// MQTT
// Make sure to update this for your own MQTT Broker!
//
const char* mqtt_server = "192.168.0.90";
//         Command Topic Using
const char* mqtt_sub_topic_ExFanc ="1";  //"ExFanc"
const char* mqtt_sub_topic_Ltc = "2";   //"Ltc"
const char* mqtt_sub_topic_GVc = "3";   //"GVc"

const char* mqtt_username = "Alifia";
const char* mqtt_password = "123";
// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "KitchenControllerbr";
// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);  // 1883 is the listener port for the Broker


void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  unsigned status;
  pinMode(ExFanPin, OUTPUT);
  pinMode(LtPin, OUTPUT);
  pinMode(GVPin, OUTPUT);
  digitalWrite(ExFanPin, HIGH);
 digitalWrite(LtPin, HIGH);
  digitalWrite(GVPin, HIGH);
 pinMode(GsPin, INPUT);
 pinMode(MDPin, INPUT);

  Wire.begin(SDA_Pin, SCL_Pin);

 status = bme.begin(0x76, &Wire);
 int z=0;
  if (!status) {
    while (z < 4) {
      status = bme.begin(0x76, &Wire);
      if (status) { break; }
      z++;
      delay(10);
    }
  }
  
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
  }
  ccs.begin(CCS811_ADDRESS,&Wire);
  while(!ccs.available());
  Serial.println( WiFi.localIP());

  client.setCallback(MQTTReceivedMessage);
  client.setSocketTimeout(1);
  if (!client.connected()) {
    Connect();
  }
  scale.begin(WSDtPin, WSClPin);
  scale.set_scale(420.0983); 
  scale.tare(20); 
}

void loop() {
  client.loop();
  if ((unsigned long)(millis() - Prev_MQTT_Connection_Check_t) > MQTT_Connection_Check_drtn) {
    Prev_MQTT_Connection_Check_t=millis();
    if (!client.connected()) {
    Connect();
  }
  }
 
  if ((unsigned long)(millis() - Prev_AllData_Update_t) > AllData_Update_drtn) {
    Prev_AllData_Update_t = millis();
    updatingAllData();
    if(digitalRead(MDPin)){
        client.publish("MDs", "Detected");
  }
  else {
        client.publish("MDs", "Not Detected");   
  }
  
  if(digitalRead(GsPin)){
        client.publish("GSDs", "Not Detected");
  }
  else {
        client.publish("GSDs", "Detected");   
  }


}
}

void MQTTReceivedMessage(String topic, byte* payload, unsigned int length) {
  // Handle the message we received
  //client.publish("ch", "commandreceived");


  uint8_t t = topic.toInt();
  switch (t) {
    case 1:
      if ((char)payload[0] == '1') {
        digitalWrite(ExFanPin, LOW);  //To turn on (Relay state-ON) Sazal_rController1_CH1
      } else {
        digitalWrite(ExFanPin, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH1
      }
      break;

    case 2:
      if ((char)payload[0] == '1') {
        digitalWrite(LtPin, LOW);  //To turn on (Relay state-ON) Sazal_rController1_CH2

      } else {
        digitalWrite(LtPin, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH2
      }
      break;

    case 3:
      if ((char)payload[0] == '1') {
        digitalWrite(GVPin, LOW);  //To turn on (Relay state-OFF) Sazal_rController1_CH3
      } else {

        digitalWrite(GVPin, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH3
      }
      break;
  }
}









bool Connect() {
  // Connect to MQTT Server and subscribe to the topic
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    client.subscribe(mqtt_sub_topic_ExFanc);
    client.subscribe(mqtt_sub_topic_Ltc);
    client.subscribe(mqtt_sub_topic_GVc);
    client.publish("ch", "connected1st");
    return true;
  } else {
    return false;
  }
}


void updatingAllData() {
  cRH = bme.readHumidity() - 0.5;
  cTemp = bme.readTemperature();
  cPr = bme.readPressure() / 100.0F;
  if(!ccs.readData()){
  cCO2=ccs.geteCO2()*1.0;
  cTVOC=ccs.getTVOC()*1.0;
  }
   if (scale.is_ready())
  {
  cw1=scale.get_units(1);
  if (cw1 <0) 
  {
    cw1=0;
  }
  }

  char RH[5] = "", Temp[5] = "", Pr[7] = "", eCO2[5] = "", TVOC[5] = "", w1[7] = "";


  dtostrf(cTemp, 5, 2, Temp);
  client.publish("Temps", Temp);
  dtostrf(cRH, 5, 2, RH);
  client.publish("RHs", RH);
  dtostrf(cPr, 7, 2, Pr);
  client.publish("APs", Pr);
  dtostrf(cCO2, 5, 5, eCO2);
  client.publish("eCO2s", eCO2);
 
  dtostrf(cTVOC, 5, 5, TVOC);
  client.publish("TVOCs", TVOC);
  
  dtostrf(cw1, 2, 2, w1);
  client.publish("w1s",w1);
}

