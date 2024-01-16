#include <ESP8266WiFi.h>
#include <PubSubClient.h>  // Allows us to connect to, and publish to the MQTT broker


///////// Temp& Humidity Pressure Sensor /////////////////

#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BME280 bme;  // I2C

///////// Temp& Humidity Pressure Sensor /////////////////
#include "WiFiUdp.h"
#ifndef STASSID
#define STASSID "alifia 8"
#define STAPSK "12345678"
#endif
#define OutputPin1 10
#define OutputPin2 12
#define OutputPin3 14

#define SDA_Pin 0
#define SCL_Pin 2


uint8_t OutputPin1_Current_Status = 1, OutputPin1_Prv_Status = 0, OutputPin2_Current_Status = 0, OutputPin2_Prv_Status = 0, OutputPin3_Current_Status = 1,
        OutputPin3_Prv_Status = 0, ServoSwitch_Status = 0, ServoSwitch_Default_Status = 0, BedTime = 0, Occupant_Presence_Pin_Prv_Status = 1,
        z = 0, i = 0, Time_updated = 0, cHours = 50, prvHours = 50, cAMPM = 5, prvAMPM = 5, cDaySN = 10, prvDaySN = 10, WiFistatus = 0,
        InputPin1_prv_Status = 1, InputPin2_prv_Status = 0, InputPin3_prv_Status = 1, AllChannelUpdated = 0, PrvAngle[2] = { 90, 90 }, ProrALT = 1, Manual_On = 0,
        Charging_Status = 0, Charging_Start_t[2] = { 0, 0 }, MQTTBroker_Reconnecting_attempt = 0;

unsigned int Btn_db_drtn = 400, Time_Update_drtn = 10000, TempRHPrALT_Update_drtn = 30000, MQTTBroker_Reconnecting_drtn = 16000, Wifi_Checking_drtn = 10000, RM = 0, prvED = 0, cED = 0, rED = 0, Y = 0, M = 0, D = 0, pa = 0, ca = 0;
unsigned long Prev_Time_Update_t = 0, Prev_Btn_Prs_T = 0, Prev_TempRHPrALT_Update_t = 0, Prev_MQTTBroker_connecting_t = 0, Prev_WiFi_Checking_t = 0;
float cRH = 0, cTemp = 0, prvRH = 0, prvTemp = 0, cPr = 0, cALT = 0, prvPr = 0, prvALT = 0;
const char* ssid = STASSID;
const char* password = STAPSK;


// MQTT
// Make sure to update this for your own MQTT Broker!
const char* mqtt_server = "192.168.250.186";
const char* mqtt_sub_topic_ExFanc = "ExFanc";
const char* mqtt_sub_topic_Ltc = "Ltc";
const char* mqtt_sub_topic_GVc = "GVc";

const char* mqtt_username = "alifia";
const char* mqtt_password = "123";
// The client id identifies the ESP8266 device. Think of it a bit like a hostname (Or just a name, like Greg).
const char* clientID = "KitchenController";

// Initialise the WiFi and MQTT Client objects
WiFiClient wifiClient;
PubSubClient client(mqtt_server, 1883, wifiClient);  // 1883 is the listener port for the Broker
//SoftwareSerial mySerial(RX_Pin, TX_Pin); // RX, TX
WiFiUDP ntpUDPKitchenController1;
void setup() {
  WiFi.begin(ssid, password);
  unsigned status;
  pinMode(OutputPin1, OUTPUT);
  pinMode(OutputPin2, OUTPUT);
  pinMode(OutputPin3, OUTPUT);
  digitalWrite(OutputPin1, HIGH);
  digitalWrite(OutputPin2, HIGH);
  digitalWrite(OutputPin3, HIGH);
  pinMode(InputPin1, INPUT);
  pinMode(InputPin2, INPUT);
  pinMode(InputPin3, INPUT);
  Wire.begin(SDA_Pin, SCL_Pin);

  status = bme.begin(0x76, &Wire);
  if (!status) {
    while (z < 4) {
      status = bme.begin(0x76, &Wire);
      if (status) { break; }
      z++;
      delay(10);
    }

  }

  
  while ((WiFi.status() != WL_CONNECTED) && (i < 12)) {
    delay(400);
    i++;
  }
  client.setCallback(MQTTReceivedMessage);
  client.setSocketTimeout(1);
  if (!client.connected()) {
    Connect();
  }
  timeClient.begin();
  timeClient.update();
  //mySerial.begin(4800);
  //mySerial.write(1);
  //delay(2);
  //SerialReceiveEvent();
}

void loop() {
  //SerialReceiveEvent();
  PhysicalSwitchHandling();
  if ((unsigned long)(millis() - Prev_Time_Update_t) >= Time_Update_drtn) {
    //timeClient.update();
    Prev_Time_Update_t = millis();
    if (Time_updated == 0) {
      if (timeClient.update()) {
        Time_updated = 1;
        timeClient.setUpdateInterval(6000000);
        UpdatingAllChannel();
        AllChannelUpdated = 1;
        Time_Update_drtn = 60000;
        prvHours = cHours;
        cHours = timeClient.getHours();
        updatingTime();
        Prev_Time_Update_t = millis() - timeClient.getSeconds() * 1000 + 1;
      }
    } else {
      prvHours = cHours;
      cHours = timeClient.getHours();

      if ((cHours > 16) || (cHours < 2)) {
        if ((ServoSwitch_Status == 0) && (BedTime == 0)) {
          if (ServoSwitch_Default_Status == 0) {
            ServoSwitch_Default_Status = 1;
            EEPROM.write(ServoSwitch_Default_Status_address, 1);
            EEPROM.commit();
          }
          ServoSwitchOnOff(1);
        }
      } else {
        if ((OutputPin3_Current_Status == 0) && (ServoSwitch_Status == 1)) {
          if (ServoSwitch_Default_Status == 1) {
            ServoSwitch_Default_Status = 0;
            EEPROM.write(ServoSwitch_Default_Status_address, 0);
            EEPROM.commit();
          }
          ServoSwitchOnOff(0);
        }
      }
      if ((cHours > 6) && (cHours < 16) && (BedTime == 1)) {
        BedTimeRoutineOnOff(2);
      }
      updatingTime();
      if (BedTime == 0) { RL_Charging_cycle(); }
    }
  }
  if ((unsigned long)(millis() - Prev_TempRHPrALT_Update_t) > TempRHPrALT_Update_drtn) {
    Prev_TempRHPrALT_Update_t = millis();
    updatingTempRHPrALT();
  }

  if ((unsigned long)(millis() - Prev_WiFi_Checking_t) > Wifi_Checking_drtn) {
    if (WiFi.status() == WL_CONNECTED) {
      WiFistatus = 1;
    } else {
      WiFistatus = 0;
      WiFi.begin(ssid, password);
      Wifi_Checking_drtn = 10000;
    }
    Prev_WiFi_Checking_t = millis();
  }

  if (WiFistatus == 1) {
    // If the connection is lost, try to connect again
    if (!client.connected()) {
      if ((unsigned long)(millis() - Prev_MQTTBroker_connecting_t) > MQTTBroker_Reconnecting_drtn) {
        Connect();
        Prev_MQTTBroker_connecting_t = millis();
        MQTTBroker_Reconnecting_attempt++;

        if (MQTTBroker_Reconnecting_attempt == 3) {
          MQTTBroker_Reconnecting_attempt = 0;
          WiFi.disconnect();
          Wifi_Checking_drtn = 180000;
          WiFistatus = 0;
          Prev_WiFi_Checking_t = millis();
        }
      }
      if (AllChannelUpdated == 1) { AllChannelUpdated = 0; }

    } else {
      // client.loop() just tells the MQTT client code to do what it needs to do itself (i.e. check for messages, etc.)
      client.loop();
      if (AllChannelUpdated == 0) {
        UpdatingAllChannel();
        AllChannelUpdated = 1;
      }
    }
  } else {
    if (AllChannelUpdated == 1) { AllChannelUpdated = 0; }
  }
}


void MQTTReceivedMessage(String topic, byte* payload, unsigned int length) {
  // Handle the message we received
  uint8_t t = topic.toInt();
  switch (t) {
    case 1:
      if ((char)payload[0] == '1') {
        digitalWrite(OutputPin1, LOW);  //To turn on (Relay state-ON) Sazal_rController1_CH1
        OutputPin1_Prv_Status = OutputPin1_Current_Status;
        OutputPin1_Current_Status = 0;  // pin status is opposite to relay status
        EEPROM.write(OutputPin1address, 10);
        EEPROM.commit();
      } else {
        digitalWrite(OutputPin1, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH1
        OutputPin1_Prv_Status = OutputPin1_Current_Status;
        OutputPin1_Current_Status = 1;
        EEPROM.write(OutputPin1address, 01);
        EEPROM.commit();
      }
      break;

    case 2:
      if ((char)payload[0] == '1') {
        digitalWrite(OutputPin2, LOW);  //To turn on (Relay state-ON) Sazal_rController1_CH2
        OutputPin2_Prv_Status = OutputPin2_Current_Status;
        OutputPin2_Current_Status = 0;  // pin status is opposite to relay status
        EEPROM.write(OutputPin2address, 10);
        EEPROM.commit();
      } else {
        digitalWrite(OutputPin2, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH2
        OutputPin2_Prv_Status = OutputPin2_Current_Status;
        OutputPin2_Current_Status = 1;  // pin status is opposite to relay status
        EEPROM.write(OutputPin2address, 01);
        EEPROM.commit();
      }
      break;

    case 3:
      if ((char)payload[0] == '1') {
        delay(100);
        if (ServoSwitch_Status == 0) { ServoSwitchOnOff(1); }
        delay(100);
        digitalWrite(OutputPin3, HIGH);  //To turn on (Relay state-OFF) Sazal_rController1_CH3
        OutputPin3_Prv_Status = OutputPin3_Current_Status;
        OutputPin3_Current_Status = 1;  // pin status is same as relay status (normally close connection)
        EEPROM.write(OutputPin3address, 01);
        EEPROM.commit();
        Manual_On = 1;

      } else {
        delay(100);
        if (ServoSwitch_Default_Status == 0) { ServoSwitchOnOff(0); }
        delay(100);
        digitalWrite(OutputPin3, LOW);  //To turn off (Relay state-OFF) Sazal_rController1_CH3
        OutputPin3_Prv_Status = OutputPin3_Current_Status;
        OutputPin3_Current_Status = 0;
        EEPROM.write(OutputPin3address, 10);
        EEPROM.commit();
        Manual_On = 0;
      }
      break;

    case 4:
      if ((char)payload[0] == '1') {
        ServoSwitchOnOff(1);  //To turn on (Servo Switch state-ON) Sazal_rController1_CH4
      } else {
        ServoSwitchOnOff(0);  //To turn off (Servo Switch state-OFF) Sazal_rController1_CH4
      }
      break;

    case 5:
      if ((char)payload[0] == '1') {
        BedTimeRoutineOnOff(1);
      } else {
        BedTimeRoutineOnOff(0);
      }
      break;

    case 8:
      ServoSwitchCalibration(payload, length);
      break;

    case 11:
      FanRegulatorPositionChange(payload, length);
      break;

    case 12:
      UpdatingAllChannel();
      break;


    default:
      break;
  }
}


void EEPROM_Read() {
  EEPROM.begin(512);
  uint8_t ev1 = EEPROM.read(OutputPin1address), ev2 = EEPROM.read(OutputPin2address), ev3 = EEPROM.read(OutputPin3address);
  switch (ev1) {
    case 00:
      digitalWrite(OutputPin1, LOW);
      OutputPin1_Current_Status = 0;
      OutputPin1_Prv_Status = 0;
      break;

    case 01:
      digitalWrite(OutputPin1, HIGH);
      OutputPin1_Current_Status = 1;
      OutputPin1_Prv_Status = 0;
      break;

    case 10:
      digitalWrite(OutputPin1, LOW);
      OutputPin1_Current_Status = 0;
      OutputPin1_Prv_Status = 1;
      break;

    case 11:
      digitalWrite(OutputPin1, HIGH);
      OutputPin1_Current_Status = 1;
      OutputPin1_Prv_Status = 1;
      break;
  }
  switch (ev2) {
    case 00:
      digitalWrite(OutputPin2, LOW);
      OutputPin2_Current_Status = 0;
      OutputPin2_Prv_Status = 0;
      break;

    case 01:
      digitalWrite(OutputPin2, HIGH);
      OutputPin2_Current_Status = 1;
      OutputPin2_Prv_Status = 0;
      break;

    case 10:
      digitalWrite(OutputPin2, LOW);
      OutputPin2_Current_Status = 0;
      OutputPin2_Prv_Status = 1;
      break;

    case 11:
      digitalWrite(OutputPin2, HIGH);
      OutputPin2_Current_Status = 1;
      OutputPin2_Prv_Status = 1;
      break;
  }
  switch (ev3) {
    case 00:
      digitalWrite(OutputPin3, LOW);
      OutputPin3_Current_Status = 0;
      OutputPin3_Prv_Status = 0;
      break;

    case 01:
      digitalWrite(OutputPin3, HIGH);
      OutputPin3_Current_Status = 1;
      OutputPin3_Prv_Status = 0;
      break;

    case 10:
      digitalWrite(OutputPin3, LOW);
      OutputPin3_Current_Status = 0;
      OutputPin3_Prv_Status = 1;
      break;

    case 11:
      digitalWrite(OutputPin3, HIGH);
      OutputPin3_Current_Status = 1;
      OutputPin3_Prv_Status = 1;
      break;
  }
  if (EEPROM.read(LCDBackLightaddress) == 0) {
    lcd.noBacklight();
  } else {
    lcd.backlight();
  }
  if (EEPROM.read(ServoSwitchaddress) == 0) {
    PrvAngle[0] = 8;
    ServoSwitchOnOff(0);
  } else {
    PrvAngle[0] = 0;
    ServoSwitchOnOff(1);
  }
  if (EEPROM.read(ServoSwitch_Default_Status_address) == 0) {
    ServoSwitch_Default_Status = 0;
  } else {
    ServoSwitch_Default_Status = 1;
  }
  if (EEPROM.read(BedTime_Status_address) == 0) {
    BedTime = 0;
  } else {
    BedTime = 1;
  }
  PrvAngle[1] = EEPROM.read(FanRegulatoraddress);
}

void ServoSwitchOnOff(uint8_t p) {
  if (p == 1) {
    RotatingServoHorn(0, ServoSwitchPin, 0);
    ServoSwitch_Status = 1;
    EEPROM.write(ServoSwitchaddress, 1);
    EEPROM.commit();
    client.publish("4s", "1");
    client.publish("8s", "0");
  } else {
    RotatingServoHorn(8, ServoSwitchPin, 0);
    ServoSwitch_Status = 0;
    EEPROM.write(ServoSwitchaddress, 0);
    EEPROM.commit();
    client.publish("4s", "0");
    client.publish("8s", "8");
  }
}

void BedTimeRoutineOnOff(uint8_t s) {
  if (s == 1) {
    BedTime = 1;
    EEPROM.write(BedTime_Status_address, 1);
    EEPROM.commit();
    if (OutputPin1_Current_Status == 0) {
      digitalWrite(OutputPin1, HIGH);
      client.publish("1s", "0");
      OutputPin1_Prv_Status = 0;
      OutputPin1_Current_Status = 1;
      EEPROM.write(OutputPin1address, 01);
      EEPROM.commit();
    } else {
      OutputPin1_Prv_Status = 1;
      OutputPin1_Current_Status = 1;
      EEPROM.write(OutputPin1address, 11);
      EEPROM.commit();
    }

    if (OutputPin3_Current_Status == 1) {
      digitalWrite(OutputPin3, LOW);
      client.publish("3s", "0");
      OutputPin3_Prv_Status = 1;
      OutputPin3_Current_Status = 0;
      EEPROM.write(OutputPin3address, 10);
      EEPROM.commit();
    } else {
      OutputPin3_Prv_Status = 0;
      OutputPin3_Current_Status = 0;
      EEPROM.write(OutputPin3address, 00);
      EEPROM.commit();
    }
    delay(100);
    if (ServoSwitch_Default_Status == 1) {
      ServoSwitchOnOff(0);
      ServoSwitch_Default_Status = 0;
      EEPROM.write(ServoSwitch_Default_Status_address, 0);
      EEPROM.commit();
    }
    lcd.noBacklight();
    EEPROM.write(LCDBackLightaddress, 0);
    EEPROM.commit();
  } else if (s == 0) {
    if (OutputPin1_Prv_Status == 0) {
      digitalWrite(OutputPin1, LOW);
      client.publish("1s", "1");
      OutputPin1_Prv_Status = 1;
      OutputPin1_Current_Status = 0;
      EEPROM.write(OutputPin1address, 10);
      EEPROM.commit();
    }
    if (OutputPin3_Prv_Status == 1) {
      digitalWrite(OutputPin3, HIGH);
      client.publish("3s", "1");
      OutputPin3_Prv_Status = 0;
      OutputPin3_Current_Status = 1;
      EEPROM.write(OutputPin3address, 01);
      EEPROM.commit();
    }
    delay(100);
    ServoSwitchOnOff(1);
    ServoSwitch_Default_Status = 1;
    EEPROM.write(ServoSwitch_Default_Status_address, 1);
    EEPROM.commit();
    BedTime = 0;
    EEPROM.write(BedTime_Status_address, 0);
    EEPROM.commit();
    lcd.backlight();
    EEPROM.write(LCDBackLightaddress, 1);
    EEPROM.commit();
  } else if (s == 2) {
    ServoSwitchOnOff(0);
    ServoSwitch_Default_Status = 0;
    EEPROM.write(ServoSwitch_Default_Status_address, 0);
    EEPROM.commit();
    BedTime = 0;
    EEPROM.write(BedTime_Status_address, 0);
    EEPROM.commit();
    client.publish("5s", "0");
    lcd.backlight();
    EEPROM.write(LCDBackLightaddress, 1);
    EEPROM.commit();
  }
}

void allFanLightOnOff(uint8_t p) {
  if (p == 0) {
    if (OutputPin1_Current_Status == 0) {
      digitalWrite(OutputPin1, HIGH);
      client.publish("1s", "0");
      OutputPin1_Prv_Status = 0;
      OutputPin1_Current_Status = 1;
      EEPROM.write(OutputPin1address, 01);
      EEPROM.commit();

    } else {
      OutputPin1_Prv_Status = 1;
      OutputPin1_Current_Status = 1;
      EEPROM.write(OutputPin1address, 11);
      EEPROM.commit();
    }
    if (OutputPin2_Current_Status == 0) {
      digitalWrite(OutputPin2, HIGH);
      client.publish("2s", "0");
      OutputPin2_Prv_Status = 0;
      OutputPin2_Current_Status = 1;
      EEPROM.write(OutputPin2address, 01);
      EEPROM.commit();
    } else {
      OutputPin2_Prv_Status = 1;
      OutputPin2_Current_Status = 1;
      EEPROM.write(OutputPin2address, 11);
      EEPROM.commit();
    }

    if (OutputPin3_Current_Status == 1) {
      digitalWrite(OutputPin3, LOW);
      client.publish("3s", "0");
      OutputPin3_Prv_Status = 1;
      OutputPin3_Current_Status = 0;
      EEPROM.write(OutputPin3address, 10);
      EEPROM.commit();
    } else {
      OutputPin3_Prv_Status = 0;
      OutputPin3_Current_Status = 0;
      EEPROM.write(OutputPin3address, 00);
      EEPROM.commit();
    }
  } else {
    if (OutputPin1_Prv_Status == 0) {
      digitalWrite(OutputPin1, LOW);
      client.publish("1s", "1");
      OutputPin1_Prv_Status = 1;
      OutputPin1_Current_Status = 0;
      EEPROM.write(OutputPin1address, 10);
      EEPROM.commit();
    }
    if (OutputPin2_Prv_Status == 0) {
      digitalWrite(OutputPin2, LOW);
      client.publish("2s", "1");
      OutputPin2_Prv_Status = 1;
      OutputPin2_Current_Status = 0;
      EEPROM.write(OutputPin2address, 10);
      EEPROM.commit();
    }
    if (OutputPin3_Prv_Status == 1) {
      digitalWrite(OutputPin3, HIGH);
      client.publish("3s", "1");
      OutputPin3_Prv_Status = 0;
      OutputPin3_Current_Status = 1;
      EEPROM.write(OutputPin3address, 01);
      EEPROM.commit();
    }
  }
}
void PhysicalSwitchHandling() {
  if ((digitalRead(InputPin1) == LOW) && (InputPin1_prv_Status == 1) && ((unsigned long)(millis() - Prev_Btn_Prs_T) >= Btn_db_drtn)) {
    if (verifying_Button_Press(InputPin1, 0) == 1) {
      if (OutputPin1_Current_Status == 0) {
        digitalWrite(OutputPin1, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH1
        client.publish("1s", "0");
        OutputPin1_Current_Status = 1;  // pin status is opposite to relay status
        OutputPin1_Prv_Status = 0;
        EEPROM.write(OutputPin1address, 01);  //1st bit for previous status and 2nd bit for current status
        EEPROM.commit();
        // mySerial.write(11);
      } else {
        digitalWrite(OutputPin1, LOW);
        client.publish("1s", "1");
        OutputPin1_Current_Status = 0;
        OutputPin1_Prv_Status = 1;
        EEPROM.write(OutputPin1address, 10);
        EEPROM.commit();
        // mySerial.write(10);
      }
      InputPin1_prv_Status = 0;
    }
    Prev_Btn_Prs_T = millis();
  } else if ((InputPin1_prv_Status == 0) && (digitalRead(InputPin1) == HIGH) && ((unsigned long)(millis() - Prev_Btn_Prs_T) >= Btn_db_drtn)) {
    InputPin1_prv_Status = 1;
  } else if ((digitalRead(InputPin2) == HIGH) && (InputPin2_prv_Status == 0) && ((unsigned long)(millis() - Prev_Btn_Prs_T) >= Btn_db_drtn)) {
    if (verifying_Button_Press(InputPin2, 1) == 1) {
      if (OutputPin2_Current_Status == 0) {
        digitalWrite(OutputPin2, HIGH);  //To turn off (Relay state-OFF) Sazal_rController1_CH2
        client.publish("2s", "0");
        OutputPin2_Current_Status = 1;  // pin status is opposite to relay status
        OutputPin2_Prv_Status = 0;
        EEPROM.write(OutputPin2address, 01);
        EEPROM.commit();
      } else {
        digitalWrite(OutputPin2, LOW);
        client.publish("2s", "1");
        OutputPin2_Current_Status = 0;
        OutputPin2_Prv_Status = 1;
        EEPROM.write(OutputPin2address, 10);
        EEPROM.commit();
      }
      InputPin2_prv_Status = 1;
    }
    Prev_Btn_Prs_T = millis();
  } else if ((InputPin2_prv_Status == 1) && (digitalRead(InputPin2) == LOW) && ((unsigned long)(millis() - Prev_Btn_Prs_T) >= Btn_db_drtn)) {
    InputPin2_prv_Status = 0;
  } else if ((digitalRead(InputPin3) == LOW) && (InputPin3_prv_Status == 1) && ((unsigned long)(millis() - Prev_Btn_Prs_T) >= Btn_db_drtn)) {
    if (verifying_Button_Press(InputPin3, 0) == 1) {
      if (OutputPin3_Current_Status == 0) {
        delay(100);
        if (ServoSwitch_Status == 0) { ServoSwitchOnOff(1); }
        delay(100);
        digitalWrite(OutputPin3, HIGH);  //To turn on (Relay state-OFF) Sazal_rController1_CH3
        client.publish("3s", "1");
        OutputPin3_Current_Status = 1;  // pin status is same as relay status (normally close connection)
        OutputPin3_Prv_Status = 0;
        EEPROM.write(OutputPin3address, 01);
        EEPROM.commit();
        Manual_On = 1;
      } else {
        delay(100);
        if (ServoSwitch_Default_Status == 0) { ServoSwitchOnOff(0); }
        delay(100);
        digitalWrite(OutputPin3, LOW);
        client.publish("3s", "0");
        OutputPin3_Current_Status = 0;
        OutputPin3_Prv_Status = 1;
        EEPROM.write(OutputPin3address, 10);
        EEPROM.commit();
        Manual_On = 0;
      }
      InputPin3_prv_Status = 0;
    }
    Prev_Btn_Prs_T = millis();
  } else if ((InputPin3_prv_Status == 0) && (digitalRead(InputPin3) == HIGH) && ((unsigned long)(millis() - Prev_Btn_Prs_T) >= Btn_db_drtn)) {
    InputPin3_prv_Status = 1;
  }
}

bool Connect() {
  // Connect to MQTT Server and subscribe to the topic
  if (client.connect(clientID, mqtt_username, mqtt_password)) {
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH1);
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH2);
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH3);   //
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH4);   //servo switch
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH5);   //bedtime routine
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH8);   //servoswitch calibration
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH11);  //Fanregulator channel
    client.subscribe(mqtt_sub_topic_Sazal_rController1_CH12);  //Update all command channel
    return true;
  } else {
    return false;
  }
}

void updatingTime() {
  uint8_t pH = 0, pM = 0, breakLoop = 0, j = 0, i = 0;

  if (cHours != prvHours) {
    if (cHours < 13) {
      if (cHours == 0) {
        pH = 12;
      } else {
        pH = cHours;
      }
    } else {
      pH = cHours - 12;
    }
    lcd.setCursor(4, 0);
    if (pH < 10) {
      lcd.print("0");
      lcd.setCursor(5, 0);
      lcd.print(pH);
    } else {
      lcd.print(pH);
    }
    if (cHours < 12) {
      prvAMPM = cAMPM;
      cAMPM = 0;
    } else {
      prvAMPM = cAMPM;
      cAMPM = 1;
    }
    if (cAMPM != prvAMPM) {
      lcd.setCursor(9, 0);
      if (cAMPM == 0) {
        lcd.print("AM");
      } else {
        lcd.print("PM");
      }
    }
  }
  lcd.setCursor(7, 0);
  pM = timeClient.getMinutes();
  if (pM < 10) {
    lcd.print(0);
    lcd.setCursor(8, 0);
    lcd.print(pM);
  } else {
    lcd.print(pM);
  }

  prvDaySN = cDaySN;
  cDaySN = timeClient.getDay();
  if (cDaySN != prvDaySN) {
    lcd.setCursor(13, 0);
    lcd.print(daysOfTheWeek[timeClient.getDay()]);
  }
  prvED = cED;
  cED = timeClient.getEpochTime() / 86400L - 18261;
  if (cED != prvED) {
    rED = cED;
    while (breakLoop == 0) {
      for (j = 0; j < 12; j++) {
        if ((j == 0) && ((i == 0) || (i % 4 == 0))) {
          rED = rED - NoOfDaysInEachMonth[j];
          if (rED <= ((NoOfDaysInEachMonth[j + 1] + 1))) {
            breakLoop = 1;
            break;
          }
        } else if ((j == 1) && ((i == 0) || (i % 4 == 0))) {
          rED = rED - NoOfDaysInEachMonth[j] - 1;
          if (rED <= (NoOfDaysInEachMonth[j + 1])) {
            breakLoop = 1;
            break;
          }
        } else {
          rED = rED - NoOfDaysInEachMonth[j];
          if (rED <= (NoOfDaysInEachMonth[j + 1])) {
            breakLoop = 1;
            break;
          }
        }
      }
      if (breakLoop == 0) { i++; }
    }
    if (j == 11) {
      M = 1;
      Y = 2020 + i + 1;
    } else {
      M = j + 2;
      Y = 2020 + i;
    }
    D = rED;
    lcd.setCursor(2, 1);
    if (D < 10) {
      lcd.print("0");
      lcd.setCursor(3, 1);
      lcd.print(D);
    } else {
      lcd.print(D);
    }
    lcd.setCursor(6, 1);
    if (M < 10) {
      lcd.print("0");
      lcd.setCursor(7, 1);
      lcd.print(M);
    } else {
      lcd.print(M);
    }
    lcd.setCursor(10, 1);
    lcd.print(monthsOfTheYear[M - 1]);
    lcd.setCursor(14, 1);
    lcd.print(Y);
  }
}
void updatingTempRHPrALT() {
  cRH = bme.readHumidity() - 0.5;
  cTemp = bme.readTemperature();
  cPr = bme.readPressure() / 100.0F;
  cALT = bme.readAltitude(SEALEVELPRESSURE_HPA);
  char RH[5] = "", Temp[5] = "", Pr[7] = "", ALT[5] = "";

  lcd.setCursor(3, 2);
  lcd.print(cTemp, 1);
  lcd.setCursor(14, 2);
  lcd.print(cRH, 1);
  if (ProrALT == 1) {
    lcd.setCursor(8, 3);
    lcd.print("P=");
    lcd.setCursor(10, 3);
    lcd.print(cPr, 2);
    lcd.setCursor(17, 3);
    lcd.print("hPa");
    ProrALT = 2;
  } else {
    lcd.setCursor(8, 3);
    lcd.print("ALT.=");
    lcd.setCursor(13, 3);
    lcd.print(cALT, 2);
    if (cALT > 10) {
      lcd.setCursor(18, 3);
      lcd.print(" m");
    } else {
      lcd.setCursor(17, 3);
      lcd.print("0 m");
    }

    ProrALT = 1;
  }

  dtostrf(cTemp, 5, 2, Temp);
  client.publish("SazalrTemp", Temp);
  dtostrf(cRH, 5, 2, RH);
  client.publish("SazalrRH", RH);
  dtostrf(cPr, 7, 2, Pr);
  client.publish("SazalrPr", Pr);
  dtostrf(cALT, 5, 2, ALT);
  client.publish("SazalrALT", ALT);
}

void UpdatingAllChannel() {
  if (OutputPin1_Current_Status == 0) {
    client.publish("1s", "1");
  } else {
    client.publish("1s", "0");
  }
  if (OutputPin2_Current_Status == 0) {
    client.publish("2s", "1");
  } else {
    client.publish("2s", "0");
  }
  if (OutputPin3_Current_Status == 0) {
    client.publish("3s", "0");
  } else {
    client.publish("3s", "1");
  }
  if (ServoSwitch_Status == 0) {
    client.publish("4s", "0");
    client.publish("8s", "8");
  } else {
    client.publish("4s", "1");
    client.publish("8s", "0");
  }

  if (BedTime == 0) {
    client.publish("5s", "0");
  } else {
    client.publish("5s", "1");
  }

  char ServoAngle[3] = "";

  dtostrf(PrvAngle[1] / 1.8, 3, 1, ServoAngle);
  client.publish("11s", ServoAngle);

  updatingTempRHPrALT();
  timeClient.update();
}

uint8_t verifying_Button_Press(uint8_t InputPin, uint8_t PS) {
  uint8_t i = 0, c = 0;
  for (i = 0; i < 20; i++) {
    if (digitalRead(InputPin) == PS) { c++; }
    delay(1);
  }
  if (c > 2) {
    return 1;
  } else {
    return 0;
  }
}
void ServoSwitchCalibration(byte* payload, unsigned int L) {
  uint8_t p = 1, c = 0;
  for (uint8_t i = L; i > 0; i--) {
    if (i != L) { p = p * 10; }
    c = c + (((char)payload[i - 1]) * 1 - 48) * p;  //as ASCII code of "0" is 48 and "1" is 49
  }
  RotatingServoHorn(c, ServoSwitchPin, 0);
}

void FanRegulatorPositionChange(byte* payload, unsigned int L) {
  uint8_t p = 1, c = 0, mc;
  for (uint8_t i = L; i > 0; i--) {
    if (i != L) { p = p * 10; }
    c = c + (((char)payload[i - 1]) * 1 - 48) * p;  //as ASCII code of "0" is 48 and "1" is 49
  }
  mc = c * 1.8;
  RotatingServoHorn(mc, FanRegulatorServoPin, 1);
  EEPROM.write(FanRegulatoraddress, mc);
  EEPROM.commit();
}

void RotatingServoHorn(uint8_t CurrentAngle, uint8_t ServoPin, uint8_t ServoNo) {

  if (CurrentAngle > PrvAngle[ServoNo]) {
    for (uint8_t i = PrvAngle[ServoNo]; i <= CurrentAngle; i++) {
      AllServo[ServoNo].attach(ServoPin, Min_Pulse, Max_Pulse);
      AllServo[ServoNo].write(i);
      delay(30);
      //AllServo[ServoNo].detach();
      //delay(25);
    }
  } else {
    for (int j = PrvAngle[ServoNo]; j >= CurrentAngle; j--) {
      AllServo[ServoNo].attach(ServoPin, Min_Pulse, Max_Pulse);
      AllServo[ServoNo].write(j);
      delay(30);
      //AllServo[ServoNo].detach();
      //delay(25);
    }
  }
  AllServo[ServoNo].detach();
  PrvAngle[ServoNo] = CurrentAngle;
}

void RL_Charging_cycle()  ///"Charging_Status==0" => not charged yet, "Charging_Status==1" => charging ongoing, "Charging_Status==2" => Charging Completed,
{
  if (cHours > 18) {
    if (Charging_Status == 0) {
      if (Manual_On == 0) {
        digitalWrite(OutputPin3, HIGH);  //To turn on (Relay state-OFF) Sazal_rController1_CH3
        client.publish("3s", "1");
        OutputPin3_Current_Status = 1;  // pin status is same as relay status (normally close connection)
        OutputPin3_Prv_Status = 0;
        EEPROM.write(OutputPin3address, 01);
        EEPROM.commit();
      }
      Charging_Start_t[0] = cHours;
      Charging_Start_t[1] = timeClient.getMinutes();
      Charging_Status = 1;
    } else if (Charging_Status == 1) {
      if (((cHours * 60 + timeClient.getMinutes()) - (Charging_Start_t[0] * 60 + Charging_Start_t[1])) > 60) {
        if ((Manual_On == 0) && (OutputPin3_Current_Status == 1)) {
          digitalWrite(OutputPin3, LOW);
          client.publish("3s", "0");
          OutputPin3_Current_Status = 0;
          OutputPin3_Prv_Status = 1;
          EEPROM.write(OutputPin3address, 10);
          EEPROM.commit();
        }
        Charging_Status = 2;
      }
    }
  } else {
    if (Charging_Status > 0) {
      Charging_Status = 0;
      if ((Manual_On == 0) && (OutputPin3_Current_Status == 1)) {
        digitalWrite(OutputPin3, LOW);
        client.publish("3s", "0");
        OutputPin3_Current_Status = 0;
        OutputPin3_Prv_Status = 1;
        EEPROM.write(OutputPin3address, 10);
        EEPROM.commit();
      }
    }
  }
}

