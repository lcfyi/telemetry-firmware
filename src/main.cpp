#include "Particle.h"
#include "Arduino.h"
#include "JsonMaker.h"
#include "Sensor_RSSI.h"
#include "Sensor_ECU.h"

#include "mcp_can.h"
#include "SparkFunMAX31855k.h"

#define BLINK_INTERVAL_OFF 1800
#define BLINK_INTERVAL_ON 200
#define PUBLISH_INTERVAL 10000

SYSTEM_THREAD(ENABLED);

void onSerialData();
JsonMaker json_maker;
Sensor_RSSI rssi;
Sensor_ECU ecu(&Serial1);

MCP_CAN can(A5,&SPI);
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
char msgString[128];

SparkFunMAX31855k thermo(&SPI1, D5);

uint32_t last_blink = 0; // Time of last blink
uint32_t last_publish = 0; // Time of last publish
boolean led_state = LOW;

Timer timer(1, onSerialData);

// Handler for any new message on ID "Proto"
void proto_response(const char *event, const char *data) {
    Serial.println("Received Message on ID: " + String(event) + " - Message Data: " + String(data));
}

void setup() {
    pinMode(D7, OUTPUT);

    ecu.begin();
    timer.start();

    // Subscribe to any new messages on ID "Proto"
    Particle.subscribe("hook-response/Proto", proto_response, MY_DEVICES);

    // Setup function only runs after Boron connected in (default) Automatic mode
    Serial.println("Particle Connected!");

    if(can.begin(MCP_ANY, CAN_500KBPS, MCP_8MHZ); 
        Serial.println("CAN Interface Activated"));
    else 
        Serial.println("CAN Interface Failed to Activate");

    can.setMode(MCP_NORMAL);
    pinMode(D6, INPUT);

    SPI.begin();
    SPI1.begin();
}

void readCAN(){
    can.readMsgBuf(&rxId, &len, rxBuf);      // Read data: len = data length, buf = data byte(s)
    
    if((rxId & 0x80000000) == 0x80000000)     // Determine if ID is standard (11 bits) or extended (29 bits)
        sprintf(msgString, "Extended ID: 0x%.8lX  DLC: %1d  Data:", (rxId & 0x1FFFFFFF), len);
    else
        sprintf(msgString, "Standard ID: 0x%.3lX       DLC: %1d  Data:", rxId, len);
  
    Serial.print(msgString);
  
    if((rxId & 0x40000000) == 0x40000000){    // Determine if message is a remote request frame.
      sprintf(msgString, " REMOTE REQUEST FRAME");
      Serial.print(msgString);
    } else {
      for(byte i = 0; i<len; i++){
        sprintf(msgString, " 0x%.2X", rxBuf[i]);
        Serial.print(msgString);
      }
    }
        
    Serial.println();

}
void loop() {

    if(!digitalRead(D6)){
        readCAN();
    }

    digitalWrite(D7, led_state);
    
    // Blink the LED
    if (led_state & (millis() - last_blink >= BLINK_INTERVAL_ON)){
        led_state = LOW;
        last_blink = millis();
    }else if(!led_state & (millis() - last_blink >= BLINK_INTERVAL_OFF)){
        led_state = HIGH;
        last_blink = millis();
    }

    //Publish a message to Proto
    if (millis() - last_publish >= PUBLISH_INTERVAL){
        Serial.println("Temperature: " + String(thermo.readTempC()) + "C");
        last_publish = millis();
        //Call makeJSON function
        json_maker.init();
        json_maker.add("RSSI", rssi.get());
        json_maker.add("PROTO-RPM", ecu.getRPM());
        json_maker.add("PROTO-SPARK", ecu.getSpark());
        Particle.publish("Proto", json_maker.get(), PRIVATE, WITH_ACK);
    }

}

void onSerialData()
{
    ecu.onSerialData();
}

