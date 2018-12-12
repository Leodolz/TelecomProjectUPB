#include <SoftwareSerial.h>

#include <Wire.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> 
#include <DHT.h>
#define DHTTYPE DHT22

#define  SLAVE_ADDRESS         0x29  //slave address, any number from 0x01 to 0x7F

#define  REG_MAP_SIZE            15

//#define  MAX_SENT_BYTES        50

 

/********* Global  Variables  ***********/

char registerMap[REG_MAP_SIZE];

volatile float humedad;
volatile float temperatura;
volatile int t,h;
const int DHTPin = 5;  
DHT dht(DHTPin, DHTTYPE);
void setup()

{

     Wire.begin(SLAVE_ADDRESS); 

     Wire.onRequest(requestEvent);

     Wire.onReceive(receiveEvent);
      dht.begin();
      Serial.begin(9600);
}

 

void loop()

{
    humedad = dht.readHumidity();
     temperatura = dht.readTemperature();
     t= temperatura*10;
     h= humedad*10;  
     delay(29000); 
    //IDLE Waiting for interrupt on Request event by master
}

 

void requestEvent()

{
     memset(registerMap,0,sizeof(registerMap));
     sprintf(registerMap,"%d %d",t,h);
     Wire.write(registerMap, strlen(registerMap));  //Sends data to ARM master
     Serial.print(registerMap);
}

 
// Reserved for debugging
void receiveEvent(int bytesReceived)

{
     
    Wire.read();

} 
