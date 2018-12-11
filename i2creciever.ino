#include <Wire.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h> 
#include <DHT.h>
#define DHTTYPE DHT22

#define  SLAVE_ADDRESS         0x29  //slave address, any number from 0x01 to 0x7F

#define  REG_MAP_SIZE            10

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

//     Wire.onReceive(receiveEvent);
      dht.begin();
}

 

void loop()

{
    //IDLE Waiting for interrupt on Request event by master
}

 

void requestEvent()

{
     humedad = dht.readHumidity();
     temperatura = dht.readTemperature();
     t= temperatura*10;
     h= humedad*10;   
     sprintf(registerMap,"%d %d",t,h);
     Wire.write(registerMap, REG_MAP_SIZE);  //Sends data to ARM master
}

 
/* Reserved for debugging
void receiveEvent(int bytesReceived)

{
      recibido = true;
     for (int a = 0; a < bytesReceived; a++)

     {

          if ( a < MAX_SENT_BYTES)

          {
               receivedCommands[a] = Wire.read();
          }

          else

          {

               Wire.read();  // if we receive more data then allowed just throw it away

          }

     }

} */
