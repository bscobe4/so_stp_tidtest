#include <Arduino.h>
#include <stdio.h>
//#include <HardwareSerial.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define HWSERIAL Serial2
#define RXBUFFERSIZE 12
#define ENDCHECKBYTE 12 //position in data that is checked for end byte
#define STARTMASKVALUE 255
#define STOPMASKVALUE 255

//static int rxmemory[RXBUFFERSIZE];
char rxbuffer[RXBUFFERSIZE];

volatile boolean startMaskFlag = 0; //True if start mask is not the correct 
volatile boolean stopMaskFlag = 0;
// put function declarations here:
//void pollTIDdata();
void requestTIDdata();

void setup() {
  
  Serial.begin(9600);
  HWSERIAL.begin(2400,SERIAL_8N1);
  //HWSERIAL.addMemoryForRead(rxmemory, RXBUFFERSIZE);

}

void loop() {
  
  //pollTIDdata();
  requestTIDdata();

}

// put function definitions here:



void requestTIDdata() {

  if (startMaskFlag == 1){
    
    Serial.print("StartMaskFlag ");
  }

  if (stopMaskFlag == 1){
    
    Serial.print("StopMaskFlag");
  }

  //int incomingByte;
 int i = 0;
 startMaskFlag = 0;
 stopMaskFlag = 0;
 //String datalist;
  Serial.println("");
  
  while (i < RXBUFFERSIZE){
    if(HWSERIAL.available()){
      //HWSERIAL.write(255);
      rxbuffer[i] = HWSERIAL.read();

      if (i == 0){
        if (rxbuffer[0] != STARTMASKVALUE){
          startMaskFlag = 1;
        }
      }

      else if (startMaskFlag == 1){
        if (rxbuffer[i] == STOPMASKVALUE){
          rxbuffer[RXBUFFERSIZE-1] = rxbuffer[i];
          i = RXBUFFERSIZE - 1;
        }
      }

      else if (i == (RXBUFFERSIZE-1)){
        if (rxbuffer[RXBUFFERSIZE-1] != STOPMASKVALUE){
          stopMaskFlag = 1;
          i--;
        }
      }

      Serial.print(rxbuffer[i], HEX);
      Serial.print(" ");
      i++;
    }
  }

  
  

  

  
}

/*void pollTIDdata() {

 //int incomingByte;
 int i = 0;
 //String datalist;

  while (i < 10){
    if (HWSERIAL.available() > 0) {
      //incomingByte = HWSERIAL.read();
      //Serial.print(" ");
      //Serial.print((incomingByte), HEX);
      rxbuffer[i] = HWSERIAL.read();
      i++;
    }
    
  }

  i = 0;
  
  while (i < 9){

    Serial.print(rxbuffer[i], HEX);
    Serial.print(" ");
    i++;

  }
  Serial.println("");

}*/