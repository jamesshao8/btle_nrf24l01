#include <SPI.h>
#include "RF24.h"
#include "BTLE.h"

RF24 radio(10,9); //10 for CE, 9 for CSN

BTLE btle(&radio);

void setup() {

  Serial.begin(9600);
  while (!Serial) { }
  Serial.println("BTLE advertisement receiver");

  btle.begin("");
}

void loop() {
    
  //Serial.print("Listening... ");
  
  if (btle.listen()) {
    Serial.println("Got payload: ");
    for (uint8_t i = 0; i < (btle.buffer.pl_size)-6; i++) 
    { 
      Serial.print(btle.buffer.payload[i],HEX); 
      Serial.print(" "); 
    }
    Serial.println("");
    int len =  btle.buffer.payload[3];
    if (len < 8)
    {
        Serial.println("Name: ");

      for (uint8_t j = 5; j < 5 + len - 1; j++) 
      { 
        char ch = btle.buffer.payload[j];
       Serial.print(ch);
       Serial.print(" "); 
      }
    }
    Serial.println("");

    int len_data =  btle.buffer.payload[5+len-1];
    if (len_data < 8)
    {
        Serial.println("Data: ");

      for (uint8_t j = 5 + len; j < 5 + len + len_data; j++) 
      { 
        Serial.print(btle.buffer.payload[j],HEX);
        Serial.print(" "); 
      }
    }

    Serial.println("");
    

  }

  
  btle.hopChannel();
}

