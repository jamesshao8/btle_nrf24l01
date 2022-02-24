#include <SPI.h>
#include "RF24.h"
#include "BTLE.h"


RF24 radio(10,9); //10 for CE, 9 for CSN
BTLE btle(&radio);

#define channel_number 6  //set the number of channels
#define sigPin 2  //set PPM signal output pin on the arduino
#define PPM_FrLen 27000  //set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen 400  //set the pulse length
int ppm[channel_number];


// The sizeof this struct should not exceed 32 bytes
struct MyData {
  byte roll;
  byte pitch;
  byte throttle;
  byte yaw;
};

MyData data;

void resetData() 
{
  // 'safe' values to use when no radio input is detected
  data.roll = 127;
  data.pitch = 127;
  data.throttle = 0;
  data.yaw = 127;
  
  setPPMValuesFromData();
}

void setPPMValuesFromData()
{
  ppm[0] = map(data.roll, 0, 255, 1000, 2000);
  ppm[1] = map(data.pitch,      0, 255, 1000, 2000);
  ppm[2] = map(data.throttle,    0, 255, 1000, 2000);
  ppm[3] = map(data.yaw,     0, 255, 1000, 2000);  
  ppm[4] = 1000;
  ppm[5] = 1000;
}

/**************************************************/

void setupPPM() 
{
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, 0);  //set the PPM signal pin to the default state (off)

  cli();
  TCCR1A = 0; // set entire TCCR1 register to 0
  TCCR1B = 0;

  OCR1A = 100;  // compare match register (not very important, sets the timeout for the first interrupt)
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);  // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
}

void setup() 
{
  resetData();
  setupPPM();
  //pinMode(2, OUTPUT);
  
  Serial.begin(9600);
  while (!Serial) { }
  Serial.println("BTLE advertisement receiver");

  btle.begin("");
}

void loop() 
{
    
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
      
      if ((btle.buffer.payload[5] == 'S') && 
      (btle.buffer.payload[6] == 'H') && 
      (btle.buffer.payload[7] == 'A') &&
      (btle.buffer.payload[8] == 'R') &&
      (btle.buffer.payload[9] == 'F') )
      {
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
          data.roll = btle.buffer.payload[5 + len];
          data.pitch = btle.buffer.payload[6 + len];
          data.throttle = btle.buffer.payload[7 + len];
          data.yaw = btle.buffer.payload[8 + len]; 
        }
      }
    }


    Serial.println("");
    

  }
  
  //btle.hopChannel();
  btle.setChannel(38);

  setPPMValuesFromData();

}

#define clockMultiplier 2 // set this to 2 if you are using a 16MHz arduino, leave as 1 for an 8MHz arduino

ISR(TIMER1_COMPA_vect)
{
  static boolean state = true;

  TCNT1 = 0;

  if ( state ) {
    //end pulse
    PORTD = PORTD & ~B00000100; // turn pin 2 off. Could also use: digitalWrite(sigPin,0)
    OCR1A = PPM_PulseLen * clockMultiplier;
    state = false;
  }
  else {
    //start pulse
    static byte cur_chan_numb;
    static unsigned int calc_rest;

    PORTD = PORTD | B00000100; // turn pin 2 on. Could also use: digitalWrite(sigPin,1)
    state = true;

    if(cur_chan_numb >= channel_number) {
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * clockMultiplier;
      calc_rest = 0;
    }
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * clockMultiplier;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
