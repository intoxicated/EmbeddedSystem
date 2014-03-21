/**
 * Controller -- lock and light
 *
 */

#include <AESLib.h>

#define RES_UNLOCK   1
#define RES_LOCK     2
#define RES_LIGHTON  4
#define RES_LIGHTOFF 8

#define RES_OK       0
#define RES_ERR      1

#define CTRL_ID   4321

const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

byte outgoing[16];
byte incoming[16];

const int doorLockLED = 12;
const int doorUnlockLED = 11;
const int lightRedLED = 10;
const int lightGreenLED = 9;
const int lightBlueLED = 8;

int changeLock = 0, changeLight = 0;
int lockState = 0, lightState = 0;
int err;

void setup()
{
   Serial.begin(9600);
   
   pinMode(doorLockLED, OUTPUT);
   pinMode(doorUnlockLED, OUTPUT);
   pinMode(lightRedLED, OUTPUT);
   pinMode(lightGreenLED, OUTPUT);
   pinMode(lightBlueLED, OUTPUT);  
}

void sendResponse(int response, int lockState, int lightState)
{
    memset(outgoing, 0x0, 16);
    outgoing[0] = CTRL_ID;
    outgoing[1] = 0xFF; //reserve
    outgoing[2] = response;
    outgoing[3] = lockState;
    outgoing[4] = lightState;
    outgoing[5] = 0xFF; //checkSum
    memset(outgoing+6, 0x80, 11); 
    
    aes128_enc_single(key, outgoing);
    Serial.write(outgoing, 16);
}

int readRequest()
{
   int count = 0;
   while(Serial.available())
   {
      incoming[count++] = Serial.read();
      delay(10);
   }
   
   return 0;
}

void loop()
{
   if(Serial.available())
   {
      err = readRequest();
      if(!err)
      {
      
      
      }  
   }
}
