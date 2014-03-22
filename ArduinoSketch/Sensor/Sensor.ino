

/** 
 * Sensor sketch  
 *
 */

//ACElib ref:https://github.com/DavyLandman/AESLib
#include <AESLib.h>
#include <structs.h>
#include <CRC16.h>

#define NODE_ID   1234

//key has to be 16 bytes
//this key should be set up both shared with controller
//in case of multiple sensors, controller may need to maintain
//multiple keys that maps to right sensor
const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

byte outgoing[16];
byte incoming[16];

//input pins 
const int motionOutput = 13;
const int motionInput = 4; 
const int lightInput = A5;

long seq = 0;
int isMotion, brightness;
int lockState = 0, lightState = 0;

void setup()
{
    Serial.begin(9600);
    pinMode(motionInput, INPUT);
    pinMode(lightInput, INPUT); 
}

void motionHasDetected()
{
   digitalWrite(motionOutput, HIGH);
   delay(3000);
   digitalWrite(motionOutput, LOW);
   delay(1000);  
}

void sendData(int lock, int light)
{
    data_msg data;
    memset(&data,0x0,16);
    data.id = NODE_ID;
    data.sequence = seq++;
    data.reserve = 0xFF;
    data.lockReq = lock;
    data.lightReq = light;
    data.checksum = 0xFF;
    memset(data.padding, 0x20, 6);
    //encrypt
    aes128_enc_single(key, outgoing);
    //send
    Serial.write(outgoing, 16);
}

/**
 * get called when new data arrive from controller
 * adjust state of light and lock if necessary
 *
 */
void serialEvent()
{
   int count = 0;
   while(Serial.available())
   {
      incoming[count++] = Serial.read();
      delay(10);
   }
   //add nullterminate just in case
   incoming[16] = '\0';
   
   //only care if 16 bytes of data
   if(count == 16)
   {
      aes128_dec_single(key, incoming);
      //response ok!
      if(incoming[0] == 0)
      {
         lockState = incoming[1];
         lightState = incoming[2]; 
      }
      else if(incoming[0] == 1) //error ??
      {
         //???
      }
      else //ignore all others
        ;
   }
   else //if not 16 bytes, ignore
     ;
}

void loop()
{
    //receive signal from motion sensor
    isMotion = digitalRead(motionInput);
    
    //read brightness
    brightness = analogRead(lightInput);
    
    //brightness = constrain(brightness, 970, 1023);
    brightness = map(brightness, 0, 1023, 0, 255);
  
    if(isMotion){
       //turn on the light and unlock
       //clear out
       if(lockState != 1 || lightState != 1)
           sendData(SIG_UNLOCK, SIG_LIGHTON);
    } 

    //some constant to determine light is on
    else 
    {
       //turn off the light and lock
       if(lockState != 0 || lightState != 0)
          sendData(SIG_LOCK, SIG_LIGHTOFF);
    }


}
