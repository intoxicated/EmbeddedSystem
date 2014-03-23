
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

byte incoming[16];

//input pins 
const int motionOutput = 13;
const int motionInput = 4; 
const int lightInput = A5;

long seq = 0;
int isMotion, brightness;
int lockState = 0, lightState = 0;
boolean receivedAck = false;
boolean sentMsg = false;

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
    data.checksum = CRC::CRC16((uint8_t *)&data, 8);
    memset(data.padding, 0x20, 6);
    //encrypt
    aes128_enc_single(key, &data);
    //send
    Serial.write((byte *)&data, 16);
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
      if(count >= 16)
          return;
      incoming[count++] = Serial.read();
      delay(10);
   }
 
   //only care if 16 bytes of data
   if(count == 16)
   {
      aes128_dec_single(key, incoming);
      res_msg * data = (res_msg *)incoming;
      //response ok!
      if(data->response == RES_OK)
      {
         lockState = data->lockState;
         lightState = data->lightState;
      }
      else if(data->response != RES_OK) 
      {
         //errors
      }
      receivedAck = true;
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

    //reset send/ack flag
    receivedAck = false;
    sentMsg = false;
    
sendMessage:  
    if(isMotion){
       motionHasDetected();
       if(lockState == SIG_LOCK || lightState == SIG_LIGHTOFF){
         //check brightness here
           sendData(SIG_UNLOCK, SIG_LIGHTON);
           sentMsg = true;
       }
    } 
    else 
    {
       if(lockState == SIG_UNLOCK || lightState == SIG_LIGHTON){
         //check brightness here
           sendData(SIG_LOCK, SIG_LIGHTOFF);
           sentMsg = true; 
       }
    }
    
    //wait 3seconds for ack
    if(sentMsg)
        delay(3000);
    
    //data has been sent but did not receive ack yet
    //go back to sendMessage and send it again
    if(sentMsg && !receivedAck)
       goto sendMessage;
}
