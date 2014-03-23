#include <AESLib.h>
#include <CRC16.h>
#include "structs.h"

#define NODE_ID   1234

/** 
 * Sensor
 *
 * The sensor is in charge of gathering environmental data, specifically,
 * motion and lumosity, and reporting to the controller whether or not
 * the state of the room should be changed.
 *
 * The sensor is more aggressive with locking the room than it is with
 * turning the lights off. Currently, the sensor will send a "lock door"
 * request after 15 seconds of no movement, and a "turn off lights" requeset
 * after 120 seconds of no movement. These are arbitrarily set and can be
 * tweaked based on the requirements of the room.
 *
 * Acknowledgements:
 * - The AES library used can be found at https://github.com/DavyLandman/AESlib
 * - The CRC16 library was written by Tim W. Shilling (www.ShillingSystems.com)
 *
 * Last Revision: March 23, 2014
 */

// Input pins 
const int motionOutput = 13;
const int motionInput = 4; 
const int lightInput = A5;

//key has to be 16 bytes
//this key should be set up both shared with controller
//in case of multiple sensors, controller may need to maintain
//multiple keys that maps to right sensor
const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

byte incoming[16];

long seq = 0;
int isMotion, brightness;
int lockState = 0, lightState = 0;
boolean receivedAck = false;
boolean sentMsg = false;

/**
* Sets up the pin mode and the serial port
*/
void setup()
{
    Serial.begin(9600);
    pinMode(motionInput, INPUT);
    pinMode(lightInput, INPUT); 
}


void motionDetected()
{
   digitalWrite(motionOutput, HIGH);
   delay(3000);
   digitalWrite(motionOutput, LOW);
   delay(1000);  
}

/**
* sendData sets up the message from the sensor to the controller
* by setting the request values, the sequence number, and encrypting
* the message.
* 
* @param lock    1 if we are locking the door, 0 otherwise
* @param light   1 if we are turning on the lights, 0 otherwise
*/
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
 * serialEvent is called when there is data available in the
 * buffer in-between runs of the loop function. In our case
 * this means there is a message for the sensor from the
 * controller. Handle it.
 */
void serialEvent()
{
   int count = 0;
   
   // Read in all the available bytes into a buffer
   while(Serial.available())
   {
      if(count >= 16)      return;
      
      incoming[count++] = Serial.read();
      delay(10);
   }
 
   // We only care if there is exactly 16 bytes of data which
   // constitutes a full message in our system.
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
}

/**
* The main event loop of the sensor node.
*/
void loop()
{
    //data has been sent but did not receive ack yet
    //go back to sendMessage and send it again
    if(sentMsg && !receivedAck)
       goto sendMessage;
       
    // Gather the environment data for this run of the loop
    isMotion = digitalRead(motionInput);
    brightness = analogRead(lightInput);
    brightness = map(brightness, 0, 1023, 0, 255);

    //reset send/ack flag
    receivedAck = false;
    sentMsg = false;
    
sendMessage:  
    if(isMotion){
       motionDetected();
       if(lockState == SIG_LOCK || lightState == SIG_LIGHTOFF) {
         //check brightness here
           sendData(SIG_UNLOCK, SIG_LIGHTON);
           sentMsg = true;
       }
    } 
    else 
    {
       if(lockState == SIG_UNLOCK || lightState == SIG_LIGHTON) {
         //check brightness here
           sendData(SIG_LOCK, SIG_LIGHTOFF);
           sentMsg = true; 
       }
    }
    
    //wait 3seconds for ack
    if(sentMsg)
        delay(3000);

}
