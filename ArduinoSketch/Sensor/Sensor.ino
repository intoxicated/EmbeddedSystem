#include <AESLib.h>
#include <CRC16.h>
#include "structs.h"

#define NODE_ID   12

/** 
 * Sensor
 *
 * The sensor is in charge of gathering environmental data, specifically,
 * motion and lumosity, and reporting to the controller whether or not
 * the state of the room should be changed.
 *
 * The sensor shoulw be more aggressive with locking the room than it is with
 * turning the lights off. The room being left in an vulnerable state is a
 * far worse scenario than the lights being left on fex extra minutes.
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
const int lightTriggerPoint = 350;

/*
* A key has to be 16 bytes. This key needs to be identical to the
* key coded into the controller. In the case of multiple sensors, it may
* be advisable to have multiple keys and thus the controller will need
* to maintain a mapping of sensors -> keys.
*/
const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

byte incoming[16];

long seq = 1;
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


/**
* sendData sets up the message from the sensor to the controller
* by setting the request values, the sequence number, and encrypting
* the message.
* 
* @param lock    Whether or not the door should be unlocked/locked.
* @param light   Whether or not the lights should be turned on/off.
*/
void sendData(int lock, int light)
{
    data_msg data;
    memset(&data,0x0,16);
    
    // Setup the message struct
    data.id = NODE_ID;
    data.sequence = seq++;
    data.reserve = 0xFF;
    data.lockReq = lock;
    data.lightReq = light;
    data.checksum = CRC::CRC16((uint8_t *)&data, 8);
    memset(data.padding, 0x20, 6);
    
    // Encrypt and Send
    aes128_enc_single(key, &data);
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
* The main event loop of the sensor node. Currently, this just runs
* our set of hardcoded environment changes.
*/
void loop()
{
  executeTests();
}


/**
* A set of "fabricated" changes to the environment that could occur in the wild
* in order to test the behavior of our controller and the effectiveness of
* our protocol.
*/
void executeTests()
{
  data_msg test;

  // Test 1: Send a lights on message.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = SIG_NULL;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6); 
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);
  
  // Test 2: Send a lights off message.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = SIG_NULL;
  test.lightReq = SIG_LIGHTOFF;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);
 
  // Test 3: Send a unlock on message
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_NULL;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);  
  
  // Test 4: Send a lock message
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = SIG_LOCK;
  test.lightReq = SIG_NULL;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);  
  
  // Test 5: Send a valid message encrypted with invalid key
  // Controller shouldn't do anything since decrypted value 
  // is garbage
  byte fakekey [] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(fakekey, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);
  
  // Test 6: Send invalid value of light/lock 
  // Controller shouldn't do anything
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = 15;
  test.lightReq = 84;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
    
  delay(5000);
  
  // Test 7: Demostrating bit flip or bit loss 
  // Controller shouldn't do anything since checksum is not match
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq++;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  test.lightReq = SIG_LIGHTOFF;
  
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);
  
  // Test 8: Testing replay, Controller should ignore it since
  //         sequence number is equal or less to previous one
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seq-1;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(fakekey, &test);
  Serial.write((byte *)&test, 16);
  
  delay(5000);
}


/**
* Pseudocode that indicates what a production environment would look like. This
* is the base code that we will start with when we start working to add the
* timing/interrupts to the code
*
* In this section, we assume that we are tracking timeSinceLastMovement.
*/
void productionCodeSample()
{
    // In this case, the sensor hasn't received an ack yet. Resend the message.
    if(sentMsg && !receivedAck)
       //Resend last message
       
    // Gather the environment data for this run of the loop
    isMotion = digitalRead(motionInput);
    brightness = analogRead(lightInput);
    brightness = map(brightness, 0, 1023, 0, 255);

    //reset send/ack flag
    receivedAck = false;
    sentMsg = false;
    
    // If there is motion, unlock the door and/or turn on the lights
    if(isMotion){
       if(lockState == SIG_LOCK || lightState == SIG_LIGHTOFF) {
           if(brightness <= lightTriggerPoint)
               sendData(SIG_UNLOCK, SIG_LIGHTON);
           else
               sendData(SIG_UNLOCK, SIG_NULL);
           sentMsg = true;
       }
    }
    else 
    {
       // Check if we have surpassed the timeSinceLastMovement lock threshold
       // Check if we have surpassed the timeSinceLsatMovement light threshold
       
       // Send a message appropriately based on the above two conditions
       // -OR-
       // Update the variable keeping track of the timeSinceLastMovement
    }
 
    
    // Wait a few seconds to allow the controller to respond if we sent a message
    // or to prevent the system from needlessly checking as fast as it can since
    // life tends not to happen in milliseconds!
    delay(3000);
}
