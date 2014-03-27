#include <AESLib.h>
#include <CRC16.h>
#include "structs.h"

#define CTRL_ID   1
#define MAX_NODES 10

/**
 * Controller
 *
 * The controller is in charge of responding to requests from sensor nodes,
 * sanity checking their requests and carrying out the requests if they are
 * valid. It then reports back to the sensor that the request was performed.
 *
 * Acknowledgements:
 * - The AES library used can be found at https://github.com/DavyLandman/AESlib
 * - The CRC16 library was written by Tim W. Shilling (www.ShillingSystems.com)
 *
 * Last Revision: March 23, 2014
 */

// Input pins
const int lightOnLED = 8;
const int lightOffLED = 9;
const int doorUnlockLED = 10;
const int doorLockLED = 11;
const int message = 12;
const int droppingMessage = 13;

/*
* A key has to be 16 bytes. This key needs to be identical to the
* key coded into the controller. In the case of multiple sensors, it may
* be advisable to have multiple keys and thus the controller will need
* to maintain a mapping of sensors -> keys.
*/
const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

/*
* Each node throughout its lifetime keeps track of a sequence number for message
* that it sends. This sequence number is used to prevent replay attacks, and confirm
* that the controller received and performed the actions requested.
*/
long nodeSeqNumber[MAX_NODES];

/*
* We need to keep track of the current state of the room. We probably need to
* incorporate a protocol to keep this from getting out of sync
*/
int lockState = 0;
int lightState = 0;

// Function prototypes
void processMessage();
void performRequest(data_msg *);
int checkErrors(data_msg *);
void sendResponse(int, int, int);


/**
* Sets up the pin mode and the serial port
*/
void setup()
{
   Serial.begin(9600);
   
   // Setup the input pins
   pinMode(doorLockLED, OUTPUT);
   pinMode(doorUnlockLED, OUTPUT);
   pinMode(lightOffLED, OUTPUT);
   pinMode(lightOnLED, OUTPUT);
   pinMode(message, OUTPUT);
   pinMode(droppingMessage, OUTPUT);
   
   // Setup the sequence number tracking
   for (int i = 0; i < MAX_NODES; i++)
   {
     nodeSeqNumber[i] = 0;
   }
   
   /*** DEBUG ***/
   digitalWrite(doorUnlockLED, HIGH);
   delay(500);
   digitalWrite(doorUnlockLED, LOW);
   digitalWrite(doorLockLED, HIGH);
   delay(500);
   digitalWrite(doorLockLED, LOW);
   digitalWrite(lightOffLED, HIGH);
   delay(500);
   digitalWrite(lightOffLED, LOW);
   digitalWrite(lightOnLED, HIGH);
   delay(500);
   digitalWrite(lightOnLED, LOW);
   digitalWrite(message, HIGH);
   delay(500);
   digitalWrite(message, LOW);
   digitalWrite(droppingMessage, HIGH);
   delay(500);
   digitalWrite(droppingMessage, LOW);
   
   /*** DEBUG ***/
   
   // For our prototype the lock and lights start locked and off, respectively
   lockState = SIG_LOCK;
   digitalWrite(doorLockLED, HIGH);
   lightState = SIG_LIGHTOFF;
   digitalWrite(lightOffLED, HIGH);
}


/**
* The main event loop of the controller node. Simply continue to loop
* until there is data available.
*/
void loop()
{
   if(Serial.available())
   {
      processMessage();
   }
}


/**
* Reads in and processes a message from the sensor node
*
* Steps:
*   1) Read the data into the buffer to reconstruct the message
*   2) Decrypt and Verify
*   3) Perform the request
*   4) Respond to the sensor.
*/
void processMessage()
{
   int error;
   int count = 0;
   byte incoming[16];
   
   // Read all available data
   while(Serial.available())
   {
      // It is an error if available data is longer than 16 bytes
      if(count >= 16)
      {
        Serial.println("Message too big.");
        return;
      }
      incoming[count++] = Serial.read();
      delay(5);
   }

   // It is an error if collected data is shorter than 16 bytes
   if(count != 16)
   {
     Serial.println("Message too small.");
     return;
   }
   
   // Decrypt and check the message for errors;
   aes128_dec_single(key,incoming); 
   data_msg * data = (data_msg *) incoming;
   if((error = checkErrors(data)) != NO_ERR)
   {
       digitalWrite(droppingMessage, HIGH);
       delay(500);
       digitalWrite(droppingMessage, LOW);
       //TODO: sendError(error);
       return;
   }
   
   /*** DEBUG ****/
   digitalWrite(message, HIGH);
   delay(100);
   digitalWrite(message, LOW);
   /**** DEBUG ****/
   
  performRequest(data);
  //sendResponse(data->id, NO_ERR, lockState, lightState);
  nodeSeqNumber[data->id]++;
}


/**
* Performs the requested action from the sensor node
*
* @param data  A pointer to the message struct received from the sensor
* @return  The action return code
*/
void performRequest(data_msg * data)
{
   // Check and see if we have somehow gotten out of sync, and fix it.
   if(data->lockReq == lockState && data->lightReq == lightState)
   {
     Serial.print("Request is same state as system.");
     //sendSyncSyncMessage
     return;
   }
   
   // Handle a request to change the state of the door lock
   if (lockState == SIG_LOCK && data->lockReq == SIG_UNLOCK)
   {
     digitalWrite(doorLockLED, LOW);
     delay(500);
     digitalWrite(doorUnlockLED, HIGH);
     lockState = SIG_UNLOCK;
   }
   else if (lockState == SIG_UNLOCK && data->lockReq == SIG_LOCK)
   {
     digitalWrite(doorUnlockLED, LOW);
     delay(500);
     digitalWrite(doorLockLED, HIGH);
     lockState = SIG_LOCK;
   }
   
   // Handle a request to change the state of the lights
   if (lightState == SIG_LIGHTOFF && data->lightReq == SIG_LIGHTON)
   {
     digitalWrite(lightOffLED, LOW);
     delay(500);
     digitalWrite(lightOnLED, HIGH);
     lightState = SIG_LIGHTON;
   }
   else if (lightState == SIG_LIGHTON && data->lightReq == SIG_LIGHTOFF)
   {
     digitalWrite(lightOnLED, LOW);
     delay(500);
     digitalWrite(lightOffLED, HIGH);
     lightState = SIG_LIGHTOFF;
   }
   
}


/**
* Sends a response back to a sensor node based on the results of the
* action requested.
*
* @param response    The response (success or failure)
* @param lockState   The lock state after the requested action
* @param lightState  The light state after the requested action
*/
void sendResponse(int sensorID, int response, int lockState, int lightState)
{
    res_msg msg;
    memset(&msg, 0x0, 16);
    msg.id = sensorID;
    msg.ack = nodeSeqNumber[sensorID];
    msg.reserve = 0xFF;
    msg.response = response;
    msg.lockState = lockState;
    msg.lightState = lightState;
    msg.checksum = CRC::CRC16((uint8_t *)&msg, 9);
    memset(msg.padding, 0x20, 6); 
    
    aes128_enc_single(key, &msg);
    Serial.write((byte *)&msg, 16);
}


/**
* Check and see if we have a properly formed, and legitimate request.
*
* A request is valid and legitimate if:
*    1) The checksum is correct
*    2) The light and/or lock request are both valid
*    3) The sequence number matches the next expected sequence number for the requesting
*          node.
*/
int checkErrors(data_msg * data)
{
   // Check that the checksum is correct
   if(CRC::CRC16((uint8_t*)data, 8) != data->checksum)
   {
     Serial.println("Checksum invalid");
     return CHKSUM_ERR;
   }
   
   // Check that the light/lock requests are valid
   if(data->lockReq != SIG_LOCK && data->lockReq != SIG_UNLOCK &&
       data->lockReq != SIG_NULL)
   { 
     Serial.println("Bogus lock request."); 
     return LOCK_ERR;
   }
   if(data->lightReq != SIG_LIGHTON && data->lightReq != SIG_LIGHTOFF &&
       data->lightReq != SIG_NULL)
   {
     Serial.println("Bogus light request.");  
     return LIGHT_ERR;
   }    

  // Check that the sequence numbers are valid
  if (data->sequence != nodeSeqNumber[data->id])
  {
    Serial.println("Incorrect sequence number.");
    Serial.print("Expected: ");
    Serial.println(nodeSeqNumber[data->id]);
    Serial.print("Received: ");
    Serial.println(data->sequence);
    
    return SEQ_ERR;
  }
  
  return NO_ERR;
}

