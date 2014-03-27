#include <AESLib.h>
#include <HashMap.h>
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
HashType<int, long> nodeArray[MAX_NODES];
HashMap<int, long> nodeMap = HashMap<int, long>(nodeArray, MAX_NODES);
int nodeCount;

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
   
   pinMode(doorLockLED, OUTPUT);
   pinMode(doorUnlockLED, OUTPUT);
   pinMode(lightOffLED, OUTPUT);
   pinMode(lightOnLED, OUTPUT);
   pinMode(message, OUTPUT);
   
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
      Serial.println("Message for us!");
      processMessage();
   }
}


/**
* Reads in and processes a message from the sensor node
*/
void processMessage()
{
   int error;
   int count = 0;
   byte incoming[16];
   
   memset(incoming, 0x00, 16);
   
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
       //sendError(error);
       return;
   }
   
   /*
   // If this is a new node, add it to our list of known nodes. Otherwise
   // update the sequence number for the given node.  
   if(data->sequence == 1 && nodeCount < 10) {
       nodeMap[nodeCount++](data->id, data->sequence);
   }
   else {
     nodeMap.setValueOf(data->id, data->sequence);
   }
   */
   
   /*** DEBUG ****/
   digitalWrite(message, HIGH);
   delay(100);
   digitalWrite(message, LOW);
   /**** DEBUG ****/
   
  performRequest(data);
  //sendResponse(data->seqNumber, NO_ERR, lockState, lightState);
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
   }
   else if (lockState == SIG_UNLOCK && data->lockReq == SIG_LOCK)
   {
     digitalWrite(doorUnlockLED, LOW);
     delay(500);
     digitalWrite(doorLockLED, HIGH);
   }
   
   // Handle a request to change the state of the lights
   if (lightState == SIG_LIGHTOFF && data->lightReq == SIG_LIGHTON)
   {
     digitalWrite(lightOffLED, LOW);
     delay(500);
     digitalWrite(lightOnLED, HIGH);
   }
   else if (lightState == SIG_LIGHTON && data->lightReq == SIG_LIGHTOFF)
   {
     digitalWrite(lightOnLED, LOW);
     delay(500);
     digitalWrite(lightOnLED, HIGH);
   }
   
   // Update the current global state of the lock and light
   lockState = data->lockReq;
   lightState = data->lightReq;
}

/**
* Sends a response back to a sensor node based on the results of the
* action requested.
*
* @param response    The response (success or failure)
* @param lockState   The lock state after the requested action
* @param lightState  The light state after the requested action
*/
void sendResponse(int ack, int response, int lockState, int lightState)
{
    res_msg msg;
    memset(&msg, 0x0, 16);
    msg.id = CTRL_ID;
    msg.ack = ack;
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
* Checks a message received from a sensor for errors.
*/
int checkErrors(data_msg * data)
{
   // We somehow have more than the max number of allowed nodes
   if(nodeCount >= MAX_NODES)
   {
     Serial.println("Too many nodes.");
     return -1;
   }     
       
   /*
   //A node is new but is trying to claim that it isn't
   if(nodeMap.getIndexOf(data->id) == -1 && data->sequence != 1)
   {
     Serial.println("Node is new but trying to claim it isn't);
     return UNODE_ERR;
   }
   
   // The sequence number is not correct for a given node
   if(data->sequence < nodeMap.getValueOf(data->id))
       return SEQ_ERR;
   if(data->sequence == nodeMap.getValueOf(data->id))
       return DUP_ERR;
   */
   
   // A bogus value is in the lock or light request field    
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
   
   // The checksum of the message is not valid.
   if(CRC::CRC16((uint8_t*)data, 8) != data->checksum)
   {
     Serial.println("Checksum invalid");
     return CHKSUM_ERR;
   }

   return 0;
}

