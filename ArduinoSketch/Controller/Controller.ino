#include <AESLib.h>
#include <HashMap.h>
#include <CRC16.h>
#include "structs.h"

#define CTRL_ID   4321
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
const int doorLockLED = 12;
const int doorUnlockLED = 11;
const int lightOffLED = 9;
const int lightOnLED = 8;

//key has to be 16 bytes
//this key should be set up both shared with controller
//in case of multiple sensors, controller may need to maintain
//multiple keys that maps to right sensor
const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

// A hashmap to set up the known nodes and their current sequence number
HashType<int, long> nodeArray[MAX_NODES];
HashMap<int, long> nodeMap = HashMap<int, long>(nodeArray, MAX_NODES);

// Buffers for incoming and outgoing messages
byte outgoing[16];
byte incoming[16];

// State information
int lockState = 0;
int lightState = 0;
int res, nodeCount = 0;
long currentSeq = 0;

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
   
   //initial state for lock and light
   digitalWrite(doorLockLED, HIGH);
   delay(500);
   
   digitalWrite(lightOffLED, HIGH);
   delay(500);
}

/**
* Sends a response back to a sensor node based on the results of the
* action requested.
*
* @param response    The response (success or failure)
* @param lockState   The lock state after the requested action
* @param lightState  The light state after the requested action
*/
void sendResponse(int response, int lockState, int lightState)
{
    res_msg msg;
    memset(&msg, 0x0, 16);
    msg.id = CTRL_ID;
    msg.ack = currentSeq + 1;
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
       return -1;       
       
   //A node is new but is trying to claim that it isn't
   if(nodeMap.getIndexOf(data->id) == -1 && data->sequence != 1)
       return UNODE_ERR;
   
   // The sequence number is not correct for a given node
   if(data->sequence < nodeMap.getValueOf(data->id))
       return SEQ_ERR;
   if(data->sequence == nodeMap.getValueOf(data->id))
       return DUP_ERR;
       
   // A bogus value is in the lock or light request field    
   if(data->lockReq != SIG_LOCK && data->lockReq != SIG_UNLOCK &&
       data->lockReq != SIG_NULL)
       return LOCK_ERR;
   if(data->lightReq != SIG_LIGHTON && data->lightReq != SIG_LIGHTOFF &&
       data->lightReq != SIG_NULL)
       return LIGHT_ERR;
   
   // The checksum of the message is not valid.
   if(CRC::CRC16((uint8_t*)data, 8) != data->checksum)
       return CHKSUM_ERR;

   return 0;
}

/**
* Performs the requested action from the sensor node
*
* @param data  A pointer to the message struct received from the sensor
* @return  The action return code
*/
int doAction(data_msg * data)
{
   if(data->lockReq == lockState && data->lightReq == lightState)
       return SAME_STATE;
       
   //change status of door and light
   if(data->lockReq == SIG_LOCK &&
       data->lockReq != lockState){
       digitalWrite(doorUnlockLED, LOW);
       delay(1000);
       digitalWrite(doorLockLED, HIGH);
       delay(1000);
   }
   else if(data->lockReq == SIG_UNLOCK &&
       data->lockReq != lockState){
       digitalWrite(doorLockLED, LOW);
       delay(1000);
       digitalWrite(doorUnlockLED, HIGH);
       delay(1000);
   }

   if(data->lightReq == SIG_LIGHTOFF &&
       data->lightReq != lightState) {
       digitalWrite(lightOnLED, LOW);
       delay(1000);
       digitalWrite(lightOffLED, HIGH);
       delay(1000);
   }
   else if(data->lightReq == SIG_LIGHTON &&
       data->lightReq != lightState){
       digitalWrite(lightOffLED, LOW);
       delay(1000);
       digitalWrite(lightOnLED, HIGH);
       delay(1000);
   }
   
   //change state of lock and light 
   lockState = data->lockReq;
   lightState = data->lightReq;
   
   return NO_ERR;
}

/**
* Reads in and processes a message from the sensor node
*/
int readAndSet()
{
   int error;
   int count = 0;
   
   // Read all available data
   while(Serial.available())
   {
      // It is an error if available data is longer than 16 bytes
      if(count >= 16)
         return LEN_ERR;
      incoming[count++] = Serial.read();
      delay(10);
   }

   // It is an error if collected data is shorter than 16 bytes
   if(count != 16)
       return LEN_ERR;
   // Decrypt the message
   aes128_dec_single(key,incoming); 
   data_msg * data = (data_msg *)incoming;
   char buffer[128];
 
   // Check that the sequence number is ok.
   currentSeq = data->sequence;
   
   // Check that there aren't any errors in the message to the client
   if((error = checkErrors(data)) != NO_ERR)
       return error;
   
   // If this is a new node, add it to our list of known nodes. Otherwise
   // update the sequence number for the given node.  
   if(data->sequence == 1 && nodeCount < 10) {
       nodeMap[nodeCount++](data->id, data->sequence);
   }
   else {
     nodeMap.setValueOf(data->id, data->sequence);
   }
   
   // Change the state as requested
   if((error = doAction(data)) != NO_ERR)
       return error;
   
   return NO_ERR;
}

/**
* The main event loop of the controller node. Simply continue to loop
* until there is data available.
*/
void loop()
{
   if(Serial.available())
   {
      res = readAndSet();
      sendResponse(res, lockState, lightState);
   }
}
