/**
 * Controller -- lock and light
 *
 */

#include <AESLib.h>
#include <structs.h>
#include <HashMap.h>
#include <CRC16.h>

#define CTRL_ID   4321
#define MAX_NODES 10

const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 
//std::map<int,long> test;
HashType<int, long> nodeArray[MAX_NODES];
HashMap<int, long> nodeMap = HashMap<int, long>(nodeArray, MAX_NODES);

byte outgoing[16];
byte incoming[16];

const int doorLockLED = 12;
const int doorUnlockLED = 11;
const int lightRedLED = 10;
const int lightGreenLED = 9;
const int lightBlueLED = 8;

int lockState = 0, lightState = 0;
int res, nodeCount = 0;
long currentSeq = 0;

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

int checkErrors(data_msg * data)
{
   if(nodeCount >= MAX_NODES)
       return -1;       
   if(nodeMap.getIndexOf(data->id) == -1 && data->sequence != 1)
       return UNODE_ERR;
   if(data->sequence < nodeMap.getValueOf(data->id))
       return SEQ_ERR;
   if(data->lockReq != SIG_LOCK && data->lockReq != SIG_UNLOCK)
       return LOCK_ERR;
   if(data->lightReq != SIG_LIGHTON && data->lightReq != SIG_LIGHTOFF)
       return LIGHT_ERR;
   if(CRC::CRC16((uint8_t*)data, 8) != data->checksum)
       return CHKSUM_ERR;
   if(data->sequence == nodeMap.getValueOf(data->id))
       return DUP_ERR;
   return 0;
}

int readAndSet()
{
   int count = 0;
   
   //read all available data
   while(Serial.available())
   {
      //error if available data is longer than 16 bytes
      if(count >= 16)
         return LEN_ERR;
      incoming[count++] = Serial.read();
      delay(10);
   }
   
   //error if collected data is shorter than 16 bytes
   if(count != 16)
       return LEN_ERR;
   int error;
   //decrypt data
   aes128_dec_single(key,incoming); 
   data_msg * data = (data_msg *)incoming;
   
   currentSeq = data->sequence;
   
   if((error = checkErrors(data)) != NO_ERR)
       return error;
   // first sequence, so save both id and sequnence    
   if(data->sequence == 1 && nodeCount < 10) {
       nodeMap[nodeCount++](data->id, data->sequence);
   }
   
   //otherwise, previously communicated
   nodeMap.setValueOf(data->id, data->sequence);
   
   //change state as request 
   if(data->lockReq != lockState)
       lockState = data->lockReq;
   if(data->lightReq != lightState)
       lightState = data->lightReq;
   
   return RES_OK;
}

void loop()
{
   if(Serial.available())
   {
      res = readAndSet();
      sendResponse(res, lockState, lightState);
   }
}
