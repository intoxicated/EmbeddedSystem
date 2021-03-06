#include <Time.h>

#include <AESLib.h>
#include <CRC16.h>
#include <Time.h>
#include "structs.h"

#define NODE_ID   0

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
const int motionInput = 2; 
const int lightInput = A5;
const int motionPower = 13;

// Debug pins
const int sendingMessage = 8;
const int resendingMessage = 9;
const int ackReceived = 10;
const int nackReceived = 11;
const int nonEvent = 12;

// Thresholds
const int BRIGHTNESS_THRESHOLD = 100;
const int LOCK_TIME_THRESHOLD = 2;
const int LIGHT_TIME_THRESHOLD = 12;

/*
* A key has to be 16 bytes. This key needs to be identical to the
* key coded into the controller.
*/
const byte key[] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15}; 

/*
* Each node throughout its lifetime keeps track of a sequence number for message
* that it sends. This sequence number is used to prevent replay attacks, and confirm
* that the controller received and performed the actions requested.
*/
int seqNumber;
boolean messageConfirmed;
data_msg lastMessage;

/**
* Whenever there is no movement detected, we need to check when the last piece of movement
* happened. This determines whether or not we need to change the state of the room.
*/
time_t lastMovement;

/**
* We need to keep track of the current state of the room. There's no point in sending a message
* to the controller if the state isn't going to change. We probably need to incorporate a
* protocol to prevent this from getting out of sync.
*/
int lockState;
int lightState;

void sendData(int, int);
void determineNonMotionEvent(int);
void determineMotionEvent(int);
void productionCode();
void resendLastMessage();

/**
* Sets up the pin mode and the serial port
*/
void setup()
{
    Serial.begin(9600);
    pinMode(motionInput, INPUT);
    pinMode(lightInput, INPUT);
    pinMode(motionPower, OUTPUT);
    digitalWrite(motionPower, HIGH);
    
    // Debug pins
    pinMode(nonEvent, OUTPUT);
    
    memset(&lastMessage, 0x00, sizeof(struct data_msg));
    
    // We should probably negotiate this for project 2.
    // But for now, door is locked and lights are off at the start of our prototype demo
    lightState = SIG_LIGHTOFF;
    lockState = SIG_LOCK;
    
    // This is also less than ideal.
    lastMovement = now();
    
    // No message means it is vacuously confirmed
    messageConfirmed = true;
    
    seqNumber = 0;
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
   byte incoming[16];
   
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
      
      // Ignore this message if it isn't for us.
      if (data->id != NODE_ID)    return;
      
      // Signal a success or a failure
      if(data->response == RES_OK && data->ack == (seqNumber - 1))
      {
         lockState = data->lockState;
         lightState = data->lightState;
         digitalWrite(ackReceived, HIGH);
         delay(500);
         digitalWrite(ackReceived, LOW);
      }
      else if(data->response != RES_OK) 
      {
         digitalWrite(nackReceived, HIGH);
         delay(500);
         digitalWrite(nackReceived, LOW); 
      }
      else if(data->ack != seqNumber -1)
      {
         digitalWrite(nackReceived, HIGH);
         delay(500);
         digitalWrite(nackReceived, LOW);  
      }
   }
}

/**
* The main event loop of the sensor node. Currently, this just runs
* our set of hardcoded environment changes.
*/
void loop()
{
  //executeTests();
  productionCode();
}


/**
* Pseudocode that indicates what a production environment would look like. This
* is the base code that we will start with when we start working to add the
* timing/interrupts to the code
*
* In this section, we assume that we are tracking timeSinceLastMovement.
*/
void productionCode()
{
    boolean motion, messageSent;
    
    /*
    if (!messageConfirmed)    
    {
       digitalWrite(resendingMessage, HIGH);
       delay(500);
       digitalWrite(resendingMessage, LOW);
       
       Serial.write((byte *)&lastMessage, 16);
       
       delay(500);
       
       return;
    }
    */
    
    // Gather the environment data for this run of the loop
    motion = digitalRead(motionInput);
 
    if (motion) determineMotionEvent();
    else        determineNonMotionEvent();
        
    messageSent = motion ? determineMotionEvent() : determineNonMotionEvent();
    
    delay(500);
}

/**
* Determines the proper message (if any) for the detection of motion by the
* sensor.
*
* @return Whether or not a message was sent to the controller
*/
boolean determineMotionEvent()
{
  boolean messageSent = false;
  boolean turnLightsOn = false;
  boolean unlockDoor = false;
  int brightness;
  
  // Set the last recorded motion to be now.
  lastMovement = now();
  
  //Serial.println("Motion!");
  
  brightness = analogRead(lightInput);
  
  // Determine what modifications we should make
  if (lightState == SIG_LIGHTOFF && brightness > BRIGHTNESS_THRESHOLD) turnLightsOn = true;
  if (lockState == SIG_LOCK)                                           unlockDoor = true;

  
  // Send the appropriate message if any modifications should be made
  if (turnLightsOn || unlockDoor)
  {
    /**** DEBUG ****/
    if (unlockDoor)    lockState = SIG_UNLOCK;    
    {
     // Serial.println("Unlocking door.");
      lockState = SIG_UNLOCK;
    }
    if (turnLightsOn)
    {
      // Serial.println("Turning lights on.");
      lightState = SIG_LIGHTON;
    }
    /**** DEBUG ****/
    
    sendData(unlockDoor ? SIG_UNLOCK : SIG_NULL,
             turnLightsOn ? SIG_LIGHTON : SIG_NULL);

    messageSent = true;
  }
    
  return messageSent; 
}


/**
* Determine if we need to send a message if no motion was detected
* on a given environment data scan.
*
* @return  Whether or not we sent a message
*/
boolean determineNonMotionEvent()
{
  time_t current;
  //Serial.println("No motion");
  // If the door is lock and the lights are off and we detect no movement we
  // have nothing to do.
  if (lockState == SIG_LOCK && lightState == SIG_LIGHTOFF)
  {
    //Serial.println("I have nothing to do.");
    return false;
  }
  
  current = now();
  
  if (current - lastMovement > LIGHT_TIME_THRESHOLD)
  {
    /*** DEBUG ***/
    //Serial.println("Locking door and turning off light.");
    /*** DEBUG ***/
    sendData(SIG_LOCK, SIG_LIGHTOFF);
    lockState = SIG_LOCK;
    lightState = SIG_LIGHTOFF;
    return true;
  }
  else if ((current - lastMovement > LOCK_TIME_THRESHOLD) && lockState == SIG_UNLOCK)
  {
    /*** DEBUG ***/
    //Serial.println("Locking door.");
    /*** DEBUG ***/
    sendData(SIG_LOCK, SIG_NULL);
    lockState = SIG_LOCK;
    return true;
  }

  return false;
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
    memset(&data, 0x0, sizeof(struct data_msg));
    
    // Setup the message struct
    data.id = NODE_ID;
    data.sequence = seqNumber++;
    data.reserve = 0xFF;
    data.lockReq = lock;
    data.lightReq = light;
    data.checksum = CRC::CRC16((uint8_t *)&data, 8);
    memset(data.padding, 0x20, 6);
    
    // Keep a copy of this in case we need to resend it
    memcpy(&lastMessage, &data, sizeof(struct data_msg));
    
    // Encrypt and Send
    aes128_enc_single(key, &data);
    Serial.write((byte *)&data, 16);
}

/**
* A set of "fabricated" changes to the environment that could occur in the wild
* in order to test the behavior of our controller and the effectiveness of
* our protocol.
*/
void executeTests()
{
  data_msg test;
  seqNumber = 0;

  delay(5000);
  
  // Test 1: Send a lights on message.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seqNumber++;
  test.reserve = 0xFF;
  test.lockReq = SIG_NULL;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6); 
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(6000);
  
  // Test 2: Send a lights off message.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seqNumber++;
  test.reserve = 0xFF;
  test.lockReq = SIG_NULL;
  test.lightReq = SIG_LIGHTOFF;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(6000);
 
  // Test 3: Send a unlock message
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seqNumber++;
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
  test.sequence = seqNumber++;
  test.reserve = 0xFF;
  test.lockReq = SIG_LOCK;
  test.lightReq = SIG_NULL;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(6000);  
  
  
  // Test 5: Send a valid message encrypted with invalid key
  // Controller shouldn't do anything since decrypted value 
  // is garbage
  byte fakekey [] = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16};
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seqNumber;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(fakekey, &test);
  Serial.write((byte *)&test, 16);
  
  delay(6000);
  
  // Test 6: Send a bogus request.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seqNumber;
  test.reserve = 0xFF;
  test.lockReq = 15;
  test.lightReq = 84;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
    
  delay(6000);
  
  // Test 7: Demostrating bit flip or bit loss.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = seqNumber;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_LIGHTON;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  test.lightReq = SIG_LIGHTOFF;      // "Bit flip/loss"
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(6000);
  
  // Test 8: Testing a replay attack.
  memset(&test, 0x00, 16);
  test.id = NODE_ID;
  test.sequence = 2;
  test.reserve = 0xFF;
  test.lockReq = SIG_UNLOCK;
  test.lightReq = SIG_NULL;
  test.checksum = CRC::CRC16((uint8_t *)&test, 8);
  memset(test.padding, 0x20, 6);
  aes128_enc_single(key, &test);
  Serial.write((byte *)&test, 16);
  
  delay(6000);
}
