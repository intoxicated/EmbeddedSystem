#ifndef __STRUCT_H__
#define __STRUCT_H__

/**
* A set of constants and structures that will be used in our prototype
* system.
*
* Last Modified: March 23, 2014
*/

// Light and lock state constants
#define SIG_NULL     0
#define SIG_UNLOCK   1
#define SIG_LOCK     2
#define SIG_LIGHTON  4
#define SIG_LIGHTOFF 8

// Response constants
#define RES_OK       0
#define RES_ERR      1

// Error codes
#define NO_ERR        0
#define LOCK_ERR     -1
#define LIGHT_ERR    -2
#define SEQ_ERR      -3
#define LEN_ERR      -4
#define UNODE_ERR    -5
#define CHKSUM_ERR   -6
#define EXCEED_ERR   -7
#define DUP_ERR      -8
#define SAME_STATE   -9


/*
 * The message struct that will be used 
 * when the sensor sends data to the controller
 */
typedef struct data_msg {

  uint8_t id;
  long    sequence;
  uint8_t reserve;
  uint8_t lockReq;
  uint8_t lightReq;
  unsigned short checksum;

  uint8_t padding[6];
};

/*
 * The message struct that will be used
 * when the controller sends an ack message to the sensor
 */
typedef struct res_msg {
  uint8_t id;
  long    ack;
  uint8_t reserve;
  uint8_t response;
  uint8_t lockState;
  uint8_t lightState;
  unsigned short checksum;

  uint8_t padding[5];
};

#endif
