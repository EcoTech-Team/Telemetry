/*
 * msg_Lib.h
 */

#ifndef MSG_LIB_H_
#define MSG_LIB_H_


#include <stdint.h>
#include <stdbool.h>


#define POLYNOMIAL          0x9b
#define ADDRESS             1

#pragma pack(1)
typedef struct
{
    uint8_t Address;
    uint8_t Command;
    uint8_t Length;
    uint8_t Payload[];
    uint8_t CRC;
} MSG_Message;


void MSG_CrcInit(void);
uint8_t MSG_CalculateCrc(uint8_t *data, uint8_t len);
bool MSG_ValidateCrc(uint8_t *data, uint8_t len);
void __attribute__((weak)) MSG_Received(uint8_t *buff, uint8_t len);


#endif /* MSG_LIB_H_ */
