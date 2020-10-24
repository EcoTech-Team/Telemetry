/*
 * msg_lib.c
 */

#include "msg_lib.h"

static uint8_t  _CrcTable[256];

void MSG_CrcInit(void)
{
	uint8_t remainder;
	uint8_t dividend = 0;
    //! Compute the remainder of each possible dividend.
    do
    {
        remainder = dividend;
        //! Perform modulo-2 division, a bit at a time.
        for (uint8_t bit=8; bit>0; bit--)
        {
            //! Try to divide the current data bit.
            if(remainder & 0x80)
                remainder = (remainder << 1) ^ POLYNOMIAL;
            else remainder = (remainder << 1);
        }
        //! Store the result into the table.
        _CrcTable[dividend++] = remainder;
    } while(dividend);
}


uint8_t MSG_CalculateCrc(uint8_t *data, uint8_t len)
{
    uint8_t remainder = 0;
    //! Divide the message by the polynomial, a byte at a time
    for (uint8_t byte=0; byte<len; byte++)
    {
        uint8_t d = data[byte] ^ remainder;
        remainder = _CrcTable[d];
    }
    //! The final remainder is the CRC.
    return remainder;
}


bool MSG_ValidateCrc(uint8_t *data, uint8_t len)
{
    if(MSG_CalculateCrc(data, len)) return false;
    else return true;
}

void BUS_Received(uint8_t *buff, uint8_t len)
{
    if(buff[0]==ADDRESS && MSG_ValidateCrc(&buff[0],buff[2]+4))
        MSG_Received(&buff[1],len-2);
}

void Clear_Message (MSG_Message *msg)
{
    msg->Address = 0;
    msg->Command = 0;
    msg->Length = 0;
    msg->CRC = 0;
    free(msg->Payload);
}
