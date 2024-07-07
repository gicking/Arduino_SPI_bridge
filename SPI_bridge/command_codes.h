/**
   \file command_codes.h
   
   \author G. Icking-Konert
   \date 2024-07-07
   \version 1.2.0
   
   \brief declaration of command and error codes
   
   declaration of command and error codes for USB <-> SPI-master bridge.
   For protocol details refer to separate file 'protocol.ods'
*/

// for including file only once
#ifndef _CODES_H_
#define _CODES_H_

// command codes (PC -> Arduino)
#define CMD_SET_PIN          0x00     //< set state of pin, e.g. board reset
#define CMD_CONFIG_SPI       0x01     //< configure SPI, e.g. CSN, baudrate and polarity
#define CMD_SEND_RECV_SPI    0x02     //< send/receive message via SPI

// status codes (Arduino -> PC)
#define WAIT_FOR_COMMAND     0x00     //< no command received
#define COMMAND_PENDING      0x01     //< command received
#define SUCCESS              0x02     //< command executed ok
#define ERROR_FRAME_LENGTH   0x03     //< zero or wrong frame length
#define ERROR_CHECKSUM       0x04     //< received and calculated checksums don't match 
#define ERROR_ILLEGAL_CMD    0x05     //< command unknown
#define ERROR_ILLEGAL_PARAM  0x06     //< illegal command parameters

#endif // _CODES_H_

