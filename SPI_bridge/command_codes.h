/**
   \file command_codes.h
   
   \author G. Icking-Konert
   \date 2017-12-15
   \version 0.1
   
   \brief declaration of command and error codes
   
   declaration of command and error codes for USB <-> SPI-master bridge.
   For protocol details refer to separate file 'protocol.ods'
*/

// for including file only once
#ifndef _CODES_H_
#define _CODES_H_

// command codes (PC -> Arduino)
#define CMD_CONFIG_SPI       0x00     //< configure SPI, e.g. baudrate and poratity
#define CMD_SET_PIN          0x01     //< set state of pin, e.g chip select or reset
#define CMD_SEND_RECEIVE     0x02     //< send/receive message via SPI

// status codes (Arduino -> PC)
#define PENDING              0x00     //< no command received
#define SUCCESS              0x01     //< command ok
#define ERROR_FRAME_LENGTH   0x02     //< zero or loo long frame length
#define ERROR_CHECKSUM       0x03     //< received and calculated checksums don't match 
#define ERROR_ILLEGAL_CMD    0x04     //< command unknown
#define ERROR_ILLEGAL_PARAM  0x05     //< error with command parameters

#endif // _CODES_H_

