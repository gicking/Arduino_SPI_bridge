/**
   \file SPI_bridge.ino

   \author G. Icking-Konert
   \date 2018-12-07
   \version 0.2
   
   \brief Arduino as a USB <-> SPI-master bridge
   
   Use an Arduino as a USB <-> SPI-master bridge. As (polling) serialEvent() is used,
   timing is critical, but code is compatible with all Arduinos.

   Serial protocol:
     command:  len+cmd+data[]+chk     (PC -> Arduino)
     response: len+ack+data[]+chk     (Arduino -> PC)

   with:
     - len:    the length of the complete frame incl. len and checksum
     - cmd:    the command code (list see codes.h)
     - chk:    inverted XOR checksum over the frame w/o chk
     - ack:    SUCCESS if ok; Else error code (see codes.h) without data
     - data[]: data array with length depending on respective command
   
   Notes:
     - synchronization is by time: >TIMEOUT_FRAME between bytes starts new frame
     - for protocol details refer to separate file 'protocol.ods'
*/

// project includes
#include <SPI.h>
#include "command_codes.h"

// general project settings
#define CMD_UART      Serial      //< UART for remote control by PC
#define CMD_BAUDRATE  115200      //< baudrate for communication with PC
#define LEN_BUF_RX    150         //< must be higher than longest SPI frame + 3B overhead
#define LEN_BUF_TX    150         //< must be higher than longest SPI frame + 3B overhead
#define TIMEOUT_FRAME 1000L       //< maximum time [us] between bytes in frame
//#define DEBUG_UART    Serial1     //< optional debugging UART @ 115.2kBaud. Requires Arduino with >1 UART

// global variables
uint8_t bufRx[LEN_BUF_RX];        //< UART buffer PC->Arduino
uint8_t statusCmd;                //< protocol handler status
#ifdef DEBUG_UART
  char  debugMsg[200];            //< string buffer for debug output
#endif



/**
  \fn inline uint8_t checksum(uint8_t *buf)
   
  \brief calculate checksum over frame (w/o checksum)
  
  \param[in] buf   buffer containing frame. buf[0] contains total frame length
  
  Calculate checksum over frame (inverted XOR over bytes w/o checksum).
  Routine is inline for speed and to avoid reentrance issues.
*/
inline uint8_t checksum(uint8_t *buf) {
  
  uint8_t chk = 0xFF, i;
  uint8_t lim = buf[0]-1;   // assert only one minus
  for (i=0; i<lim; i++)
    chk ^= buf[i];

  return(chk);
  
} // checksum()



/**
  \fn void serialEvent(void)
   
  \brief receive protocol handler
  
  Receive protocol handler. Is called periodically by main(). Not interrupt driven!
*/
void serialEvent()
{
  uint8_t           c;                //< received byte (if any)
  static uint8_t    idxRx = 0;        //< index pointer in bufRx[]
  static uint32_t   lastTime = 0;     //< last time [us] a byte was received (for frame sync)
  uint32_t          currTime;         //< for consistency call micros only once

  // handle received bytes
  while (CMD_UART.available()) {

    // get byte from buffer
    c = CMD_UART.read();
    
    // on inter-byte timeout start new frame (w/o error response to PC)
    currTime = micros();
    if (currTime - lastTime > TIMEOUT_FRAME) {
      statusCmd = PENDING;              // revert state machine
      idxRx     = 0;                    // init buffer pointer for new frame
    } // timeout
    lastTime = currTime;                // for next call
  
    // store received byte in buffer
    bufRx[idxRx++] = c;
    
    // check for zero length or buffer overflow
    if ((bufRx[0] == 0) || (idxRx > LEN_BUF_RX)) {
      statusCmd = ERROR_FRAME_LENGTH;   // set error status (sent in loop)
      idxRx     = 0;                    // init buffer pointer for new frame
      CMD_UART.end();                   // avoid issues if PC keeps sending
    } // buffer overflow
  
    
    // frame complete
    else if ((bufRx[0] != 0) && (idxRx >= bufRx[0])) {
      
      // check checksum (inverted XOR over frame bytes w/o chk)
      if (checksum(bufRx) != bufRx[bufRx[0]-1]) {
        statusCmd = ERROR_CHECKSUM;     // set error status (sent in loop)
      }
      
      // command received successfully
      else {
       statusCmd = SUCCESS;            // set success status (sent in loop)
      }
      
      // init buffer pointer for new frame
      idxRx = 0;
  
      // avoid issues if PC keeps sending
      CMD_UART.end();
      
    } // frame complete

  } // while Serial.available()
  
} // serialEvent()



/**
  \fn void setup(void)
   
  \brief Arduino initialization routine
  
  Standard Arduino initialization routine. Is called only once after startup.
*/
void setup() {
  
  /////
  // initialize global variables
  /////

  // init UART buffer PC->Arduino
  for (uint8_t i=0; i<LEN_BUF_RX; i++)
    bufRx[i] = 0;

  // no command waiting
  statusCmd = PENDING;
  

  /////
  // SPI setup
  /////
  
  // configure SS pin output high (see App Note SPI lib)
  digitalWrite(SS, HIGH);
  pinMode(SS, OUTPUT);
  
  // initialize SPI by default to 1MBaud, Mode 0, LSB first
  SPI.beginTransaction(SPISettings(1000000L, LSBFIRST, SPI_MODE0));
  SPI.begin();


  /////
  // command UART initialization
  /////
  
  // initialize the communication to PC
  CMD_UART.begin(CMD_BAUDRATE);
  while (!CMD_UART);   // wait for serial port (relevant for native USB)


  /////
  // debug UART initialization to 115.2kBaud (optional)
  /////
  
  #ifdef DEBUG_UART
    DEBUG_UART.begin(115200);
    while (!DEBUG_UART);   // wait for serial port (relevant for native USB)
    DEBUG_UART.println("\ndebug output ready ... \n\n");
  #endif

} // setup()



/**
  \fn void loop(void)
   
  \brief Arduino loop routine
  
  Standard Arduino loop routine. Is called continuously.
*/
void loop() {

  uint8_t   bufTx[LEN_BUF_TX];    // for sending response
  uint8_t   lenRx;                // expected frame length

  
  // if command was received, handle it
  if (statusCmd != PENDING) {

    // check for frame errors -> send error status to PC
    if (statusCmd != SUCCESS)  {

      // assemble error response: len+err+chk
      bufTx[0] = 3;
      bufTx[1] = statusCmd;
      bufTx[2] = checksum(bufTx);
      
      // send error message to debug UART
      #ifdef DEBUG_UART
        sprintf(debugMsg, "frame error: %d", statusCmd);
        DEBUG_UART.println(debugMsg);
      #endif
      
    } // if frame error


    // no error in handler -> execute command
    else {

      // command is in buf_Rx[1]
      switch (bufRx[1]) {

        ///////
        // configure SPI interface: len+cmd+BR[3]+BR[2]+BR[1]+BR[0]+order+mode+chk
        ///////
        case CMD_CONFIG_SPI:
          
          // check command length
          lenRx = 9;
          if (bufRx[0] != lenRx) {
            
            // assemble error response: len+err+chk
            bufTx[0] = 3;
            bufTx[1] = ERROR_ILLEGAL_PARAM;
            bufTx[2] = checksum(bufTx);
      
            // send error message to debug UART
            #ifdef DEBUG_UART
              sprintf(debugMsg, "SET_CSN: illegal frame length (expect %d, read %d)", (int) lenRx, (int) (bufRx[0]));
              DEBUG_UART.println(debugMsg);
            #endif
            
          } // illegal parameter length
          
          // no error -> configure SPI
          else {
            
            // constant definition see SPI.h
            uint32_t  baudrate = (((uint32_t) (bufRx[2])) << 24) + \
                                 (((uint32_t) (bufRx[3])) << 16) + \
                                 (((uint32_t) (bufRx[4])) <<  8) + \
                                 (((uint32_t) (bufRx[5])) <<  0);       // max. speed [Baud]
            uint8_t   order    = bufRx[6];                              // LSBFIRST(=0) / MSBFIRST(=1)
            uint8_t   mode     = bufRx[7];                              // SPI_MODEx
            
            // configure SPI
            SPI.end();
            #if defined (__arm__)
              SPI.beginTransaction(SPISettings(baudrate, (BitOrder) order, mode));
            #else
              SPI.beginTransaction(SPISettings(baudrate, order, mode));
            #endif
            SPI.begin();
            
            // report success to PC
            bufTx[0] = 3;
            bufTx[1] = SUCCESS;
            bufTx[2] = checksum(bufTx);
      
            // send message to debug UART
            #ifdef DEBUG_UART
              sprintf(debugMsg, "CONFIG_SPI: BR=%d, order=%d, mode=%d", (int) baudrate, (int) order, (int) mode);
              DEBUG_UART.println(debugMsg);
            #endif
          
          } // no error
          
          // CMD_CONFIG_SPI
          break;  


        ///////
        // set pin state: len+cmd+pin+state+chk
        ///////
        case CMD_SET_PIN:
          
          // check command length
          lenRx = 5;
          if (bufRx[0] != lenRx) {
            
            // assemble error response: len+err+chk
            bufTx[0] = 3;
            bufTx[1] = ERROR_ILLEGAL_PARAM;
            bufTx[2] = checksum(bufTx);
      
            // send error message to debug UART
            #ifdef DEBUG_UART
              sprintf(debugMsg, "SET_CSN: illegal frame length (expect %d, read %d)", (int) lenRx, (int) (bufRx[0]));
              DEBUG_UART.println(debugMsg);
            #endif
            
          } // illegal parameter length
          
          // no error -> set pin state
          else {
            
            // set pin to new status
            pinMode(bufRx[2], OUTPUT);
            digitalWrite(bufRx[2], (bufRx[3] != 0));

            // report success to PC
            bufTx[0] = 3;
            bufTx[1] = SUCCESS;
            bufTx[2] = checksum(bufTx);
      
            // send message to debug UART
            #ifdef DEBUG_UART
              sprintf(debugMsg, "SET_PIN: pin=%d, , state=%d)", (int) (bufRx[2]), (int) (bufRx[3]));
              DEBUG_UART.println(debugMsg);
            #endif
          
          } // no error
          
          // CMD_SET_PIN
          break;  


        ///////
        // send/receive SPI frame: len+cmd+CSN+data[]+chk
        ///////
        case CMD_SEND_RECEIVE:
          
          // skip length check due to variable frame width

          // avoid issue with variable declaration
          {
            uint8_t  lenSPI = bufRx[0] - 4;     // SPI frame width
            
            // set chip-select pin low
            digitalWrite(bufRx[2], LOW);
            pinMode(bufRx[2], OUTPUT);
            
            // send & receive data
            for (int i=0; i<lenSPI; i++)
              bufTx[2+i] = SPI.transfer(bufRx[3+i]);

            // release chip-select pin
            digitalWrite(bufRx[2], HIGH);

            // report success to PC
            bufTx[0] = 3+lenSPI;
            bufTx[1] = SUCCESS;
            // data[] already set above
            bufTx[bufTx[0]-1] = checksum(bufTx);
        
            // send SPI frame to debug UART
            #ifdef DEBUG_UART
              sprintf(debugMsg, "SEND_RECEIVE: %dB: ", (int) lenSPI);
              DEBUG_UART.print(debugMsg);
              for (int i=0; i<lenSPI; i++) {
                sprintf(debugMsg, "0x%02x ", bufRx[3+i]);
                DEBUG_UART.print(debugMsg);
              }
              DEBUG_UART.println(" ");
            #endif

          }
          
          // CMD_SEND_RECEIVE
          break;  


        ///////
        // unknown command -> error: len+err+chk
        ///////
        default:
         
          // assemble error response: len+err+chk
          bufTx[0] = 3;
          bufTx[1] = ERROR_ILLEGAL_CMD;
          bufTx[2] = checksum(bufTx);
      
          // send error message to debug UART
          #ifdef DEBUG_UART
            sprintf(debugMsg, "Error: unknown command 0x%02x", (int) bufRx[1]);
            DEBUG_UART.println(debugMsg);
          #endif

          // default
          break;
        
      } // switch command

    } // if statusCmd==SUCCESS

    // reset protocol handler state machine
    statusCmd = PENDING;
    bufRx[0]  = 0;
    
    // flush and re-enable command UART
    while (CMD_UART.available())
      CMD_UART.read();
    CMD_UART.begin(CMD_BAUDRATE);

    // send response to PC
    CMD_UART.write(bufTx, bufTx[0]);
    
  } // command pending

} // loop()
