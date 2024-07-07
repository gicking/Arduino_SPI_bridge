#!/usr/bin/python3

"""
  read/write via SPI using Arduino as USB<->SPI gateway
  
  Serial protocol:
    command:  len+cmd+data[]+chk     (PC -> Arduino)
    response: len+ack+data[]+chk     (Arduino -> PC)

  with:
    - len:    the length of the complete frame incl. len and checksum
    - cmd:    the command code (list see codes.h)
    - chk:    inverted XOR checksum over the frame w/o chk
    - ack:    return code, see command_codes.h
    - data[]: data array with length depending on respective command
   
  Notes:
    - synchronization is by time: >TIMEOUT_FRAME between bytes starts new frame
    - for protocol details refer to separate file 'protocol.ods'
"""

import sys
import platform
import argparse
import serial, serial.tools.list_ports
import time

# constants:
VERSION = '1.2.0'


#-------------------------------------------------------------------
# class for SPI_Bridge instance
#-------------------------------------------------------------------
class SPI_Bridge:
  """ An instance of SPI_Bridge """

  # supported command codes (PC -> Arduino)
  CMD_SET_PIN         = 0x00      # set state of pin, e.g. board reset
  CMD_CONFIG_SPI      = 0x01      # configure SPI, e.g. CSN, baudrate and polarity
  CMD_SEND_RECV_SPI   = 0x02      # send/receive message via SPI
  CMD_DEBUG           = 0xFF      # dummy test command

  # supported status codes (Arduino -> PC)
  WAIT_FOR_COMMAND    = 0x00      # no command received
  COMMAND_PENDING     = 0x01      # command received
  SUCCESS             = 0x02      # command executed ok
  ERROR_FRAME_LENGTH  = 0x03      # zero or wrong frame length
  ERROR_CHECKSUM      = 0x04      # received and calculated checksums don't match
  ERROR_ILLEGAL_CMD   = 0x05      # command unknown
  ERROR_ILLEGAL_PARAM = 0x06      # illegal command parameters
  ERROR_PC_PORT       = 0xF0      # port port closed
  ERROR_PC_TIMEOUT    = 0xF1      # receive timeout
  ERROR_PC_LENGTH     = 0xF2      # wrong frame length received
  ERROR_PC_CHECKSUM   = 0xF3      # received and calculated checksums don't match
  ERROR_PC_CHECKSUM   = 0xF3      # received and calculated checksums don't match


  # constants from Arduino libs
  LOW                 = 0
  HIGH                = 1
  LSBFIRST            = 0
  MSBFIRST            = 1
  SPI_MODE0           = 0x00
  SPI_MODE1           = 0x04
  SPI_MODE2           = 0x08
  SPI_MODE3           = 0x0C


  #########
  # constructor
  #########
  def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=1):
    """ Return a SPI_Bridge object with opened COM port """
        
    # try to open serial port
    self.port = 0
    try:
      self.port = serial.Serial(port     = port, \
                                baudrate = baudrate, \
                                timeout  = timeout)
    except:
      # on error list available strings
      str = "\nerror opening port \'"+args.port+"\' ("
      ports = serial.tools.list_ports.comports()
      #print(ports)
      for port in ports:
        if platform.system() == 'Windows':
          str += port[0] + ","
        else:
          str += port.device + ","
      str = str[:-1] + "), exit!\n"
      print(str)
      exit(1)

  # constructor
  ##################

  
  #########
  # destructor
  #########
  def __del__(self):
    """ close open COM port """
    
    if (self.port != 0):
      self.port.close()
      self.port = 0

  # destructor
  ##################


  ##################
  # calculate inverted 8-bit XOR checksum over buffer
  ##################
  def __checksum__(self, buf):
    """ calculate inverted 8-bit XOR checksum over buf """
    
    # calculate checksum
    chk = 0xFF
    for c in buf:
      chk = chk ^ c
    
    # debug
    #print(hex(chk))
    
    return chk

  # __checksum__
  ##################


  #########
  # set state of pin
  #########
  def setPin(self, pin=0, state=True):
    """ set state of Arduino digital pin """

    # check for open port
    if self.port == 0:
      return self.ERROR_PC_PORT

    # construct command frame: len+cmd+pin+state+chk
    Tx = bytearray(5)
    Tx[0] = len(Tx)
    Tx[1] = self.CMD_SET_PIN
    Tx[2] = pin
    if state == False:
      Tx[3] = 0x00
    else:
      Tx[3] = 0x01
    Tx[4] = self.__checksum__(Tx[:-1])

    # empty Rx buffer (changed for Python 3.x)
    if (sys.version_info.major == 3):
      self.port.flushInput()
    else:
      self.port.reset_input_buffer()

      # send command
    self.port.write(Tx)
    # print("send:    "+"".join("0x%02x " % b for b in Tx), flush=True)

    # receive response
    lenRx = 3
    Rx = bytearray(lenRx)
    self.port.readinto(Rx)
    # print("receive: "+"".join("0x%02x " % b for b in Rx), flush=True)

    # check for timeout
    if (len(Rx) != lenRx):
      sys.stderr.write("\ntimeout response\n")
      return self.ERROR_PC_TIMEOUT

    # check length
    if (Rx[0] != lenRx):
      sys.stderr.write("\nlength error (expect " + str(lenRx) + " read " + str(Rx[0]) + ")\n")
      return self.ERROR_PC_LENGTH

    # check checksum
    if (Rx[lenRx - 1] != self.__checksum__(Rx[:-1])):
      sys.stderr.write(
        "\nchecksum error (expect 0x%0.2X" % self.__checksum__(Rx[:-1]) + " read 0x%0.2X" % Rx[lenRx - 1] + ")\n")
      return self.ERROR_PC_CHECKSUM

    # check for ACK
    if (Rx[1] != self.SUCCESS):
      sys.stderr.write("\nerror response (expect 0x%0.2X" % self.SUCCESS + " read 0x%0.2X" % Rx[1] + ")\n")
      return Rx[1]

    # return success
    return self.SUCCESS

  # setPin
  ##################


  #########
  # configure SPI
  #########
  def configSPI(self, csn=10, baudrate=1000000, order=LSBFIRST, mode=SPI_MODE0):
    """ configure Arduino SPI """
    
    # check for open port
    if self.port == 0:
      return self.ERROR_PC_PORT
  
    # construct command frame: len+cmd+CSN+BR[3]+BR[2]+BR[1]+BR[0]+order+mode+chk
    Tx = bytearray(10)
    Tx[0] = len(Tx)
    Tx[1] = self.CMD_CONFIG_SPI
    Tx[2] = csn
    Tx[3] = ((baudrate >> 24) & 0xFF)
    Tx[4] = ((baudrate >> 16) & 0xFF)
    Tx[5] = ((baudrate >>  8) & 0xFF)
    Tx[6] = ((baudrate >>  0) & 0xFF)
    Tx[7] = order
    Tx[8] = mode
    Tx[9] = self.__checksum__(Tx[:-1])

    # empty Rx buffer (changed for Python 3.x)
    if (sys.version_info.major == 3):
      self.port.flushInput()
    else:
      self.port.reset_input_buffer() 

    # send command
    self.port.write(Tx)
    #sys.stdout.write("send:    "+"".join("0x%02x " % b for b in Tx)); sys.stdout.flush()

    # receive response
    lenRx = 3
    Rx = bytearray(lenRx)
    self.port.readinto(Rx)
    #sys.stdout.write("receive: "+"".join("0x%02x " % b for b in Rx)); sys.stdout.flush()
        
    # check for timeout
    if (len(Rx) != lenRx):
      sys.stderr.write("\ntimeout response\n")
      return self.ERROR_PC_TIMEOUT

    # check length
    if (Rx[0] != lenRx):
      sys.stderr.write("\nlength error (expect " + str(lenRx) + " read " + str(Rx[0]) + ")\n")      
      return self.ERROR_PC_LENGTH
      
    # check checksum
    if (Rx[lenRx-1] != self.__checksum__(Rx[:-1])):
      sys.stderr.write("\nchecksum error (expect 0x%0.2X" % self.__checksum__(Rx[:-1]) + " read 0x%0.2X" % Rx[lenRx-1] + ")\n")
      return self.ERROR_PC_CHECKSUM
      
    # check for ACK
    if (Rx[1] != self.SUCCESS):
      sys.stderr.write("\nerror response (expect 0x%0.2X" % self.SUCCESS +" read 0x%0.2X" % Rx[1] + ")\n")      
      return Rx[1]

    # return success
    return self.SUCCESS

  # configSPI
  ##################



  #########
  # send/receive SPI frame
  #########
  def sendReceive(self, bufTx):
    """ send/receive SPI frame """
    
    # check for open port
    if self.port == 0:
      return self.ERROR_PC_PORT
    
    # construct command frame: len+cmd+data[]+chk
    Tx = bytearray(3+len(bufTx))
    Tx[0] = len(Tx)
    Tx[1] = self.CMD_SEND_RECV_SPI
    i=0
    for c in bufTx:
      Tx[2+i] = c
      i += 1
    Tx[-1] = self.__checksum__(Tx[:-1])

    # empty Rx buffer (changed for Python 3.x)
    if (sys.version_info.major == 3):
      self.port.flushInput()
    else:
      self.port.reset_input_buffer() 

    # send command
    self.port.write(Tx)
    #print("send:    "+"".join("0x%02x " % b for b in Tx))

    # receive response
    lenRx = 3+len(bufTx)
    Rx = bytearray(lenRx)
    self.port.readinto(Rx)
    #print("receive: "+"".join("0x%02x " % b for b in Rx)); sys.stdout.flush()


    # check for timeout
    if (len(Rx) != lenRx):
      sys.stderr.write("\ntimeout response\n")
      return self.ERROR_PC_TIMEOUT, bytearray(0)

    # check length
    if (Rx[0] != lenRx):
      sys.stderr.write("\nlength error (expect " + str(lenRx) + " read " + str(Rx[0]) + ")\n")      
      return self.ERROR_PC_LENGTH, bytearray(0)
      
    # check checksum
    if (Rx[lenRx-1] != self.__checksum__(Rx[:-1])):
      sys.stderr.write("\nchecksum error (expect 0x%0.2X" % self.__checksum__(Rx[:-1]) + " read 0x%0.2X" % Rx[lenRx-1] + ")\n")
      return self.ERROR_PC_CHECKSUM, bytearray(0)
      
    # check for ACK
    if (Rx[1] != self.SUCCESS):
      sys.stderr.write("\nerror response (expect 0x%0.2X" % self.SUCCESS +" read 0x%0.2X" % Rx[1] + ")\n")      
      return Rx[1], bytearray(0)

    # return success
    return self.SUCCESS, Rx

  # sendReceive
  ##################



#-------------------------------------------------------------------
# MODULE TEST
#-------------------------------------------------------------------
# only executute this block of code if running this module directly,
# *not* if importing it
# -see here: http://effbot.org/pyfaq/tutor-what-is-if-name-main-for.htm
if __name__ == "__main__":

  ##################
  # helper routine: exit program but avoid iPython console to terminate as well
  ##################
  def getchar():
    """
     python equivalent of getchar()
    """
    ch = 0
    if platform.system() == 'Windows':
      import msvcrt as m
      ch = m.getch()
      #sys.stdio.flush()
      #sys.stderr.flush()
    else:
      import tty, termios
      fd = sys.stdin.fileno()
      old_settings = termios.tcgetattr(fd)
      tty.setraw(sys.stdin.fileno())
      ch = sys.stdin.read(1)
      termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch
      
  # getchar
  ##################
  
  
  ##################
  # helper routine: exit program but avoid iPython console to terminate as well
  ##################
  def Exit(code=0):
    
    # flush colsole output
    sys.stderr.flush()
    sys.stdout.flush()
    
    # for iPython console raise exception, else exit normally
    try:
      __IPYTHON__
    except NameError:
      if (sys.version_info.major == 3):
        pass
        #input("\npress return ... ")
      else:
        pass
        #raw_input("\npress return ... ")
      print("")
      exit(code)
    else:
      raise Exception('exit')
      
  # Exit
  ##################

    
  # commandline parameters with defaults
  parser = argparse.ArgumentParser(description="Fluke 187/189 read-out")
  parser.add_argument('-p', '--port',    type=str,   help='port name', required=False, default='/dev/ttyACM0')
  parser.add_argument('-b', '--baud',    type=int,   help='baudrate',  required=False, default=115200)
  args = parser.parse_args()

  # print header
  print("\nSPI bridge test\n")
  
  # open connection to SPI slave
  bridge = SPI_Bridge(port=args.port, baudrate=args.baud, timeout=0.2)
  
  # avoid issue with Arduino bootloader
  sys.stdout.write("wait for bootloader ... "); sys.stdout.flush()
  time.sleep(2.0)
  sys.stdout.write("done\n"); sys.stdout.flush()
  
  # configure Arduino SPI
  sys.stdout.write("config SPI ... "); sys.stdout.flush()
  err = bridge.configSPI(csn=10, baudrate=4000000, order=bridge.MSBFIRST, mode=bridge.SPI_MODE0)
  if err != bridge.SUCCESS:
    Exit(1)
  sys.stdout.write("done\n"); sys.stdout.flush()

  # main loop. Just send dummy frames
  bufTx = bytearray([0x01, 0x02, 0x03, 0x04])
  while True:
    
    # send SPI frame
    #sys.stdout.write("send/receive SPI ... "); sys.stdout.flush()
    print("Tx: " + "".join("0x%02x " % b for b in bufTx) + "/ ", end="", flush=False)
    err, bufRx  = bridge.sendReceive(bufTx)
    if err != bridge.SUCCESS:
      Exit(1)
    else:
      pass
      #sys.stdout.write("done\n"); sys.stdout.flush()
      print("Rx: " + "".join("0x%02x " % b for b in bufRx[2:-1]), flush=False)

    # wait a bit
    #time.sleep(0.001)

  # wait for return and exit
  Exit();

