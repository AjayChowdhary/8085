/**
 * EEPROM Programmer
 * (Compatible with 28C64 (8kB) and 28C256(32kB) parallel EEPROM)
 * 
 * Author: Anshuman Mishra (CEDT,NSIT) under the guidance of Prof. Dhananjay V. Gadre and Nikhilesh Prasannakumar
 * (Adapted from MEEPROMMER firmware by mario)
 * Revision: 2.0
 *
 * This software is freeware and can be modified, reused or thrown away without any restrictions.
 * Use this code at your own risk. I'm not responsible for any bad effect or damages caused by this software!!!
 *
 **/


#define VERSIONSTRING "EEPROM Programmer CEDT v2.0"

// define the IO lines for the data - bus
#define D0 2
#define D1 3
#define D2 4
#define D3 5
#define D4 6
#define D5 7
#define D6 8
#define D7 9

// shiftOut part
#define DS     A0
#define LATCH  A1
#define CLOCK  A2

// define the IO lines for the eeprom control
#define CE     A3
#define OE     A4
#define WE     A5

// direct access to port

#define STROBE_PORT PORTC
#define STROBE_DS      0
#define STROBE_LATCH   1
#define STROBE_CLOCK   2
#define STROBE_CE      3
#define STROBE_OE      4
#define STROBE_WE      5

//a buffer for bytes to burn
#define BUFFERSIZE 1024
byte buffer[BUFFERSIZE];
//command buffer for parsing commands
#define COMMANDSIZE 32
char cmdbuf[COMMANDSIZE];
 
unsigned int startAddress,endAddress;
unsigned int lineLength,dataLength;

#define CHIP28C64 0
#define CHIP28C256 2


unsigned int chipType;

//define COMMANDS
#define NOCOMMAND    0
#define VERSION      1
#define SET_ADDRESS  2

#define READ_HEX    10
#define READ_BIN    11
#define READ_ITL    12
 
#define WRITE_HEX   20
#define WRITE_BIN   21
#define WRITE_ITL   22
 
#define CHIP_TYPE   30
#define CHIP_ERASE  31
#define VPP_ON      32
#define VPP_OFF     33

/****************************************************************
 *
 *  CONTROL and DATA functions
 *
 ****************************************************************/
 
// switch IO lines of databus to INPUT state
void data_bus_input() {
  pinMode(D0, INPUT);
  pinMode(D1, INPUT);
  pinMode(D2, INPUT);
  pinMode(D3, INPUT);
  pinMode(D4, INPUT);
  pinMode(D5, INPUT);
  pinMode(D6, INPUT);
  pinMode(D7, INPUT);
}
 
//switch IO lines of databus to OUTPUT state
void data_bus_output() {
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
}
 
//set databus to input and read a complete byte from the bus
//be sure to set data_bus to input before
byte read_data_bus()
{
  //access the port pins directly if possible, for speed
  //the port pins do not match up with the data bits
  //so we need to rearrange them

  return (PIND >> 2) | ((PINB & 0x3) << 6);
}
 
//write a byte to the data bus
//be sure to set data_bus to output before
inline void write_data_bus(byte data)
{
  //access the ports directly if possible, for speed
  //the ports do not match up with the data bits
  //so we need to rearrange them

  //2 bits belong to PORTB and have to be set separtely
  PORTB = (PORTB & 0xF8) | (data >> 6);
  //bit 0 to 6 belong to bit 2 to 8 of PORTD
  PORTD = data << 2;

}


//faster shiftOut function then normal IDE function
//defined as macro so it can be 'unrolled' for more speed
//set the port pins directly if possible

#define FAST_SHIFT(data) { \
  /*shift out the top bit of the byte*/ \
  if (data & 0x80) \
    bitSet(STROBE_PORT,STROBE_DS); \
  else \
    bitClear(STROBE_PORT,STROBE_DS); \
  /*shift data left so next bit is ready*/ \
  data <<= 1; \
  /*register shifts bits on upstroke of clock pin*/ \
  bitSet(STROBE_PORT,STROBE_CLOCK); \
  bitClear(STROBE_PORT,STROBE_CLOCK); \
}


//shift out the given address to the 74hc595 registers
inline void set_address_bus(unsigned int address)
{
  byte hi, low;

  //get high - byte of 16 bit address
 
    hi = address >> 8;

  //get low - byte of 16 bit address
  low = address & 0xff;

  //shift out highbyte using macro for speed
  FAST_SHIFT(hi); FAST_SHIFT(hi); FAST_SHIFT(hi); FAST_SHIFT(hi);
  FAST_SHIFT(hi); FAST_SHIFT(hi); FAST_SHIFT(hi); FAST_SHIFT(hi);
  //shift out lowbyte
  FAST_SHIFT(low); FAST_SHIFT(low); FAST_SHIFT(low); FAST_SHIFT(low);
  FAST_SHIFT(low); FAST_SHIFT(low); FAST_SHIFT(low); FAST_SHIFT(low);


  //strobe latch line
  bitSet(STROBE_PORT,STROBE_LATCH);
  bitClear(STROBE_PORT,STROBE_LATCH);

}

//short function to set the OE(output enable line of the eeprom)
// attention, this line is LOW - active
inline void set_oe (byte state)
{
  digitalWrite(OE, state);
}
 
//short function to set the CE(chip enable line of the eeprom)
// attention, this line is LOW - active
inline void set_ce (byte state)
{
  digitalWrite(CE, state);
}
 
//short function to set the WE(write enable line of the eeprom)
// attention, this line is LOW - active
inline void set_we (byte state)
{
  
      digitalWrite(WE, state);
}

//short function to set up the programmer for reading
void read_start() {
  //set databus for reading
  data_bus_input();
  //enable chip select
  set_ce(LOW);
  //disable write
  set_we(HIGH);
  //enable output
  set_oe(LOW);
}

//short function to stop reading
void read_end() {
  //disable output
  set_oe(HIGH);
  //disable chip select
  set_ce(HIGH);
}  

//highlevel function to read a byte from a given address
inline byte read_byte(unsigned int address)
{
  //set address bus
  set_address_bus(address);
  //read data
  return read_data_bus();
}

//flag set if we are on the first write pass
boolean firstWritePass = true;
 
//short function to set up the programmer for writing
void write_start() {
  firstWritePass = true;
  //first disable output
  set_oe(HIGH);
  //disable write
  set_we(HIGH);
  //set databus to output
  data_bus_output();
}

//short function to stop writing
void write_end() {
  //set databus to input
  data_bus_input();
}

//highlevel function to write a byte to a given address
inline boolean fast_write(unsigned int address, byte data)
{
  static unsigned int lastAddress = 0;
  static byte lastData = 0;
  
      //this function uses /DATA polling to get the end of the
      //page write cycle. This is much faster than waiting 10ms
  
      //enable chip select
      set_ce(LOW);
  
      //only wait for write if the address page has changed since the last write
      //address page is 64 bytes, so xor the last address with the current one to
      // look for a change.  Don't run on the first pass through.
      //28C64 does not support page writes so we poll every time
      if (((lastAddress ^ address) & 0xFFC0 || chipType == CHIP28C64) && !firstWritePass)
      {
        unsigned long startTime = millis();
  
        //poll data until data matches
        data_bus_input();
        set_oe(LOW);
  
        while(lastData != read_data_bus()) {
          //set timeout here longer than JBurn timeout
          if (millis() - startTime > 3000) return false;
        }
        
        set_oe(HIGH);
        delayMicroseconds(1);
        data_bus_output();
      }
  
      //set address and data for write
      set_address_bus(address);
      write_data_bus(data);
      delayMicroseconds(1);
   
      //strobe write
      set_we(LOW);
      set_we(HIGH);
      //disable chip select
      set_ce(HIGH);
  
      lastAddress = address;
      lastData = data;
      firstWritePass = false;
      

  return true;
}

/*************************************************
 *
 * COMMAND and PARSING functions
 *
 *************************************************/
 
//waits for a string submitted via serial connection
//returns only if linebreak is sent or the buffer is filled
void readCommand() {
  //first clear command buffer
  for(int i=0; i< COMMANDSIZE;i++) cmdbuf[i] = 0;
  //initialize variables
  char c = ' ';
  int idx = 0;
  //now read serial data until linebreak or buffer is full
  do {
    if(Serial.available()) {
      c = Serial.read();
      cmdbuf[idx++] = c;
    }
  }
  while (c != '\n' && idx < (COMMANDSIZE)); //save the last '\0' for string end
  //change last newline to '\0' termination
  cmdbuf[idx - 1] = 0;
}
 
//parse the given command by separating command character and parameters
//at the moment only 5 commands are supported
byte parseCommand() {
  //set ',' to '\0' terminator (command string has a fixed strucure)
  //first string is the command character
  cmdbuf[1]  = 0;
  //second string is startaddress (4 bytes)
  cmdbuf[6]  = 0;
  //third string is endaddress (4 bytes)
  cmdbuf[11] = 0;
  //fourth string is length (2 bytes)
  cmdbuf[14] = 0;
  startAddress=hexWord((cmdbuf+2));
  dataLength=hexWord((cmdbuf+7));
  lineLength=hexByte(cmdbuf+12);
  byte retval = 0;
  switch(cmdbuf[0]) {
  case 'A':
    retval = SET_ADDRESS;
    break;
  case 'R':
    retval = READ_HEX;
    break;
  case 'r':
    retval = READ_BIN;
    break;
  case 'W':
    retval = WRITE_HEX;
    break;
  case 'w':
    retval = WRITE_BIN;
    break;
  case 'V':
    retval = VERSION;
    break;
  case 'C':
    retval = CHIP_TYPE;
    break;
  case 'E':
    retval = CHIP_ERASE;
    break;
  default:
    retval = NOCOMMAND;
    break;
  }
 
  return retval;
}
 
/************************************************************
 * convert a single hex digit (0-9,a-f) to byte
 * @param char c single character (digit)
 * @return byte represented by the digit
 ************************************************************/
inline byte hexDigit(char c)
{
  //use lookup table for char to hex conversion
  const char chartohex[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
                    0, 0, 0, 0, 0, 0, 0,
                    10, 11, 12, 13, 14, 15 };
  return chartohex[c - '0'];
}
 
/************************************************************
 * convert a hex byte (00 - ff) to byte
 * @param c-string with the hex value of the byte
 * @return byte represented by the digits
 ************************************************************/
byte hexByte(char* a)
{
  return ((hexDigit(a[0]) << 4) | hexDigit(a[1]));
}
 
/************************************************************
 * convert a hex word (0000 - ffff) to unsigned int
 * @param c-string with the hex value of the word
 * @return unsigned int represented by the digits
 ************************************************************/
unsigned int hexWord(char* data) {
  return ((hexDigit(data[0]) << 12) |
    (hexDigit(data[1]) << 8) |
    (hexDigit(data[2]) << 4) |
    (hexDigit(data[3])));
}
 
 
/************************************************
 *
 * INPUT / OUTPUT Functions
 *
 *************************************************/
 
 
/**
 * read a data block from eeprom and write out a hex dump
 * of the data to serial connection
 * @param from       start address to read fromm
 * @param to         last address to read from
 * @param linelength how many hex values are written in one line
 **/
void read_block(unsigned int from, unsigned int to, int linelength)
{
  //count the number fo values that are already printed out on the
  //current line
  int               outcount = 0;
  //loop from "from address" to "to address" (included)
  unsigned int address = from;
  read_start();
  do {
    if (outcount == 0) {
      //print out the address at the beginning of the line
      Serial.println();
      Serial.print("0x");
      printAddress(address);
      Serial.print(" : ");
    }
    //print data, separated by a space
    byte data = read_byte(address);
    printByte(data);
    Serial.print(" ");
    outcount = (++outcount % linelength);
  } while (address++ != to);
  read_end();

  //print a newline after the last data line
  Serial.println();
 
}
 
/**
 * read a data block from eeprom and write out the binary data
 * to the serial connection
 * @param from       start address to read fromm
 * @param to         last address to read from
 **/
void read_binblock(unsigned int from, unsigned int to) {
  unsigned int address = from;

  read_start();
  do {
    Serial.write(read_byte(address));
  } while (address++ != to);
  read_end();

  //print out an additional 0-byte as success return code
  Serial.print('\0');
}  
 
/**
 * write a data block to the eeprom
 * @param address  startaddress to write on eeprom
 * @param buffer   data buffer to get the data from
 * @param len      number of bytes to be written
 **/
void write_block(unsigned int address, byte* buffer, int len) {
  write_start();
  for (unsigned int i = 0; i < len; i++) {
    if (fast_write(address++, *buffer++) == false)
      break;
  }
  write_end();
}
 
/**
 * erase entire chip
 **/
void chip_erase() {
  switch (chipType) {
    case CHIP28C64:
      write_start();
      for (unsigned int addr = 0; addr < 0x2000; addr++) {
        if (fast_write(addr, 0xFF) == false)
          break;
      }
      write_end();
      break;
    case CHIP28C256:

      write_start();
     for (unsigned int addr = 0; addr < 0x8000; addr++) {
        if (fast_write(addr, 0xFF) == false)
          break;
      }
      Serial.println('%');
      write_end();
      break;
    default:
      break;
  }
}
 
/**
 * print out a 16 bit word as 4 character hex value
 **/
void printAddress(unsigned int address) {
  if(address < 0x0010) Serial.print("0");
  if(address < 0x0100) Serial.print("0");
  if(address < 0x1000) Serial.print("0");
  Serial.print(address, HEX);
 
}
 
/**
 * print out a byte as 2 character hex value
 **/
void printByte(byte data) {
  if(data < 0x10) Serial.print("0");
  Serial.print(data, HEX);  
}
 
 
 
 
 
/************************************************
 *
 * MAIN
 *
 *************************************************/
void setup() {
  //default to 28C64
  chipType = CHIP28C64;

  //define the shiuftOut Pins as output
  pinMode(DS, OUTPUT);
  pinMode(LATCH, OUTPUT);
  pinMode(CLOCK, OUTPUT);
 
  //define the EEPROM Pins as output
  // take care that they are HIGH
  digitalWrite(OE, HIGH);
  pinMode(OE, OUTPUT);
  digitalWrite(CE, HIGH);
  pinMode(CE, OUTPUT);
  digitalWrite(WE, HIGH);
  pinMode(WE, OUTPUT);

  //set speed of serial connection
 
 Serial.begin(512000);
}
 
/**
 * main loop, that runs invinite times, parsing a given command and
 * executing the given read or write requestes.
 **/
void loop() {
  readCommand();
  byte cmd = parseCommand();
  int bytes = 0;
  switch(cmd) {
  case SET_ADDRESS:
    // Set the address bus to an arbitrary value.
    // Useful for debugging shift-register wiring, byte-order.
    // e.g. A,00FF
    Serial.print("Setting address bus to 0x");
    Serial.println(cmdbuf + 2);
    set_address_bus(startAddress);
    break;
  case READ_HEX:
    //set a default if needed to prevent infinite loop
    if(lineLength==0) lineLength=32;
    endAddress = startAddress + dataLength -1;
    read_block(startAddress,endAddress,lineLength);
    Serial.println('%');
    break;
  case READ_BIN:
    endAddress = startAddress + dataLength -1;
    read_binblock(startAddress,endAddress);
    break;
  case READ_ITL:
    break;
  case WRITE_BIN:
    //take care for max buffer size
    if(dataLength > 1024) dataLength = 1024;
    endAddress = startAddress + dataLength -1;
    while(bytes < dataLength) {
      if(Serial.available()) buffer[bytes++] = Serial.read();
    }    
    write_block(startAddress,buffer,dataLength);
    delayMicroseconds(150);
    Serial.println('%');
    break;
  case WRITE_HEX:
    break;
  case WRITE_ITL:
    break;
  case VERSION:
    Serial.println(VERSIONSTRING);
    break;
  case CHIP_TYPE:
    chipType = startAddress;
    break;
  case CHIP_ERASE:
    chip_erase();
    Serial.println('%');
    break;
  default:
    break;    
  }
}

