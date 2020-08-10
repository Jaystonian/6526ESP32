/*
  6526ESP32 - A virtual WiFi modem and C64/C128 coprocessor 
    for MOS6526 & NodeMCU-32S (Commodore C64 & C128)
    https://github.com/Jaystonian/6526ESP32
    
    To use, follow instructions for StrikeLink WiFi or updated PDF specific to this project.
    C64/C128 CCGMS available here: https://commodore.software/downloads/category/59-ccgms

                            --- based on ---
    
   https://github.com/svenpetersen1965/C64-WiFi-Modem-User-Port    

                            --- based on ---
   https://1200baud.wordpress.com/2017/03/04/build-your-own-9600-baud-c64-wifi-modem-for-20/

   WiFi SIXFOUR - A virtual WiFi modem based on the ESP 8266 chipset
   Copyright (C) 2016 Paul Rickards <rickards@gmail.com>

   based on
   ESP8266 based virtual modem
   Copyright (C) 2016 Jussi Salin <salinjus@gmail.com>

   https://github.com/jsalin/esp8266_modem

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <WiFi.h>
#include "ESP32WebServer.h"
#include <EEPROM.h>
#include <ESPmDNS.h>
#include <time.h>
#include "carts/1541diag.h"
#include "carts/tool64.h"

//#define DEBUG

// These translation tables are not used yet.
static unsigned char petToAscTable[256] = {
0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x14,0x09,0x0d,0x11,0x93,0x0a,0x0e,0x0f,
0x10,0x0b,0x12,0x13,0x08,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,
0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,
0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,
0x40,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,
0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x5b,0x5c,0x5d,0x5e,0x5f,
0xc0,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,
0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0xdb,0xdc,0xdd,0xde,0xdf,
0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,
0x90,0x91,0x92,0x0c,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,
0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,
0x60,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,
0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0x7b,0x7c,0x7d,0x7e,0x7f,
0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf
};

static unsigned char ascToPetTable[256] = {
0x00,0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x14,0x09,0x0d,0x11,0x93,0x0a,0x0e,0x0f,
0x10,0x0b,0x12,0x13,0x08,0x15,0x16,0x17,0x18,0x19,0x1a,0x1b,0x1c,0x1d,0x1e,0x1f,
0x20,0x21,0x22,0x23,0x24,0x25,0x26,0x27,0x28,0x29,0x2a,0x2b,0x2c,0x2d,0x2e,0x2f,
0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,0x3a,0x3b,0x3c,0x3d,0x3e,0x3f,
0x40,0xc1,0xc2,0xc3,0xc4,0xc5,0xc6,0xc7,0xc8,0xc9,0xca,0xcb,0xcc,0xcd,0xce,0xcf,
0xd0,0xd1,0xd2,0xd3,0xd4,0xd5,0xd6,0xd7,0xd8,0xd9,0xda,0x5b,0x5c,0x5d,0x5e,0x5f,
0xc0,0x41,0x42,0x43,0x44,0x45,0x46,0x47,0x48,0x49,0x4a,0x4b,0x4c,0x4d,0x4e,0x4f,
0x50,0x51,0x52,0x53,0x54,0x55,0x56,0x57,0x58,0x59,0x5a,0xdb,0xdc,0xdd,0xde,0xdf,
0x80,0x81,0x82,0x83,0x84,0x85,0x86,0x87,0x88,0x89,0x8a,0x8b,0x8c,0x8d,0x8e,0x8f,
0x90,0x91,0x92,0x0c,0x94,0x95,0x96,0x97,0x98,0x99,0x9a,0x9b,0x9c,0x9d,0x9e,0x9f,
0xa0,0xa1,0xa2,0xa3,0xa4,0xa5,0xa6,0xa7,0xa8,0xa9,0xaa,0xab,0xac,0xad,0xae,0xaf,
0xb0,0xb1,0xb2,0xb3,0xb4,0xb5,0xb6,0xb7,0xb8,0xb9,0xba,0xbb,0xbc,0xbd,0xbe,0xbf,
0x60,0x61,0x62,0x63,0x64,0x65,0x66,0x67,0x68,0x69,0x6a,0x6b,0x6c,0x6d,0x6e,0x6f,
0x70,0x71,0x72,0x73,0x74,0x75,0x76,0x77,0x78,0x79,0x7a,0x7b,0x7c,0x7d,0x7e,0x7f,
0xe0,0xe1,0xe2,0xe3,0xe4,0xe5,0xe6,0xe7,0xe8,0xe9,0xea,0xeb,0xec,0xed,0xee,0xef,
0xf0,0xf1,0xf2,0xf3,0xf4,0xf5,0xf6,0xf7,0xf8,0xf9,0xfa,0xfb,0xfc,0xfd,0xfe,0xff
};

#define VERSIONA 0
#define VERSIONB 1
#define VERSION_ADDRESS 0    // EEPROM address
#define VERSION_LEN     2    // Length in bytes
#define SSID_ADDRESS    2
#define SSID_LEN        32
#define PASS_ADDRESS    34
#define PASS_LEN        63
#define IP_TYPE_ADDRESS 97   // for future use
#define STATIC_IP_ADDRESS 98 // length 4, for future use
#define STATIC_GW       102  // length 4, for future use
#define STATIC_DNS      106  // length 4, for future use
#define STATIC_MASK     110  // length 4, for future use
#define BAUD_ADDRESS    111
#define ECHO_ADDRESS    112
#define SERVER_PORT_ADDRESS 113 // 2 bytes
#define AUTO_ANSWER_ADDRESS 115 // 1 byte
#define TELNET_ADDRESS  116     // 1 byte
#define VERBOSE_ADDRESS 117
#define PET_TRANSLATE_ADDRESS 118
#define FLOW_CONTROL_ADDRESS 119
#define PIN_POLARITY_ADDRESS 120
#define DIAL0_ADDRESS   200
#define DIAL1_ADDRESS   250
#define DIAL2_ADDRESS   300
#define DIAL3_ADDRESS   350
#define DIAL4_ADDRESS   400
#define DIAL5_ADDRESS   450
#define DIAL6_ADDRESS   500
#define DIAL7_ADDRESS   550
#define DIAL8_ADDRESS   600
#define DIAL9_ADDRESS   650
#define BUSY_MSG_ADDRESS 700
#define BUSY_MSG_LEN    80
#define LAST_ADDRESS    780

///////////////////////////////////////////////////////////////////////////////////


#define PortD00 25
#define PortD01 21
#define PortD02 0 
#define PortD03 4
#define PortD04 27
#define PortD05 25
#define PortD06 26
#define PortD07 13

#define PortA0 34
#define PortA1 35
#define PortA2 36
#define PortA3 39
#define PortA4 15
#define PortA5 5
#define PortA6 14

#define PortCLK 22
#define PortRW 32
#define PortCS 3 //CHIPSELECT
#define PortNMI 1
#define PortMODEMENABLE 2 //Blue LED
#define PortGATEENABLE 12 //set low to disable
#define PortMODEM_RTS 18
#define PortMODEM_CTS 19
#define PortMODEM_DCD 23
#define PortMODEM_TXD 17
#define PortMODEM_RXD 16



// Global variables
String build = "20200808162000";
String cmd = "";           // Gather a new AT command to this string from serial
bool cmdMode = true;       // Are we in AT command mode or connected mode
bool callConnected = false;// Are we currently in a call
bool telnet = false;       // Is telnet control code handling enabled
bool verboseResults = false;
//#define DEBUG 1          // Print additional debug information to serial channel
//#undef DEBUG
#define LISTEN_PORT 6400   // Listen to this if not connected. Set to zero to disable.
int tcpServerPort = LISTEN_PORT;
#define RING_INTERVAL 3000 // How often to print RING when having a new incoming connection (ms)
unsigned long lastRingMs = 0; // Time of last "RING" message (millis())
//long myBps;                // What is the current BPS setting
#define MAX_CMD_LENGTH 256 // Maximum length for AT command
char plusCount = 0;        // Go to AT mode at "+++" sequence, that has to be counted
unsigned long plusTime = 0;// When did we last receive a "+++" sequence
#define LED_TIME 15         // How many ms to keep LED on at activity
unsigned long ledTime = 0;
#define TX_BUF_SIZE 256    // Buffer where to read from serial before writing to TCP
// (that direction is very blocking by the ESP TCP stack,
// so we can't do one byte a time.)
uint8_t txBuf[TX_BUF_SIZE];
const int speedDialAddresses[] = { DIAL0_ADDRESS, DIAL1_ADDRESS, DIAL2_ADDRESS, DIAL3_ADDRESS, DIAL4_ADDRESS, DIAL5_ADDRESS, DIAL6_ADDRESS, DIAL7_ADDRESS, DIAL8_ADDRESS, DIAL9_ADDRESS };
String speedDials[10];
const int bauds[] = { 300, 1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200 };
byte serialspeed;
bool echo = true;
bool autoAnswer = false;
String ssid, password, busyMsg;
byte ringCount = 0;
String resultCodes[] = { "OK", "CONNECT", "RING", "NO CARRIER", "ERROR", "", "NO DIALTONE", "BUSY", "NO ANSWER" };
enum resultCodes_t { RR_OK, R_CONNECT, R_RING, R_NOCARRIER, R_ERROR, R_NONE, R_NODIALTONE, R_BUSY, R_NOANSWER };

unsigned long connectTime = 0;
bool petTranslate = false; // Fix PET MCTerm 1.26C Pet->ASCII encoding to actual ASCII
bool hex = false;
enum flowControl_t { F_NONE, F_HARDWARE, F_SOFTWARE };
byte flowControl = F_NONE;      // Use flow control
bool txPaused = false;          // Has flow control asked us to pause?
enum pinPolarity_t { P_INVERTED, P_NORMAL }; // Is LOW (0) or HIGH (1) active?
byte pinPolarity = P_INVERTED;

// Telnet codes
#define DO 0xfd
#define WONT 0xfc
#define WILL 0xfb
#define DONT 0xfe

#define SYSRESET 64738

WiFiClient tcpClient;
WiFiServer tcpServer(tcpServerPort);
ESP32WebServer webServer(80);
MDNSResponder mdns;
HardwareSerial SerialSM(1);




//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool cbCLK = false, cbCLK_OLD = false, cbCLK_NEG = false, cbCLK_POS = false; uint cuCLK_time=0, cuCLK=0;
bool cbRW = false, cbRW_OLD = false, cbRW_NEG = false, cbRW_POS = false; uint cuRW_time=0, cuRW=0;
bool cbCS = false, cbCS_OLD = false, cbCS_NEG = false, cbCS_POS = false; uint cuCS_time=0, cuCS=0;

uint cuWaitCS = 20;
uint cuWaitRW = 20;
uint cuWaitCLK = 20;

#define DATAPAGESIZE 128
byte cnAddress = 0, cnAddress_OLD = 0;
bool cbEnabledModem = false;
bool cbEnabledAddressing = false;
byte cnDataPage[DATAPAGESIZE]; //byte storage for DD80-DDFF
uint cnByteCounter = 0;
uint cnByteDest = 0;
byte cnMode = 0, cnMode_old = 0;
byte cnCommand = 0, cnCommand_old = 0;
const int cnWaitTime = 1000;
uint lastCycle = 0;


void checkStateTransitions(){ // neg/pos get set on edges
  //not all of this is going to be needed but will help with debug
  uint thisCycle = ESP.getCycleCount();  
  
  cbCLK=digitalRead(PortCLK);
  cbRW=digitalRead(PortRW);
  cbCS=digitalRead(PortCS);
  
  if(cbCLK!=cbCLK_OLD){cbCLK_OLD=cbCLK;cbCLK_NEG=!cbCLK;cbCLK_POS=cbCLK;cuCLK_time=thisCycle;}
  if(cbRW!=cbRW_OLD){cbRW_OLD=cbRW;cbRW_NEG=!cbRW;cbRW_POS=cbRW;cuRW_time=thisCycle;}
  if(cbCS!=cbCS_OLD){cbCS_OLD=cbCS;cbCS_NEG=!cbCS;cbCS_POS=cbCS;cuCS_time=thisCycle;}

  cuCLK = thisCycle - cuCLK_time;
  cuRW = thisCycle - cuRW_time;
  cuCS = thisCycle - cuCS_time;
  
}

void enableAddressing(bool bEnable){
  //if(cbEnabledAddressing == bEnable) return;
  cbEnabledAddressing = bEnable;
  if(bEnable){
    enableModem(false);
    digitalWrite(PortGATEENABLE,HIGH);
  }else{
    digitalWrite(PortGATEENABLE,LOW);
  }
}
void enableModem(bool bEnable) {
  //if(cbEnabledModem == bEnable) return;
  cbEnabledModem = bEnable;
  if(bEnable){
    enableAddressing(false);
    digitalWrite(PortMODEMENABLE, HIGH);
    digitalWrite(PortMODEM_RTS, HIGH); // ready to receive data
  }else{
    digitalWrite(PortMODEMENABLE, LOW);
  }

}

void setDataRead()
{
  if(cbRW){ //read means output data
    pinMode(PortD00,OUTPUT);
    pinMode(PortD01,OUTPUT);
    pinMode(PortD02,OUTPUT);
    pinMode(PortD03,OUTPUT);
    pinMode(PortD04,OUTPUT);
    pinMode(PortD05,OUTPUT);
    pinMode(PortD06,OUTPUT);
    pinMode(PortD07,OUTPUT);
  }else{ //write is active-low
    pinMode(PortD00,INPUT);
    pinMode(PortD01,INPUT);
    pinMode(PortD02,INPUT);
    pinMode(PortD03,INPUT);
    pinMode(PortD04,INPUT);
    pinMode(PortD05,INPUT);
    pinMode(PortD06,INPUT);
    pinMode(PortD07,INPUT);
  }
  
}

void putByteData(byte nData){
  //RW must be HIGH before this call
  digitalWrite(PortD07,nData & 128);
  digitalWrite(PortD06,nData & 64);
  digitalWrite(PortD05,nData & 32);
  digitalWrite(PortD04,nData & 16);
  digitalWrite(PortD03,nData & 8);
  digitalWrite(PortD02,nData & 4);
  digitalWrite(PortD01,nData & 2);
  digitalWrite(PortD00,nData & 1);
}
byte getByteData()
{
  //assume RW is LOW before this call
  byte bIn = (digitalRead(PortD07)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD06)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD05)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD04)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD03)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD02)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD01)?1:0);
  bIn = bIn << 1 + (digitalRead(PortD00)?1:0);
}
byte getByteAddress()
{
  byte bIn = 2;
  bIn += (digitalRead(PortA6)?1:0);
  bIn = bIn << 1 + (digitalRead(PortA5)?1:0);
  bIn = bIn << 1 + (digitalRead(PortA4)?1:0);
  bIn = bIn << 1 + (digitalRead(PortA3)?1:0);
  bIn = bIn << 1 + (digitalRead(PortA2)?1:0);
  bIn = bIn << 1 + (digitalRead(PortA1)?1:0);
  bIn = bIn << 1 + (digitalRead(PortA0)?1:0);
}





////////////////////////////////////////////////////////////////////////////////////////////////
String connectTimeString() {
  unsigned long now = millis();
  int secs = (now - connectTime) / 1000;
  int mins = secs / 60;
  int hours = mins / 60;
  String out = "";
  if (hours < 10) out.concat("0");
  out.concat(String(hours));
  out.concat(":");
  if (mins % 60 < 10) out.concat("0");
  out.concat(String(mins % 60));
  out.concat(":");
  if (secs % 60 < 10) out.concat("0");
  out.concat(String(secs % 60));
  return out;
}

void writeSettings() {
  setEEPROM(ssid, SSID_ADDRESS, SSID_LEN);
  setEEPROM(password, PASS_ADDRESS, PASS_LEN);
  setEEPROM(busyMsg, BUSY_MSG_ADDRESS, BUSY_MSG_LEN);

  EEPROM.write(BAUD_ADDRESS, serialspeed);
  EEPROM.write(ECHO_ADDRESS, byte(echo));
  EEPROM.write(AUTO_ANSWER_ADDRESS, byte(autoAnswer));
  EEPROM.write(SERVER_PORT_ADDRESS, highByte(tcpServerPort));
  EEPROM.write(SERVER_PORT_ADDRESS + 1, lowByte(tcpServerPort));
  EEPROM.write(TELNET_ADDRESS, byte(telnet));
  EEPROM.write(VERBOSE_ADDRESS, byte(verboseResults));
  EEPROM.write(PET_TRANSLATE_ADDRESS, byte(petTranslate));
  EEPROM.write(FLOW_CONTROL_ADDRESS, byte(flowControl));
  EEPROM.write(PIN_POLARITY_ADDRESS, byte(pinPolarity));

  for (int i = 0; i < 10; i++) {
    setEEPROM(speedDials[i], speedDialAddresses[i], 50);
  }
  EEPROM.commit();
}

void readSettings() {
  echo = EEPROM.read(ECHO_ADDRESS);
  autoAnswer = EEPROM.read(AUTO_ANSWER_ADDRESS);
  // serialspeed = EEPROM.read(BAUD_ADDRESS);

  ssid = getEEPROM(SSID_ADDRESS, SSID_LEN);
  password = getEEPROM(PASS_ADDRESS, PASS_LEN);
  busyMsg = getEEPROM(BUSY_MSG_ADDRESS, BUSY_MSG_LEN);
  tcpServerPort = word(EEPROM.read(SERVER_PORT_ADDRESS), EEPROM.read(SERVER_PORT_ADDRESS + 1));
  telnet = EEPROM.read(TELNET_ADDRESS);
  verboseResults = EEPROM.read(VERBOSE_ADDRESS);
  petTranslate = EEPROM.read(PET_TRANSLATE_ADDRESS);
  flowControl = EEPROM.read(FLOW_CONTROL_ADDRESS);
  pinPolarity = EEPROM.read(PIN_POLARITY_ADDRESS);

  for (int i = 0; i < 10; i++) {
    speedDials[i] = getEEPROM(speedDialAddresses[i], 50);
  }
}

void defaultEEPROM() {
  EEPROM.write(VERSION_ADDRESS, VERSIONA);
  EEPROM.write(VERSION_ADDRESS + 1, VERSIONB);

  setEEPROM("", SSID_ADDRESS, SSID_LEN);
  setEEPROM("", PASS_ADDRESS, PASS_LEN);
  setEEPROM("d", IP_TYPE_ADDRESS, 1);
  EEPROM.write(SERVER_PORT_ADDRESS, highByte(LISTEN_PORT));
  EEPROM.write(SERVER_PORT_ADDRESS + 1, lowByte(LISTEN_PORT));

  EEPROM.write(BAUD_ADDRESS, 0x00);
  EEPROM.write(ECHO_ADDRESS, 0x01);
  EEPROM.write(AUTO_ANSWER_ADDRESS, 0x01);
  EEPROM.write(TELNET_ADDRESS, 0x00);
  EEPROM.write(VERBOSE_ADDRESS, 0x01);
  EEPROM.write(PET_TRANSLATE_ADDRESS, 0x00);
  EEPROM.write(FLOW_CONTROL_ADDRESS, 0x00);
  EEPROM.write(PIN_POLARITY_ADDRESS, 0x01);

  setEEPROM("bbs.fozztexx.com:23", speedDialAddresses[0], 50);
  setEEPROM("cottonwoodbbs.dyndns.org:6502", speedDialAddresses[1], 50);
  setEEPROM("borderlinebbs.dyndns.org:6400", speedDialAddresses[2], 50);
  setEEPROM("particlesbbs.dyndns.org:6400", speedDialAddresses[3], 50);
  setEEPROM("reflections.servebbs.com:23", speedDialAddresses[4], 50);
  setEEPROM("heatwavebbs.com:9640", speedDialAddresses[5], 50);

  for (int i = 5; i < 10; i++) {
    setEEPROM("", speedDialAddresses[i], 50);
  }

  setEEPROM("SORRY, SYSTEM IS CURRENTLY BUSY. PLEASE TRY AGAIN LATER.", BUSY_MSG_ADDRESS, BUSY_MSG_LEN);
  EEPROM.commit();
}

String getEEPROM(int startAddress, int len) {
  String myString;

  for (int i = startAddress; i < startAddress + len; i++) {
    if (EEPROM.read(i) == 0x00) {
      break;
    }
    myString += char(EEPROM.read(i));
    //SerialSM.print(char(EEPROM.read(i)));
  }
  //SerialSM.println();
  return myString;
}

void setEEPROM(String inString, int startAddress, int maxLen) {
  for (int i = startAddress; i < inString.length() + startAddress; i++) {
    EEPROM.write(i, inString[i - startAddress]);
    //SerialSM.print(i, DEC); SerialSM.print(": "); SerialSM.println(inString[i - startAddress]);
    //if (EEPROM.read(i) != inString[i - startAddress]) { SerialSM.print(" (!)"); }
    //SerialSM.println();
  }
  // null pad the remainder of the memory space
  for (int i = inString.length() + startAddress; i < maxLen + startAddress; i++) {
    EEPROM.write(i, 0x00);
    //SerialSM.print(i, DEC); SerialSM.println(": 0x00");
  }
}

void sendResult(int resultCode) {
  SerialSM.print("\r\n");
  if (verboseResults == 0) {
    SerialSM.println(resultCode);
    return;
  }
  if (resultCode == R_CONNECT) {
    SerialSM.print(String(resultCodes[R_CONNECT]) + " " + String(bauds[serialspeed]));
  } else if (resultCode == R_NOCARRIER) {
    SerialSM.print(String(resultCodes[R_NOCARRIER]) + " (" + connectTimeString() + ")");
  } else {
    SerialSM.print(String(resultCodes[resultCode]));
  }
  SerialSM.print("\r\n");
}

void sendString(String msg) {
  SerialSM.print("\r\n");
  SerialSM.print(msg);
  SerialSM.print("\r\n");
}

// Hold for 5 seconds to switch to 300 baud
// Slow flash: keep holding
// Fast flash: let go
//JWD:replace functionality for switch with configuration code
int checkButton() {
  /*
  long time = millis();
  while (digitalRead(SWITCH_PIN) == LOW && millis() - time < 5000) {
    delay(250);
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    yield();
  }
  if (millis() - time > 5000) {
    SerialSM.flush();
    SerialSM.end();
    serialspeed = 0;
    delay(100);
    SerialSM.begin(bauds[serialspeed]);
    sendResult(RR_OK);
    while (digitalRead(SWITCH_PIN) == LOW) {
      delay(50);
      digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      yield();
    }
    return 1;
  } else {
    return 0;
  }*/
  return 0;
}

void connectWiFi() {
  if (ssid == "" || password == "") {
    SerialSM.println("CONFIGURE SSID AND PASSWORD. TYPE AT? FOR HELP.");
    return;
  }
  WiFi.begin(ssid.c_str(), password.c_str());
  SerialSM.print("\nCONNECTING TO SSID "); SerialSM.print(ssid);
  uint8_t i = 0;
  while (WiFi.status() != WL_CONNECTED && i++ < 20) {
    //digitalWrite(LED_PIN, LOW);
    delay(250);
    //digitalWrite(LED_PIN, HIGH);
    delay(250);
    SerialSM.print(".");
  }
  SerialSM.println();
  if (i == 21) {
    SerialSM.print("COULD NOT CONNET TO "); SerialSM.println(ssid);
    WiFi.disconnect();
    updateLed();
  } else {
    SerialSM.print("CONNECTED TO "); SerialSM.println(WiFi.SSID());
    SerialSM.print("IP ADDRESS: "); SerialSM.println(WiFi.localIP());
    updateLed();
  }
}

void updateLed() {
  if (WiFi.status() == WL_CONNECTED) {
    //digitalWrite(LED_PIN, LOW);  // on
  } else {
    //digitalWrite(LED_PIN, HIGH); //off
  }
}

void disconnectWiFi() {
  WiFi.disconnect();
  updateLed();
}

void setBaudRate(int inSpeed) {
  if (inSpeed == 0) {
    sendResult(R_ERROR);
    return;
  }
  int foundBaud = -1;
  for (int i = 0; i < sizeof(bauds); i++) {
    if (inSpeed == bauds[i]) {
      foundBaud = i;
      break;
    }
  }
  // requested baud rate not found, return error
  if (foundBaud == -1) {
    sendResult(R_ERROR);
    return;
  }
  if (foundBaud == serialspeed) {
    sendResult(RR_OK);
    return;
  }
  SerialSM.print("SWITCHING SERIAL PORT TO ");
  SerialSM.print(inSpeed);
  SerialSM.println(" IN 5 SECONDS");
  delay(5000);
  SerialSM.end();
  delay(200);
  
  //SerialSM.begin(bauds[foundBaud]);
  SerialSM.begin(bauds[foundBaud], SERIAL_8N1, 17, 16);
  
  serialspeed = foundBaud;
  delay(200);
  sendResult(RR_OK);
}

void setCarrier(byte carrier) {
  if (pinPolarity == P_NORMAL) carrier = !carrier;
  digitalWrite(PortMODEM_DCD, carrier);
}

void displayNetworkStatus() {
  SerialSM.print("WIFI STATUS: ");
  if (WiFi.status() == WL_CONNECTED) {
    SerialSM.println("CONNECTED");
  }
  if (WiFi.status() == WL_IDLE_STATUS) {
    SerialSM.println("OFFLINE");
  }
  if (WiFi.status() == WL_CONNECT_FAILED) {
    SerialSM.println("CONNECT FAILED");
  }
  if (WiFi.status() == WL_NO_SSID_AVAIL) {
    SerialSM.println("SSID UNAVAILABLE");
  }
  if (WiFi.status() == WL_CONNECTION_LOST) {
    SerialSM.println("CONNECTION LOST");
  }
  if (WiFi.status() == WL_DISCONNECTED) {
    SerialSM.println("DISCONNECTED");
  }
  if (WiFi.status() == WL_SCAN_COMPLETED) {
    SerialSM.println("SCAN COMPLETED");
  }
  yield();

  SerialSM.print("SSID.......: "); SerialSM.println(WiFi.SSID());

  //  SerialSM.print("ENCRYPTION: ");
  //  switch(WiFi.encryptionType()) {
  //    case 2:
  //      SerialSM.println("TKIP (WPA)");
  //      break;
  //    case 5:
  //      SerialSM.println("WEP");
  //      break;
  //    case 4:
  //      SerialSM.println("CCMP (WPA)");
  //      break;
  //    case 7:
  //      SerialSM.println("NONE");
  //      break;
  //    case 8:
  //      SerialSM.println("AUTO");
  //      break;
  //    default:
  //      SerialSM.println("UNKNOWN");
  //      break;
  //  }

  byte mac[6];
  WiFi.macAddress(mac);
  SerialSM.print("MAC ADDRESS: ");
  SerialSM.print(mac[0], HEX);
  SerialSM.print(":");
  SerialSM.print(mac[1], HEX);
  SerialSM.print(":");
  SerialSM.print(mac[2], HEX);
  SerialSM.print(":");
  SerialSM.print(mac[3], HEX);
  SerialSM.print(":");
  SerialSM.print(mac[4], HEX);
  SerialSM.print(":");
  SerialSM.println(mac[5], HEX);
  yield();

  SerialSM.print("IP ADDRESS.: "); SerialSM.println(WiFi.localIP()); yield();
  SerialSM.print("GATEWAY....: "); SerialSM.println(WiFi.gatewayIP()); yield();
  SerialSM.print("SUBNET MASK: "); SerialSM.println(WiFi.subnetMask()); yield();
  SerialSM.print("SERVER PORT: "); SerialSM.println(tcpServerPort); yield();
  SerialSM.print("WEB CONFIG.: HTTP://"); SerialSM.println(WiFi.localIP()); yield();
  SerialSM.print("CALL STATUS: "); yield();
  if (callConnected) {
    SerialSM.print("CONNECTED TO "); SerialSM.println(ipToString(tcpClient.remoteIP())); yield();
    SerialSM.print("CALL LENGTH: "); SerialSM.println(connectTimeString()); yield();
  } else {
    SerialSM.println("NOT CONNECTED");
  }
}

void displayCurrentSettings() {
  SerialSM.println("ACTIVE PROFILE:"); yield();
  SerialSM.print("BAUD: "); SerialSM.println(bauds[serialspeed]); yield();
  SerialSM.print("SSID: "); SerialSM.println(ssid); yield();
  SerialSM.print("PASS: "); SerialSM.println(password); yield();
  //SerialSM.print("SERVER TCP PORT: "); SerialSM.println(tcpServerPort); yield();
  SerialSM.print("BUSY MSG: "); SerialSM.println(busyMsg); yield();
  SerialSM.print("E"); SerialSM.print(echo); SerialSM.print(" "); yield();
  SerialSM.print("V"); SerialSM.print(verboseResults); SerialSM.print(" "); yield();
  SerialSM.print("&K"); SerialSM.print(flowControl); SerialSM.print(" "); yield();
  SerialSM.print("&P"); SerialSM.print(pinPolarity); SerialSM.print(" "); yield();
  SerialSM.print("NET"); SerialSM.print(telnet); SerialSM.print(" "); yield();
  SerialSM.print("PET"); SerialSM.print(petTranslate); SerialSM.print(" "); yield();
  SerialSM.print("S0:"); SerialSM.print(autoAnswer); SerialSM.print(" "); yield();
  SerialSM.println(); yield();

  SerialSM.println("SPEED DIAL:");
  for (int i = 0; i < 10; i++) {
    SerialSM.print(i); SerialSM.print(": "); SerialSM.println(speedDials[i]);
    yield();
  }
  SerialSM.println();
}

void displayStoredSettings() {
  SerialSM.println("STORED PROFILE:"); yield();
  SerialSM.print("BAUD: "); SerialSM.println(bauds[EEPROM.read(BAUD_ADDRESS)]); yield();
  SerialSM.print("SSID: "); SerialSM.println(getEEPROM(SSID_ADDRESS, SSID_LEN)); yield();
  SerialSM.print("PASS: "); SerialSM.println(getEEPROM(PASS_ADDRESS, PASS_LEN)); yield();
  //SerialSM.print("SERVER TCP PORT: "); SerialSM.println(word(EEPROM.read(SERVER_PORT_ADDRESS), EEPROM.read(SERVER_PORT_ADDRESS+1))); yield();
  SerialSM.print("BUSY MSG: "); SerialSM.println(getEEPROM(BUSY_MSG_ADDRESS, BUSY_MSG_LEN)); yield();
  SerialSM.print("E"); SerialSM.print(EEPROM.read(ECHO_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.print("V"); SerialSM.print(EEPROM.read(VERBOSE_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.print("&K"); SerialSM.print(EEPROM.read(FLOW_CONTROL_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.print("&P"); SerialSM.print(EEPROM.read(PIN_POLARITY_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.print("NET"); SerialSM.print(EEPROM.read(TELNET_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.print("PET"); SerialSM.print(EEPROM.read(PET_TRANSLATE_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.print("S0:"); SerialSM.print(EEPROM.read(AUTO_ANSWER_ADDRESS)); SerialSM.print(" "); yield();
  SerialSM.println(); yield();

  SerialSM.println("STORED SPEED DIAL:");
  for (int i = 0; i < 10; i++) {
    SerialSM.print(i); SerialSM.print(": "); SerialSM.println(getEEPROM(speedDialAddresses[i], 50));
    yield();
  }
  SerialSM.println();
}

void waitForSpace() {
  SerialSM.print("PRESS SPACE");
  char c = 0;
  while (c != 0x20) {
    if (SerialSM.available() > 0) {
      c = SerialSM.read();
      if (petTranslate == true){
        if (c > 127) c-= 128;
      }
    }
  }
  SerialSM.print("\r");
}

void displayHelp() {
  welcome();
  SerialSM.println("AT COMMAND SUMMARY:"); yield();
  SerialSM.println("DIAL HOST.....: ATDTHOST:PORT"); yield();
  SerialSM.println("SPEED DIAL....: ATDSN (N=0-9)"); yield();
  SerialSM.println("SET SPEED DIAL: AT&ZN=HOST:PORT (N=0-9)"); yield();
  SerialSM.println("HANDLE TELNET.: ATNETN (N=0,1)"); yield();
  SerialSM.println("PET MCTERM TR.: ATPETN (N=0,1)"); yield();
  SerialSM.println("NETWORK INFO..: ATI"); yield();
  SerialSM.println("HTTP GET......: ATGET<URL>"); yield();
  //SerialSM.println("SERVER PORT...: AT$SP=N (N=1-65535)"); yield();
  SerialSM.println("AUTO ANSWER...: ATS0=N (N=0,1)"); yield();
  SerialSM.println("SET BUSY MSG..: AT$BM=YOUR BUSY MESSAGE"); yield();
  SerialSM.println("LOAD NVRAM....: ATZ"); yield();
  SerialSM.println("SAVE TO NVRAM.: AT&W"); yield();
  SerialSM.println("SHOW SETTINGS.: AT&V"); yield();
  SerialSM.println("FACT. DEFAULTS: AT&F"); yield();
  SerialSM.println("PIN POLARITY..: AT&PN (N=0/INV,1/NORM)"); yield();
  SerialSM.println("ECHO OFF/ON...: ATE0 / ATE1"); yield();
  SerialSM.println("VERBOSE OFF/ON: ATV0 / ATV1"); yield();
  SerialSM.println("SET SSID......: AT$SSID=WIFISSID"); yield();
  SerialSM.println("SET PASSWORD..: AT$PASS=WIFIPASSWORD"); yield();
  SerialSM.println("SET BAUD RATE.: AT$SB=N (3,12,24,48,96"); yield();
  SerialSM.println("                192,384,576,1152)*100"); yield();
  waitForSpace();
  SerialSM.println("FLOW CONTROL..: AT&KN (N=0/N,1/HW,2/SW)"); yield();
  SerialSM.println("WIFI OFF/ON...: ATC0 / ATC1"); yield();
  SerialSM.println("HANGUP........: ATH"); yield();
  SerialSM.println("ENTER CMD MODE: +++"); yield();
  SerialSM.println("EXIT CMD MODE.: ATO"); yield();
  SerialSM.println("QUERY MOST COMMANDS FOLLOWED BY '?'"); yield();
}

void storeSpeedDial(byte num, String location) {
  //if (num < 0 || num > 9) { return; }
  speedDials[num] = location;
  //SerialSM.print("STORED "); SerialSM.print(num); SerialSM.print(": "); SerialSM.println(location);
}

void welcome() {
  SerialSM.println();
  SerialSM.println("6526ESP32 BUILD " + build + " BY JAYSTONIAN");
  SerialSM.println("BASED ON WIFI SIXFOUR BY @PAULRICKARDS");
  SerialSM.println("BASED ON GITHUB.COM/JSALIN/ESP8266_MODEM");
}

/**
   Arduino main init function
*/
void setup() {

/////////////////////////////////////////

#ifdef DEBUG
  Serial.begin(115200);
  yield();
  Serial.printf("Debug Mode Starting...");
  //return;
#endif

  pinMode(PortMODEMENABLE,OUTPUT);
  pinMode(PortGATEENABLE,OUTPUT);
  enableModem(false);
  enableAddressing(false);
  
  pinMode(PortCLK,INPUT);
  pinMode(PortNMI,INPUT);
  pinMode(PortRW,INPUT);
  pinMode(PortCS,INPUT);

  pinMode(PortD00,INPUT);
  pinMode(PortD01,INPUT);
  pinMode(PortD02,INPUT);
  pinMode(PortD03,INPUT);
  pinMode(PortD04,INPUT);
  pinMode(PortD05,INPUT);
  pinMode(PortD06,INPUT);
  pinMode(PortD07,INPUT);
  
  pinMode(PortA0,INPUT);
  pinMode(PortA1,INPUT);
  pinMode(PortA2,INPUT);
  pinMode(PortA3,INPUT);
  pinMode(PortA4,INPUT);
  pinMode(PortA5,INPUT);
  pinMode(PortA6,INPUT);

  //pinMode(PortMODEM_TXD, OUTPUT); //unnecessary?
  //pinMode(PortMODEM_RXD, INPUT); //unnecessary?
/////////////////////////////////////////


  
  //pinMode(LED_PIN, OUTPUT);
  //digitalWrite(LED_PIN, HIGH); // off
  //pinMode(SWITCH_PIN, INPUT);
  //digitalWrite(SWITCH_PIN, HIGH);
  pinMode(PortMODEM_DCD, OUTPUT);
  pinMode(PortMODEM_RTS, OUTPUT);
  //digitalWrite(PortMODEM_RTS, HIGH); // ready to receive data
  pinMode(PortMODEM_CTS, INPUT);
  //digitalWrite(PortMODEM_CTS, HIGH); // pull up

  
  setCarrier(false);

  yield();

  EEPROM.begin(LAST_ADDRESS + 1);
  delay(10);

  if (EEPROM.read(VERSION_ADDRESS) != VERSIONA || EEPROM.read(VERSION_ADDRESS + 1) != VERSIONB) {
    defaultEEPROM();
  }

  readSettings();
  // Fetch baud rate from EEPROM
  serialspeed = EEPROM.read(BAUD_ADDRESS);
  // Check if it's out of bounds-- we have to be able to talk
  if (serialspeed < 0 || serialspeed > sizeof(bauds)) {
    serialspeed = 0;
  }

  //SerialSM.begin(bauds[serialspeed]);
  SerialSM.begin(bauds[serialspeed], SERIAL_8N1, 17, 16);

  if(cbEnabledModem){ setupWifi(); return;}
  enableAddressing(true);

}

void setupWifi(){
  char c;
  //unsigned long startMillis = millis();
  //while (c != 8 && c != 127 && c!= 20) { // Check for the backspace key to begin
  //while (c != 32) { // Check for space to begin
  while (c != 0x0a && c != 0x0d) {
    if (SerialSM.available() > 0) {
      c = SerialSM.read();
      if (petTranslate == true){
        if (c > 127) c-= 128;
      }
    }
    if (checkButton() == 1) {
      break; // button pressed, we're setting to 300 baud and moving on
    }
    //if (millis() - startMillis > 2000) {
      //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
      //startMillis = millis();
    //}
    yield();
  }

  enableModem(true);

  welcome();
  if (tcpServerPort > 0) tcpServer.begin(); //must wait for active WiFi first, I think.. ESP32 aborts on this line.
  WiFi.mode(WIFI_STA);
  connectWiFi();
  sendResult(RR_OK);

  //tcpServer(tcpServerPort); // can't start tcpServer inside a function-- must live outside

  //digitalWrite(LED_PIN, LOW); // on

  webServer.on("/", handleRoot);
  webServer.on("/ath", handleWebHangUp);
  webServer.begin();
  //mdns.begin("C64WiFi", WiFi.localIP());
  mdns.begin("C64WiFi");
}

String ipToString(IPAddress ip) {
  char s[16];
  sprintf(s, "%d.%d.%d.%d", ip[0], ip[1], ip[2], ip[3]);
  return s;
}

void hangUp() {
  tcpClient.stop();
  callConnected = false;
  setCarrier(callConnected);
  sendResult(R_NOCARRIER);
  connectTime = 0;
}

void handleWebHangUp() {
  String t = "NO CARRIER (" + connectTimeString() + ")";
  hangUp();
  webServer.send(200, "text/plain", t);
}

void handleRoot() {
  String page = "WIFI STATUS: ";
  if (WiFi.status() == WL_CONNECTED) {
    page.concat("CONNECTED");
  }
  if (WiFi.status() == WL_IDLE_STATUS) {
    page.concat("OFFLINE");
  }
  if (WiFi.status() == WL_CONNECT_FAILED) {
    page.concat("CONNECT FAILED");
  }
  if (WiFi.status() == WL_NO_SSID_AVAIL) {
    page.concat("SSID UNAVAILABLE");
  }
  if (WiFi.status() == WL_CONNECTION_LOST) {
    page.concat("CONNECTION LOST");
  }
  if (WiFi.status() == WL_DISCONNECTED) {
    page.concat("DISCONNECTED");
  }
  if (WiFi.status() == WL_SCAN_COMPLETED) {
    page.concat("SCAN COMPLETED");
  }
  yield();
  page.concat("\nSSID.......: " + WiFi.SSID());

  byte mac[6];
  WiFi.macAddress(mac);
  page.concat("\nMAC ADDRESS: ");
  page.concat(String(mac[0], HEX));
  page.concat(":");
  page.concat(String(mac[1], HEX));
  page.concat(":");
  page.concat(String(mac[2], HEX));
  page.concat(":");
  page.concat(String(mac[3], HEX));
  page.concat(":");
  page.concat(String(mac[4], HEX));
  page.concat(":");
  page.concat(String(mac[5], HEX));
  yield();

  page.concat("\nIP ADDRESS.: "); page.concat(ipToString(WiFi.localIP()));
  page.concat("\nGATEWAY....: "); page.concat(ipToString(WiFi.gatewayIP()));
  yield();

  page.concat("\nSUBNET MASK: "); page.concat(ipToString(WiFi.subnetMask()));
  yield();
  page.concat("\nSERVER PORT: "); page.concat(tcpServerPort);
  page.concat("\nCALL STATUS: ");
  if (callConnected) {
    page.concat("CONNECTED TO ");
    page.concat(ipToString(tcpClient.remoteIP()));
    page.concat("\nCALL LENGTH: "); page.concat(connectTimeString()); yield();
  } else {
    page.concat("NOT CONNECTED");
  }
  page.concat("\n");
  webServer.send(200, "text/plain", page);
  delay(100);
}

/**
   Turn on the LED and store the time, so the LED will be shortly after turned off
*/
void led_on()
{
  //digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  ledTime = millis();
}

void answerCall() {
  tcpClient = tcpServer.available();
  tcpClient.setNoDelay(true); // try to disable naggle
  //tcpServer.stop();
  sendResult(R_CONNECT);
  connectTime = millis();
  cmdMode = false;
  callConnected = true;
  setCarrier(callConnected);
  SerialSM.flush();
}

void handleIncomingConnection() {
  if (callConnected == 1 || (autoAnswer == false && ringCount > 3)) {
    // We're in a call already or didn't answer the call after three rings
    // We didn't answer the call. Notify our party we're busy and disconnect
    ringCount = lastRingMs = 0;
    WiFiClient anotherClient = tcpServer.available();
    anotherClient.print(busyMsg);
    anotherClient.print("\r\n");
    anotherClient.print("CURRENT CALL LENGTH: ");
    anotherClient.print(connectTimeString());
    anotherClient.print("\r\n");
    anotherClient.print("\r\n");
    anotherClient.flush();
    anotherClient.stop();
    return;
  }

  if (autoAnswer == false) {
    if (millis() - lastRingMs > 6000 || lastRingMs == 0) {
      lastRingMs = millis();
      sendResult(R_RING);
      ringCount++;
    }
    return;
  }

  if (autoAnswer == true) {
    WiFiClient tempClient = tcpServer.available(); // this is the key to keeping the connection open
    tcpClient = tempClient; // hand over the new connection to the global client
    tempClient.stop();   // stop the temporary one
    sendString(String("RING ") + ipToString(tcpClient.remoteIP()));
    delay(1000);
    sendResult(R_CONNECT);
    connectTime = millis();
    cmdMode = false;
    tcpClient.flush();
    callConnected = true;
    setCarrier(callConnected);
  }
}

void dialOut(String upCmd) {
  // Can't place a call while in a call
  if (callConnected) {
    sendResult(R_ERROR);
    return;
  }
  String host, port;
  int portIndex;
  // Dialing a stored number
  if (upCmd.indexOf("ATDS") == 0) {
    byte speedNum = upCmd.substring(4, 5).toInt();
    portIndex = speedDials[speedNum].indexOf(':');
    if (portIndex != -1) {
      host = speedDials[speedNum].substring(0, portIndex);
      port = speedDials[speedNum].substring(portIndex + 1);
    } else {
      port = "23";
    }
  } else {
    // Dialing an ad-hoc number
    int portIndex = cmd.indexOf(":");
    if (portIndex != -1)
    {
      host = cmd.substring(4, portIndex);
      port = cmd.substring(portIndex + 1, cmd.length());
    }
    else
    {
      host = cmd.substring(4, cmd.length());
      port = "23"; // Telnet default
    }
  }
  host.trim(); // remove leading or trailing spaces
  port.trim();
  SerialSM.print("DIALING "); SerialSM.print(host); SerialSM.print(":"); SerialSM.println(port);
  char *hostChr = new char[host.length() + 1];
  host.toCharArray(hostChr, host.length() + 1);
  int portInt = port.toInt();
  tcpClient.setNoDelay(true); // Try to disable naggle
  if (tcpClient.connect(hostChr, portInt))
  {
    tcpClient.setNoDelay(true); // Try to disable naggle
    sendResult(R_CONNECT);
    connectTime = millis();
    cmdMode = false;
    SerialSM.flush();
    callConnected = true;
    setCarrier(callConnected);
    //if (tcpServerPort > 0) tcpServer.stop();
  }
  else
  {
    sendResult(R_NOANSWER);
    callConnected = false;
    setCarrier(callConnected);
  }
  delete hostChr;
}

/**
   Perform a command given in command mode
*/
void command()
{
  cmd.trim();
  if (cmd == "") return;
  SerialSM.println();
  String upCmd = cmd;
  upCmd.toUpperCase();

  /**** Just AT ****/
  if (upCmd == "AT") sendResult(RR_OK);

  /**** Dial to host ****/
  else if ((upCmd.indexOf("ATDT") == 0) || (upCmd.indexOf("ATDP") == 0) || (upCmd.indexOf("ATDI") == 0) || (upCmd.indexOf("ATDS") == 0))
  {
    dialOut(upCmd);
  }

  /**** Change telnet mode ****/
  else if (upCmd == "ATNET0")
  {
    telnet = false;
    sendResult(RR_OK);
  }
  else if (upCmd == "ATNET1")
  {
    telnet = true;
    sendResult(RR_OK);
  }

  else if (upCmd == "ATNET?") {
    SerialSM.println(String(telnet));
    sendResult(RR_OK);
  }

  /**** Answer to incoming connection ****/
  else if ((upCmd == "ATA") && tcpServer.hasClient()) {
    answerCall();
  }

  /**** Display Help ****/
  else if (upCmd == "AT?" || upCmd == "ATHELP") {
    displayHelp();
    sendResult(RR_OK);
  }

  /**** Reset, reload settings from EEPROM ****/
  else if (upCmd == "ATZ") {
    readSettings();
    sendResult(RR_OK);
  }

  /**** Disconnect WiFi ****/
  else if (upCmd == "ATC0") {
    disconnectWiFi();
    sendResult(RR_OK);
  }

  /**** Connect WiFi ****/
  else if (upCmd == "ATC1") {
    connectWiFi();
    sendResult(RR_OK);
  }

  /**** Control local echo in command mode ****/
  else if (upCmd.indexOf("ATE") == 0) {
    if (upCmd.substring(3, 4) == "?") {
      sendString(String(echo));
      sendResult(RR_OK);
    }
    else if (upCmd.substring(3, 4) == "0") {
      echo = 0;
      sendResult(RR_OK);
    }
    else if (upCmd.substring(3, 4) == "1") {
      echo = 1;
      sendResult(RR_OK);
    }
    else {
      sendResult(R_ERROR);
    }
  }

  /**** Control verbosity ****/
  else if (upCmd.indexOf("ATV") == 0) {
    if (upCmd.substring(3, 4) == "?") {
      sendString(String(verboseResults));
      sendResult(RR_OK);
    }
    else if (upCmd.substring(3, 4) == "0") {
      verboseResults = 0;
      sendResult(RR_OK);
    }
    else if (upCmd.substring(3, 4) == "1") {
      verboseResults = 1;
      sendResult(RR_OK);
    }
    else {
      sendResult(R_ERROR);
    }
  }

  /**** Control pin polarity of CTS, RTS, DCD ****/
  else if (upCmd.indexOf("AT&P") == 0) {
    if (upCmd.substring(4, 5) == "?") {
      sendString(String(pinPolarity));
      sendResult(RR_OK);
    }
    else if (upCmd.substring(4, 5) == "0") {
      pinPolarity = P_INVERTED;
      sendResult(RR_OK);
      setCarrier(callConnected);
    }
    else if (upCmd.substring(4, 5) == "1") {
      pinPolarity = P_NORMAL;
      sendResult(RR_OK);
      setCarrier(callConnected);
    }
    else {
      sendResult(R_ERROR);
    }
  }

  /**** Control Flow Control ****/
  else if (upCmd.indexOf("AT&K") == 0) {
    if (upCmd.substring(4, 5) == "?") {
      sendString(String(flowControl));
      sendResult(RR_OK);
    }
    else if (upCmd.substring(4, 5) == "0") {
      flowControl = 0;
      sendResult(RR_OK);
    }
    else if (upCmd.substring(4, 5) == "1") {
      flowControl = 1;
      sendResult(RR_OK);
    }
    else if (upCmd.substring(4, 5) == "2") {
      flowControl = 2;
      sendResult(RR_OK);
    }
    else {
      sendResult(R_ERROR);
    }
  }

  /**** Set current baud rate ****/
  else if (upCmd.indexOf("AT$SB=") == 0) {
    setBaudRate(upCmd.substring(6).toInt());
  }

  /**** Display current baud rate ****/
  else if (upCmd.indexOf("AT$SB?") == 0) {
    sendString(String(bauds[serialspeed]));;
  }

  /**** Set busy message ****/
  else if (upCmd.indexOf("AT$BM=") == 0) {
    busyMsg = cmd.substring(6);
    sendResult(RR_OK);
  }

  /**** Display busy message ****/
  else if (upCmd.indexOf("AT$BM?") == 0) {
    sendString(busyMsg);
    sendResult(RR_OK);
  }

  /**** Display Network settings ****/
  else if (upCmd == "ATI") {
    displayNetworkStatus();
    sendResult(RR_OK);
  }

  /**** Display profile settings ****/
  else if (upCmd == "AT&V") {
    displayCurrentSettings();
    waitForSpace();
    displayStoredSettings();
    sendResult(RR_OK);
  }

  /**** Save (write) current settings to EEPROM ****/
  else if (upCmd == "AT&W") {
    writeSettings();
    sendResult(RR_OK);
  }

  /**** Set or display a speed dial number ****/
  else if (upCmd.indexOf("AT&Z") == 0) {
    byte speedNum = upCmd.substring(4, 5).toInt();
    if (speedNum >= 0 && speedNum <= 9) {
      if (upCmd.substring(5, 6) == "=") {
        String speedDial = cmd;
        storeSpeedDial(speedNum, speedDial.substring(6));
        sendResult(RR_OK);
      }
      if (upCmd.substring(5, 6) == "?") {
        sendString(speedDials[speedNum]);
        sendResult(RR_OK);
      }
    } else {
      sendResult(R_ERROR);
    }
  }

  /**** Set WiFi SSID ****/
  else if (upCmd.indexOf("AT$SSID=") == 0) {
    ssid = cmd.substring(8);
    sendResult(RR_OK);
  }

  /**** Display WiFi SSID ****/
  else if (upCmd == "AT$SSID?") {
    sendString(ssid);
    sendResult(RR_OK);
  }

  /**** Set WiFi Password ****/
  else if (upCmd.indexOf("AT$PASS=") == 0) {
    password = cmd.substring(8);
    sendResult(RR_OK);
  }

  /**** Display WiFi Password ****/
  else if (upCmd == "AT$PASS?") {
    sendString(password);
    sendResult(RR_OK);
  }

  /**** Reset EEPROM and current settings to factory defaults ****/
  else if (upCmd == "AT&F") {
    defaultEEPROM();
    readSettings();
    sendResult(RR_OK);
  }

  /**** Set auto answer off ****/
  else if (upCmd == "ATS0=0") {
    autoAnswer = false;
    sendResult(RR_OK);
  }

  /**** Set auto answer on ****/
  else if (upCmd == "ATS0=1") {
    autoAnswer = true;
    sendResult(RR_OK);
  }

  /**** Display auto answer setting ****/
  else if (upCmd == "ATS0?") {
    sendString(String(autoAnswer));
    sendResult(RR_OK);
  }

  /**** Set PET MCTerm Translate On ****/
  else if (upCmd == "ATPET=1") {
    petTranslate = true;
    sendResult(RR_OK);
  }

  /**** Set PET MCTerm Translate Off ****/
  else if (upCmd == "ATPET=0") {
    petTranslate = false;
    sendResult(RR_OK);
  }

  /**** Display PET MCTerm Translate Setting ****/
  else if (upCmd == "ATPET?") {
    sendString(String(petTranslate));
    sendResult(RR_OK);
  }

  /**** Set HEX Translate On ****/
  else if (upCmd == "ATHEX=1") {
    hex = true;
    sendResult(RR_OK);
  }

  /**** Set HEX Translate Off ****/
  else if (upCmd == "ATHEX=0") {
    hex = false;
    sendResult(RR_OK);
  }

  /**** Hang up a call ****/
  else if (upCmd.indexOf("ATH") == 0) {
    hangUp();
  }

  /**** Hang up a call ****/
  else if (upCmd.indexOf("AT$RB") == 0) {
    sendResult(RR_OK);
    SerialSM.flush();
    delay(500);
    //ESP.reset();
    ESP.restart();
  }

  /**** Exit modem command mode, go online ****/
  else if (upCmd == "ATO") {
    if (callConnected == 1) {
      sendResult(R_CONNECT);
      cmdMode = false;
    } else {
      sendResult(R_ERROR);
    }
  }

  /**** Set incoming TCP server port ****/
  else if (upCmd.indexOf("AT$SP=") == 0) {
    tcpServerPort = upCmd.substring(6).toInt();
    sendString("CHANGES REQUIRES NV SAVE (AT&W) AND RESTART");
    sendResult(RR_OK);
  }

  /**** Display icoming TCP server port ****/
  else if (upCmd == "AT$SP?") {
    sendString(String(tcpServerPort));
    sendResult(RR_OK);
  }

  /**** See my IP address ****/
  else if (upCmd == "ATIP?")
  {
    SerialSM.println(WiFi.localIP());
    sendResult(RR_OK);
  }

  /**** HTTP GET request ****/
  else if (upCmd.indexOf("ATGET") == 0)
  {
    // From the URL, aquire required variables
    // (12 = "ATGEThttp://")
    int portIndex = cmd.indexOf(":", 12); // Index where port number might begin
    int pathIndex = cmd.indexOf("/", 12); // Index first host name and possible port ends and path begins
    int port;
    String path, host;
    if (pathIndex < 0)
    {
      pathIndex = cmd.length();
    }
    if (portIndex < 0)
    {
      port = 80;
      portIndex = pathIndex;
    }
    else
    {
      port = cmd.substring(portIndex + 1, pathIndex).toInt();
    }
    host = cmd.substring(12, portIndex);
    path = cmd.substring(pathIndex, cmd.length());
    if (path == "") path = "/";
    char *hostChr = new char[host.length() + 1];
    host.toCharArray(hostChr, host.length() + 1);

    // Establish connection
    if (!tcpClient.connect(hostChr, port))
    {
      sendResult(R_NOCARRIER);
      connectTime = 0;
      callConnected = false;
      setCarrier(callConnected);
    }
    else
    {
      sendResult(R_CONNECT);
      connectTime = millis();
      cmdMode = false;
      callConnected = true;
      setCarrier(callConnected);

      // Send a HTTP request before continuing the connection as usual
      String request = "GET ";
      request += path;
      request += " HTTP/1.1\r\nHost: ";
      request += host;
      request += "\r\nConnection: close\r\n\r\n";
      tcpClient.print(request);
    }
    delete hostChr;
  }

  /**** Unknown command ****/
  else sendResult(R_ERROR);

  cmd = "";
}

// RTS/CTS protocol is a method of handshaking which uses one wire in each direction to allow each
// device to indicate to the other whether or not it is ready to receive data at any given moment.
// One device sends on RTS and listens on CTS; the other does the reverse. A device should drive
// its handshake-output wire low when it is ready to receive data, and high when it is not. A device
// that wishes to send data should not start sending any bytes while the handshake-input wire is low;
// if it sees the handshake wire go high, it should finish transmitting the current byte and then wait
// for the handshake wire to go low before transmitting any more.
// http://electronics.stackexchange.com/questions/38022/what-is-rts-and-cts-flow-control
void handleFlowControl() {
  if (flowControl == F_NONE) return;
  if (flowControl == F_HARDWARE) {
    if (digitalRead(PortMODEM_CTS) == pinPolarity) txPaused = true;
    else txPaused = false;
  }
  if (flowControl == F_SOFTWARE) {
    
  }
}

/**
   Arduino main loop function
*/
void loopWifi(){
  // Check flow control
  handleFlowControl();
    
  // Service the Web server
  webServer.handleClient();

  // Check to see if user is requesting rate change to 300 baud
  checkButton();

  // New unanswered incoming connection on server listen socket
  if (tcpServer.hasClient()) {
    handleIncomingConnection();
  }

  /**** AT command mode ****/
  if (cmdMode == true)
  {

    // In command mode - don't exchange with TCP but gather characters to a string
    if (SerialSM.available())
    {
      char chr = SerialSM.read();

      if (petTranslate == true)
        // Fix PET MCTerm 1.26C Pet->ASCII encoding to actual ASCII
        if (chr > 127) chr-= 128;
      else
        // Convert uppercase PETSCII to lowercase ASCII (C64) in command mode only
        if ((chr >= 193) && (chr <= 218)) chr-= 96;

      // Return, enter, new line, carriage return.. anything goes to end the command
      if ((chr == '\n') || (chr == '\r'))
      {
        command();
      }
      // Backspace or delete deletes previous character
      else if ((chr == 8) || (chr == 127) || (chr == 20))
      {
        cmd.remove(cmd.length() - 1);
        if (echo == true) {
          SerialSM.write(chr);
        }
      }
      else
      {
        if (cmd.length() < MAX_CMD_LENGTH) cmd.concat(chr);
        if (echo == true) {
          SerialSM.write(chr);
        }
        if (hex) {
          SerialSM.print(chr, HEX);
        }
      }
    }
  }
  /**** Connected mode ****/
  else
  {
    // Transmit from terminal to TCP
    if (SerialSM.available())
    {
      led_on();

      // In telnet in worst case we have to escape every byte
      // so leave half of the buffer always free
      int max_buf_size;
      if (telnet == true)
        max_buf_size = TX_BUF_SIZE / 2;
      else
        max_buf_size = TX_BUF_SIZE;

      // Read from serial, the amount available up to
      // maximum size of the buffer
      size_t len = std::min(SerialSM.available(), max_buf_size);
      SerialSM.readBytes(&txBuf[0], len);

      // Enter command mode with "+++" sequence
      for (int i = 0; i < (int)len; i++)
      {
        if (txBuf[i] == '+') plusCount++; else plusCount = 0;
        if (plusCount >= 3)
        {
          plusTime = millis();
        }
        if (txBuf[i] != '+')
        {
          plusCount = 0;
        }
      }

      // Double (escape) every 0xff for telnet, shifting the following bytes
      // towards the end of the buffer from that point
      if (telnet == true)
      {
        for (int i = len - 1; i >= 0; i--)
        {
          if (txBuf[i] == 0xff)
          {
            for (int j = TX_BUF_SIZE - 1; j > i; j--)
            {
              txBuf[j] = txBuf[j - 1];
            }
            len++;
          }
        }
      }
      // Fix PET MCTerm 1.26C Pet->ASCII encoding to actual ASCII
      if (petTranslate == true) {
        for (int i = len - 1; i >= 0; i--) {
          if (txBuf[i] > 127) txBuf[i]-= 128;
        }
      }
      // Write the buffer to TCP finally
      tcpClient.write(&txBuf[0], len);
      yield();
    }

    // Transmit from TCP to terminal
    while (tcpClient.available() && txPaused == false)
    {
      led_on();
      uint8_t rxByte = tcpClient.read();

      // Is a telnet control code starting?
      if ((telnet == true) && (rxByte == 0xff))
      {
#ifdef DEBUG
        SerialSM.print("<t>");
#endif
        rxByte = tcpClient.read();
        if (rxByte == 0xff)
        {
          // 2 times 0xff is just an escaped real 0xff
          SerialSM.write(0xff); SerialSM.flush();
        }
        else
        {
          // rxByte has now the first byte of the actual non-escaped control code
#ifdef DEBUG
          SerialSM.print(rxByte);
          SerialSM.print(",");
#endif
          uint8_t cmdByte1 = rxByte;
          rxByte = tcpClient.read();
          uint8_t cmdByte2 = rxByte;
          // rxByte has now the second byte of the actual non-escaped control code
#ifdef DEBUG
          SerialSM.print(rxByte); SerialSM.flush();
#endif
          // We are asked to do some option, respond we won't
          if (cmdByte1 == DO)
          {
            tcpClient.write((uint8_t)255); tcpClient.write((uint8_t)WONT); tcpClient.write(cmdByte2);
          }
          // Server wants to do any option, allow it
          else if (cmdByte1 == WILL)
          {
            tcpClient.write((uint8_t)255); tcpClient.write((uint8_t)DO); tcpClient.write(cmdByte2);
          }
        }
#ifdef DEBUG
        SerialSM.print("</t>");
#endif
      }
      else
      {
        // Non-control codes pass through freely
        SerialSM.write(rxByte); yield(); SerialSM.flush(); yield();
      }
      handleFlowControl();
    }
  }

  // If we have received "+++" as last bytes from serial port and there
  // has been over a second without any more bytes
  if (plusCount >= 3)
  {
    if (millis() - plusTime > 1000)
    {
      //tcpClient.stop();
      cmdMode = true;
      sendResult(RR_OK);
      plusCount = 0;
    }
  }

  // Go to command mode if TCP disconnected and not in command mode
  if ((!tcpClient.connected()) && (cmdMode == false) && callConnected == true)
  {
    cmdMode = true;
    sendResult(R_NOCARRIER);
    connectTime = 0;
    callConnected = false;
    setCarrier(callConnected);
    //if (tcpServerPort > 0) tcpServer.begin();
  }

  // Turn off tx/rx led if it has been lit long enough to be visible
  //if (millis() - ledTime > LED_TIME) digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // toggle LED state
}


void prepareNewMode(){
  //mode changed, setup address space
  for(int a=1;a<DATAPAGESIZE;++a) cnDataPage[a]=0; // just clear memory
  cnByteCounter = 0;
  cnCommand_old = cnCommand;
  cnCommand = 255;
  /*
  switch(cnMode){
    case 1: 
       break;
    case 2: 
       break;
    case 3: 
       break;
    case 4: 
       break;
    case 5: // Boot Stored Cartridge
       break;
    case 6: 
       break;
    default: //case 0 etc = ready mode.  Just clear memory. */
      
  //}
}


void executeCommand()
{
  switch(cnMode){
    case 5: switch(cnCommand){//cartridge mode
      case 0: //1541diag I guess
        // load the simple byte copy routine to array
        cnByteDest = 0x8000;
        cnDataPage[73] = 0xA9; //LDA #$xx
        cnDataPage[74] = _1541diagcart_bin[cnByteCounter];
        cnDataPage[75] = 0x8D; //STA $xxxx
        cnDataPage[76] = (cnByteDest & 255);
        cnDataPage[77] = (cnByteDest >> 8); //cnByteDest / 256
        cnDataPage[78] = 0x18; //CLC
        cnDataPage[79] = 0x90; //BCC
        cnDataPage[80] = 0xF8; // -8 (73?)
        cnDataPage[81] = 0x00;
      default:;
    }
    default:; //mode default
  }
}

void transferBinary(const unsigned char *aBinary, uint nSize){
  Serial.printf("Binary %d %d\r\n", nSize, aBinary[0]);
}
void triggerByteRead(byte nAddr)
{
  switch(cnMode){
  // To copy one byte:
  //73 = sys56777
  //73        75  76 77  78   79  80 81
  //LDA #$xx, STA $xxxx, CLC, BCC -8
  //                          JMP $xxxx
  //A9 xx 8D xx xx 18 90 F8
  // each time it reads 78, either the byte for LDA is updated ***OR*** BCC is changed to JMP
  case 5: //cartridge mode
    switch(cnCommand){
    case 0: //1541diag
    transferBinary(_1541diagcart_bin,_1541diagcart_bin_size);
    
    if(nAddr==78){
      ++cnByteCounter; ++cnByteDest;
      if(cnByteCounter >= _1541diagcart_bin_size){
        cnDataPage[74] = _1541diagcart_bin[cnByteCounter];
        cnDataPage[76] = (cnByteDest & 255);
        cnDataPage[77] = (cnByteDest >> 8); //cnByteDest / 256
      }else{
        //run the binary
        cnDataPage[79] = 0x4c; //JMP $8009
        cnDataPage[80] = 0x09;
        cnDataPage[81] = 0x80;
      }
    }; break;
    case 1: //Tool-64
    if(nAddr==78){
      ++cnByteCounter; ++cnByteDest;
      if(cnByteCounter >= tool64_bin_size){
        cnDataPage[74] = tool64_bin[cnByteCounter];
        cnDataPage[76] = (cnByteDest & 255);
        cnDataPage[77] = (cnByteDest >> 8); //cnByteDest / 256
      }else{
        //run the binary
        cnDataPage[79] = 0x4c; //JMP $8009
        cnDataPage[80] = (SYSRESET & 255);
        cnDataPage[81] = (SYSRESET >> 8);
      }
    }; break;
    default:;
    }; break;
  default:;
  }
}

void loop()
{  
  checkStateTransitions();
  
  if(cbEnabledModem) { loopWifi(); return; }
  if(!cbEnabledAddressing) return;//for now, both can be disabled... comment this line out later

  if(cbCS || (!cbCS && (cuCS_time<cuWaitCS))){
    // device not enabled/selected this cycle, or too soon after enabling to be enabled.
    pinMode(PortD00,INPUT);
    pinMode(PortD01,INPUT);
    pinMode(PortD02,INPUT);
    pinMode(PortD03,INPUT);
    pinMode(PortD04,INPUT);
    pinMode(PortD05,INPUT);
    pinMode(PortD06,INPUT);
    pinMode(PortD07,INPUT);
    return; 
  }
  //if((cuRW_time<cuWaitRW))) return; //debug

  setDataRead(); //configure data bus for read/write
  
  byte nAddr = getByteAddress(); //6 bits, 0 to 127
  if(cbRW){ //read output
    putByteData(cnDataPage[nAddr]);
    triggerByteRead(nAddr);
    
  }else{ //write input
    byte nByte = getByteData();
    if(nAddr < 3) cnDataPage[nAddr] = nByte; // first three bytes will always be accepted for writing
    if(nAddr==0 && cnMode != nByte){ //changing mode
      cnMode_old = cnMode;
      cnMode = nByte;
      if(cnMode_old==1 && cnMode==0) enableAddressing(true); //disables wifi
      prepareNewMode();
      return;
    }
    if(nAddr==1 && cnCommand != nByte){ //new command
      cnCommand_old = cnCommand;
      cnCommand = nByte;
      executeCommand();
      return;
    }

    //legitimate writes
    bool bEnableWrite = false;
    switch(cnMode){
      case 3: if(nAddr >= 7) bEnableWrite=true; break; //expression evaluator
      case 4: if(nAddr < 72) bEnableWrite=true; break; //interactive menu
      default:;
    }
    cnDataPage[nAddr] = nByte;
    
    //writes are commands or storage of data, or prohibited by mode.
    //writing 0 to dd80 command byte will be the only way to break out of wifi mode, by returning to Ready mode.
    //usage: set mode first (clears io space), [then set var memory], then set command byte.
  }


}

/*
bool cbCLK = false, cbCLK_OLD = false, cbCLK_NEG = false, cbCLK_POS = false; clock_t cbCLK_NEG_time=0, cbCLK_POS_time=0;
bool cbRW = false, cbRW_OLD = false, cbRW_NEG = false, cbRW_POS = false; clock_t cbRW_NEG_time=0, cbRW_POS_time=0;
bool cbCS = false, cbCS_OLD = false, cbCS_NEG = false, cbCS_POS = false; clock_t cbCS_NEG_time=0, cbCS_POS_time=0;

byte cnAddress = 0, cnAddress_OLD = 0;
bool cbEnabledModem = false;
bool cbEnabledAddressing = false;
byte cnDataPage[128];
*/
