
#include "Arduino.h"
#include "config.h"
#include "def.h"
#include "types.h"
#include "MultiWii.h"
#include <RF24.h>
#include "NRF24_RX.h"
#include "Serial.h"

#include <SPI.h>

#if defined(NRF24_RX)

#define CPACKET_MAGIC 110
#define REQ_SIGNAL     251
#define ACCEPT_SIGNAL    252
#define RPACKET_MAGIC  120
#define FALSE_PACKET   145

int16_t nrf24_rcData[RC_CHANS];
int rcPins[] = {
  A2, 12, 10, 11, A1, A3}; //{A0, A1, A3, A6, A7, A2};

int checksum(char *buf, int len)
{
  char tt = 0;
  for (int i = 0; i < len - 1; i++)
  {
    tt ^= buf[i];
  }
  return tt;
}

struct ControlPackets
{
  unsigned char magic;
  unsigned char throttle;
  unsigned char pitch;
  unsigned char roll;
  unsigned char yaw;
  unsigned char aux1;
  unsigned char aux2;
  unsigned char switches;
  unsigned char random[9];
  unsigned char checksum;
};

struct ResponsePackets
{
  unsigned char magic;
  unsigned char alt;
  unsigned char pitch;
  unsigned char roll;
  unsigned char yaw;
  unsigned char lat;
  unsigned char lon;
  unsigned char heading;
  unsigned char random[9];
  unsigned char checksum;
};

ControlPackets rfCusData;
ResponsePackets rfResData;
unsigned char *buff = (unsigned char *)&rfCusData;
unsigned char *rbuff = (unsigned char *)&rfResData;

volatile int index = 0;
volatile boolean process = true;
/*
// Single radio pipe address for the 2 nodes to communicate.
 static const uint64_t pipe = 0xE8E8F0F0E1LL;
 
 RF24 radio(A2, 10); // CE, CSN
 
 RF24Data nrf24Data;
 RF24AckPayload nrf24AckPayload;
 extern RF24AckPayload nrf24AckPayload;
 
 void resetRF24Data() 
 {
 nrf24Data.throttle = 0;
 nrf24Data.yaw = 128;
 nrf24Data.pitch = 128;
 nrf24Data.roll = 128;
 nrf24Data.dial1 = 0;
 nrf24Data.dial2 = 0;
 nrf24Data.switches = 0;
 }
 
 void resetRF24AckPayload() 
 {
 nrf24AckPayload.lat = 0;
 nrf24AckPayload.lon = 0;
 nrf24AckPayload.heading = 0;
 nrf24AckPayload.pitch = 0;
 nrf24AckPayload.roll = 0;
 nrf24AckPayload.alt = 0;
 nrf24AckPayload.flags = 0;
 }
 */

void NRF24_Init()
{

  //resetRF24Data();
  //resetRF24AckPayload();

  /*radio.begin();
   radio.setDataRate(RF24_250KBPS);
   radio.setAutoAck(1);                    // Ensure autoACK is enabled
   radio.enableAckPayload();
   
   radio.openReadingPipe(1,pipe);
   radio.startListening();  */
  /*pinMode(rcPins[0], INPUT);
   pinMode(rcPins[1], INPUT);
   pinMode(rcPins[2], INPUT);
   pinMode(rcPins[3], INPUT);
   pinMode(rcPins[4], INPUT);
   pinMode(rcPins[5], INPUT);*/
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);
  // SPI.begin();
  index = 0;
  process = true;
  SPI.attachInterrupt();
  /*for (int i = 0; i < sizeof(ControlPackets); i++)
  {
    SPDR = rbuff[i];
  }*/
  SPDR = ACCEPT_SIGNAL;  // Recieved Signal
  //Serial.begin(115200);
}

ISR(SPI_STC_vect)
{
  unsigned char g = SPDR;
  if (!process) // && (SPSR & (1<<SPIF)) != 0)
  {
    if (index < sizeof(ControlPackets)-1)
    {
      SPDR = index+1;
      buff[index++] = g;
    }
    else
    {
      buff[sizeof(ControlPackets)-1] = SPDR;
      //if(buff[0] == 110)
      process = true;

      if (rfCusData.magic == CPACKET_MAGIC)
      {
        rfResData.magic = RPACKET_MAGIC; 
        rfResData.lat = 35.62;
        rfResData.lon = 139.68;
        rfResData.heading = att.heading;
        rfResData.pitch = att.angle[PITCH];
        rfResData.roll = att.angle[ROLL];
        rfResData.alt = alt.EstAlt;

        /*for (int i = 0; i < sizeof(ControlPackets); i++)
        {
          SPDR = rbuff[i];
        }*/

        nrf24_rcData[THROTTLE] = map(rfCusData.throttle, 0, 255, 1000, 2000);
        nrf24_rcData[YAW] = map(rfCusData.yaw, 0, 255, 1000, 2000);
        nrf24_rcData[PITCH] = map(rfCusData.pitch, 0, 255, 1000, 2000);
        nrf24_rcData[ROLL] = map(rfCusData.roll, 0, 255, 1000, 2000);
        nrf24_rcData[AUX1] = map(rfCusData.aux1, 0, 255, 1000, 2000);
        nrf24_rcData[AUX2] = map(rfCusData.aux2, 0, 255, 1000, 2000);
        SPDR = ACCEPT_SIGNAL;  // Recieved Signal
      }
      else 
      {
        /*for (int i = 0; i < sizeof(ControlPackets); i++)
        {
          SPDR = FALSE_PACKET;
        }*/
        SPDR = FALSE_PACKET;  // Recieved Signal
      }
    }
  }
  else if(g == REQ_SIGNAL)  // Handshake
  {
    //buff[0] = g;
    //SPDR = ACCEPT_SIGNAL;  // Recieved Signal

    index = 0;
    process = false;
  }
}

void NRF_Write_TELE()
{
  // radio.writeAckPayload(1, serialBufferTX, TX_BUFFER_SIZE);
}

void NRF24_Read_RC()
{

  //static unsigned long lastRecvTime = 0;
  /*
  nrf24AckPayload.lat = 35.62;
   nrf24AckPayload.lon = 139.68;
   nrf24AckPayload.heading = att.heading;
   nrf24AckPayload.pitch = att.angle[PITCH];
   nrf24AckPayload.roll = att.angle[ROLL];
   nrf24AckPayload.alt = alt.EstAlt;
   memcpy(&nrf24AckPayload.flags, &f, 1); // first byte of status flags
   	
   unsigned long now = millis();
   while ( radio.available() ) 
   {
   #if defined(NRF_TELEMETRY_CUSTOM)
   //radio.writeAckPayload(1, serialBufferTX, TX_BUFFER_SIZE);
   radio.writeAckPayload(1, &nrf24AckPayload, sizeof(RF24AckPayload));
   #else
   radio.writeAckPayload(1, &nrf24AckPayload, sizeof(RF24AckPayload));
   #endif
   radio.read(&nrf24Data, sizeof(RF24Data));
   lastRecvTime = now;
   }
   if ( now - lastRecvTime > 1000 ) {
   // signal lost?
   resetRF24Data();
   }*/

  /*nrf24Data.throttle = analogRead(rcPins[0]);
   nrf24Data.yaw = analogRead(rcPins[1]);
   nrf24Data.pitch = analogRead(rcPins[2]);
   nrf24Data.roll = analogRead(rcPins[3]);
   nrf24Data.dial1 = analogRead(rcPins[4]);*/

  /*rfResData.lat = 35.62;
   rfResData.lon = 139.68;
   rfResData.heading = att.heading;
   rfResData.pitch = att.angle[PITCH];
   rfResData.roll = att.angle[ROLL];
   rfResData.alt = alt.EstAlt;
   
   for (int i = 0; i < index; i++)
   {
   SPDR = rbuff[i];
   }
   
   if (process)
   {
   index = 0;
   if (rfCusData.magic == CPACKET_MAGIC)
   {
   nrf24_rcData[THROTTLE] = map(rfCusData.throttle, 0, 255, 1000, 2000);
   nrf24_rcData[YAW] = map(rfCusData.yaw, 0, 255, 1000, 2000);
   nrf24_rcData[PITCH] = map(rfCusData.pitch, 0, 255, 1000, 2000);
   nrf24_rcData[ROLL] = map(rfCusData.roll, 0, 255, 1000, 2000);
   nrf24_rcData[AUX1] = map(rfCusData.aux1, 0, 255, 1000, 2000);
   nrf24_rcData[AUX2] = map(rfCusData.aux2, 0, 255, 1000, 2000);
   }
   process = false;
   }//*/

}

#endif



