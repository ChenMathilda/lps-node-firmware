/*
 * uwbTwrp2p.c
 *
 *  Created on: Dec 2, 2020
 *      Author: bitcraze
 */

//#define DEBUG_MODULE "DWM"

#include "uwb.h"

#include <stm32f0xx_hal.h>

#include <string.h>
#include <stdio.h>

#include "cfg.h"
#include "led.h"

#include "lpp.h"

#include "libdw1000.h"

#include "dwOps.h"
#include "mac.h"

static uint8_t base_address[] = {0,0,0,0,0,0,0xcf,0xbc};

#define ANTENNA_OFFSET 154.6   // In meter

static int ANTENNA_DELAY = (ANTENNA_OFFSET*499.2e6*128)/299792458.0; // In radio tick

#define LPS_TWRP2P_POLL 0x01   // Poll is initiated by the tag
#define LPS_TWRP2P_ANSWER 0x02
#define LPS_TWRP2P_FINAL 0x03
#define LPS_TWRP2P_REPORT 0x04 // Report contains all measurement from the anchor

#define LPS_TWRP2P_TYPE 0
#define LPS_TWRP2P_SEQ 1

#define LOCODECK_TS_FREQ (499.2e6 * 128)
#define SPEED_OF_LIGHT (299792458.0)

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) reportPayload_t;

uwbConfig_t config;

extern uwbAlgorithm_t uwbTWRP2PAlgorithm;

typedef struct {
  uint8_t pollRx[5];
  uint8_t answerTx[5];
  uint8_t finalRx[5];

  float pressure;
  float temperature;
  float asl;
  uint8_t pressure_ok;
} __attribute__((packed)) lpsTWRp2pReportPayload_t;

#define MAX_UWB_RECEIVE_TIMEOUT 65 //65ms is max interval

// Timestamps for ranging
static dwTime_t poll_tx;
static dwTime_t poll_rx;
static dwTime_t answer_tx;
static dwTime_t answer_rx;
static dwTime_t final_tx;
static dwTime_t final_rx;

static packet_t txPacket;
static packet_t rxPacket;
static volatile uint8_t curr_seq = 0;
static int curr_peer = 0;

float twrp2p_pressure = 0;
float twrp2p_temperature = 0;
float twrp2p_asl = 0;
bool twrp2p_pressure_ok = true;

static float twrp2p_pdistance = 0;
static uint32_t twrp2p_timeout_p2p=0;
static uint32_t twrp2p_default_twr_interval=4000;

// #define printf(...)
//#define debug(...) // printf(__VA_ARGS__)

static void txcallback(dwDevice_t *dev)
{
  dwTime_t departure;
  dwGetTransmitTimestamp(dev, &departure);
  departure.full += (ANTENNA_DELAY / 2);

  switch (txPacket.payload[0]) {
    case LPS_TWRP2P_POLL:
      printf("Node as tag:sent LPS_TWRP2P_POLL\r\n");
      poll_tx = departure;
      break;
    case LPS_TWRP2P_FINAL:
      printf("Node as tag:sent LPS_TWRP2P_FINAL\r\n");
      final_tx = departure;
      break;
    case LPS_TWRP2P_ANSWER:
      printf("Node as anchor:sent LPS_TWRP2P_ANSWER\r\n");
      answer_tx = departure;
      break;
    case LPS_TWRP2P_REPORT:
      printf("Node as anchor:sent LPS_TWRP2P_REPORT\r\n");
      break;
  }
}

static uint32_t rxcallback(dwDevice_t *dev)
{
  dwTime_t arival = { .full=0 };
  dwGetReceiveTimestamp(dev, &arival);

  int dataLength = dwGetDataLength(dev);
  if (dataLength == 0) return 0;


  memset(&rxPacket, 0, MAC802154_HEADER_LENGTH);
  dwGetData(dev, (uint8_t*)&rxPacket, dataLength);

  memcpy(txPacket.destAddress, rxPacket.sourceAddress, 8);
  memcpy(txPacket.sourceAddress, rxPacket.destAddress, 8);


  printf("LPS_TWRP2P_TYPE = %d\r\n", (unsigned int)rxPacket.payload[LPS_TWRP2P_TYPE]);
  switch(rxPacket.payload[LPS_TWRP2P_TYPE]) {
    case LPS_TWRP2P_POLL:
    	printf("Node as anchor:received LPS_TWRP2P_POLL\r\n");

    	curr_peer = rxPacket.sourceAddress;

    	int payloadLength = 2;
    	txPacket.payload[LPS_TWRP2P_TYPE] = LPS_TWRP2P_ANSWER;
    	txPacket.payload[LPS_TWRP2P_SEQ] = rxPacket.payload[LPS_TWRP2P_SEQ];

    	arival.full -= (ANTENNA_DELAY/2);
    	poll_rx = arival;

    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+payloadLength);
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	//twrp2p_timeout_p2p = twrp2p_default_twr_interval;
    	break;

    // Tag received messages
    case LPS_TWRP2P_ANSWER:
    	printf("Node as tag:received LPS_TWRP2P_ANSWER111\r\n");
    	//if(rxPacket.payload[LPS_TWRP2P_SEQ] != curr_seq)
    		//return 0;
    	printf("received LPS_TWRP2P_ANSWER222\r\n");
    	txPacket.payload[LPS_TWRP2P_TYPE] = LPS_TWRP2P_FINAL;
    	txPacket.payload[LPS_TWRP2P_SEQ] = rxPacket.payload[LPS_TWRP2P_SEQ];

    	arival.full -= (ANTENNA_DELAY / 2);
    	answer_rx = arival;

    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	//twrp2p_timeout_p2p = twrp2p_default_twr_interval;
    	break;
    case LPS_TWRP2P_FINAL:
    {
    	printf("Node as anchor:received LPS_TWRP2P_FINAL\r\n");
    	//if(curr_peer != rxPacket.sourceAddress) return 0;
    	reportPayload_t *report = (reportPayload_t *)(txPacket.payload+2);

    	arival.full -= (ANTENNA_DELAY/2);
    	final_rx = arival;

    	txPacket.payload[LPS_TWRP2P_TYPE] = LPS_TWRP2P_REPORT;
    	txPacket.payload[LPS_TWRP2P_SEQ] = rxPacket.payload[LPS_TWRP2P_SEQ];
        memcpy(&report->pollRx, &poll_rx, 5);
        memcpy(&report->answerTx, &answer_tx, 5);
        memcpy(&report->finalRx, &final_rx, 5);
        report->pressure = twrp2p_pressure;
        report->temperature = twrp2p_temperature;
        report->asl = twrp2p_asl;
        report->pressure_ok = twrp2p_pressure_ok;


    	dwNewTransmit(dev);
    	dwSetDefaults(dev);
    	dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2+sizeof(reportPayload_t));
    	dwWaitForResponse(dev, true);
    	dwStartTransmit(dev);
    	//twrp2p_timeout_p2p = twrp2p_default_twr_interval/2; //set a shorter delay to sent next poll
    	break;
    }
    case LPS_TWRP2P_REPORT:
    {
    	printf("Node as tag:received LPS_TWRP2P_REPORT\r\n");
    	lpsTWRp2pReportPayload_t *report = (lpsTWRp2pReportPayload_t *)(rxPacket.payload+2);
    	double tround1, treply1, treply2, tround2, tprop_ctn, tprop;

    	//if (rxPacket.payload[LPS_TWRP2P_SEQ] != curr_seq)
    		// return 0;
    	memcpy(&poll_rx, &report->pollRx, 5);
    	memcpy(&answer_tx, &report->answerTx, 5);
    	memcpy(&final_rx, &report->finalRx, 5);

    	tround1 = answer_rx.low32 - poll_tx.low32;
    	treply1 = answer_tx.low32 - poll_rx.low32;
    	tround2 = final_rx.low32 - answer_tx.low32;
    	treply2 = final_tx.low32 - answer_rx.low32;

    	tprop_ctn = ((tround1*tround2) - (treply1*treply2)) / (tround1 + tround2 + treply1 + treply2);

    	tprop = tprop_ctn / LOCODECK_TS_FREQ;
    	twrp2p_pdistance = SPEED_OF_LIGHT * tprop;

    	printf("****************LPS node Distance = %d\r\n",(int)(100*twrp2p_pdistance));
    	dwNewReceive(dev);
    	dwSetDefaults(dev);
    	dwStartReceive(dev);
    	twrp2p_timeout_p2p = twrp2p_default_twr_interval;
    	break;
    	}
  	 }
  	 return twrp2p_timeout_p2p;
}

static void initiateRanging(dwDevice_t *dev)
{

  base_address[0] =  config.anchors[curr_peer];
  curr_peer++;
  txPacket.payload[LPS_TWRP2P_TYPE] = LPS_TWRP2P_POLL;
  txPacket.payload[LPS_TWRP2P_SEQ] = ++curr_seq;

  memcpy(txPacket.sourceAddress, config.address, 8);
  memcpy(txPacket.destAddress, base_address, 8);

  dwIdle(dev);

  memset(&poll_tx, 0, sizeof(poll_tx));
  memset(&poll_rx, 0, sizeof(poll_rx));
  memset(&answer_tx, 0, sizeof(answer_tx));
  memset(&answer_rx, 0, sizeof(answer_rx));
  memset(&final_tx, 0, sizeof(final_tx));
  memset(&final_rx, 0, sizeof(final_rx));
  curr_seq = 0;
  curr_peer = 0;
  twrp2p_pressure = twrp2p_temperature = twrp2p_asl = 0;
  twrp2p_pressure_ok = true;
  //twrp2p_timeout_p2p = 0;

  dwNewTransmit(dev);
  dwSetDefaults(dev);
  dwSetData(dev, (uint8_t*)&txPacket, MAC802154_HEADER_LENGTH+2);

  dwWaitForResponse(dev, true);
  dwStartTransmit(dev);
  //printf("LPS node: InitiateRanging\r\n");

}

static uint32_t twrp2pOnEvent(dwDevice_t *dev, uwbEvent_t event)
{
	//printf("twrp2pInit111111111111111111111111111111\r\n");
  switch(event) {
    case eventPacketReceived:
      //rxcallback(dev);
      twrp2p_timeout_p2p=rxcallback(dev);
      break;
    case eventPacketSent:
      txcallback(dev);
      //twrp2p_timeout_p2p=twrp2p_default_twr_interval;
      break;
    case eventReceiveTimeout:
      dwNewReceive(dev);
      dwSetDefaults(dev);
      dwStartReceive(dev);
      //twrp2p_timeout_p2p = (twrp2p_timeout_p2p>MAX_UWB_RECEIVE_TIMEOUT ? twrp2p_timeout_p2p-MAX_UWB_RECEIVE_TIMEOUT : 0) ;
      break;
    case eventTimeout:  // Comes back to timeout after each ranging attempt
      //printf("eventTimeout\r\n");
      initiateRanging(dev);
      twrp2p_timeout_p2p = 0;
      break;
    case eventReceiveFailed:
      return 0;
  }
  return twrp2p_timeout_p2p;
}

static void twrp2pInit(uwbConfig_t * config, dwDevice_t *dev)
{
	ledOn(ledMode);
  // Initialize the packet in the TX buffer
  memset(&txPacket, 0, sizeof(txPacket));
  MAC80215_PACKET_INIT(txPacket, MAC802154_TYPE_DATA);
  txPacket.pan = 0xbccf;

  dwSetReceiveWaitTimeout(dev, 0);
  dwCommitConfiguration(dev);
}


uwbAlgorithm_t uwbTWRP2PAlgorithm = {
  .init = twrp2pInit,
  .onEvent = twrp2pOnEvent,
};

//LOG_GROUP_START(peerdist)
//LOG_ADD(LOG_FLOAT, distance2peer, &pdistance)
//LOG_GROUP_STOP(peerdist)




