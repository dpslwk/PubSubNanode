/*
 PubSubNanode.h - A simple client for MQTT.
  Nicholas O'Leary
  http://knolleary.net
*/

#ifndef PubSubClient_h
#define PubSubClient_h

#include "EtherShield.h"

// packet buffer for EtherSheild
// Packet buffer, must be big enough to packet and payload
// Estimate at ~ MAX_PACKET_SIZE * 1.5
#define BUFFER_SIZE 200
static uint8_t esBuf[BUFFER_SIZE+1];

#defien QUE_SIZE 4
#define MAX_PACKET_SIZE 128
#define KEEPALIVE 15000 // max value = 255000

// from mqtt-v3r1 
#define MQTTPROTOCOLVERSION 3
#define MQTTCONNECT     1 << 4  // Client request to connect to Server
#define MQTTCONNACK     2 << 4  // Connect Acknowledgment
#define MQTTPUBLISH     3 << 4  // Publish message
#define MQTTPUBACK      4 << 4  // Publish Acknowledgment
#define MQTTPUBREC      5 << 4  // Publish Received (assured delivery part 1)
#define MQTTPUBREL      6 << 4  // Publish Release (assured delivery part 2)
#define MQTTPUBCOMP     7 << 4  // Publish Complete (assured delivery part 3)
#define MQTTSUBSCRIBE   8 << 4  // Client Subscribe request
#define MQTTSUBACK      9 << 4  // Subscribe Acknowledgment
#define MQTTUNSUBSCRIBE 10 << 4 // Client Unsubscribe request
#define MQTTUNSUBACK    11 << 4 // Unsubscribe Acknowledgment
#define MQTTPINGREQ     12 << 4 // PING Request
#define MQTTPINGRESP    13 << 4 // PING Response
#define MQTTDISCONNECT  14 << 4 // Client is Disconnecting
#define MQTTReserved    15 << 4 // Reserved



class PubSubClient {
private:
   EtherShield _es;
	uint8_t serverip;
	uint16_t port;
	uint8_t que[QUE_SIZE][MAX_PACKET_SIZE];
	uint8_t queLength[QUE_SIZE];
	uint8_t queHead;
	uint8_t queTail;
	uint8_t queCount;
	bool resultWaitFlag;
	bool tcpReqFlag;
   uint8_t buffer[MAX_PACKET_SIZE];
   uint8_t nextMsgId;
   long lastOutActivity;
   long lastInActivity;
   bool pingOutstanding;
   void (*callback)(char*,uint8_t*,int);
   uint8_t readPacket();
   uint8_t readByte();
   int write(uint8_t header, uint8_t* buf, uint8_t length);
   uint8_t writeString(char* string, uint8_t* buf, uint8_t pos);
	int queWrite(uint8_t* buf, uint8_t length);
	uint16_t queRead(uint8_t); // datafill callback
public:
   PubSubClient();
   PubSubClient(uint8_t *, uint16_t, void(*)(char*,uint8_t*,int));
	int init(unit8_t *, uint8_t *, uint8_t *, uint16_t);
   int connect(char *);
   int connect(char*, char*, uint8_t, uint8_t, char*);
   void disconnect();
   int publish(char *, char *);
   int publish(char *, uint8_t *, uint8_t);
   int publish(char *, uint8_t *, uint8_t, uint8_t);
   void subscribe(char *);
   int loop();
   int connected();
};


#endif
