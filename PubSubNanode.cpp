/*
 PubSubNanode.cpp - A simple client for MQTT.
 Nicholas O'Leary
 http://knolleary.net
 */

#include "PubSubNanode.h"
#include <EtherShield.h>
#include "string.h"
#include "wiring.h"

PubSubNanode::PubSubNanode() : _es() {
}

PubSubNanode::PubSubNanode(uint8_t *serverip, uint16_t port, void (*callback)(char*,uint8_t*,int)) : _es() {
	//Set the Server IP 
	_es.ES_client_tcp_set_serverip(serverip);
	//Serial.println("Init serverip ip");
	
	this->callback = callback;
	this->port = port;
	
	// clear queState counters
	queHead = queTail = queCount = 0;
	
	// not yet connected
	rc = 0; 
}

int PubSubNanode::init(uint8_t *mymac, uint8_t *myip, uint8_t *gwip, uint16_t initPort) {
	// initialize SPI
	_es.ES_enc28j60SpiInit();
	//Serial.println("SPI Inint");
	
	// initialize enc28j60
	_es.ES_enc28j60Init(mymac);
	//Serial.println("Init mac");
	
	//init the ethernet/ip layer:
	_es.ES_init_ip_arp_udp_tcp(mymac, myip, initPort);
	//Serial.println("Init ip");
	
	//Set Gateway ip
	_es.ES_client_set_gwip(gwip);
	//Serial.println("Init Gateway ip");
	
}

int PubSubNanode::connect(char *id) {
	return connect(id,0,0,0,0);
}

int PubSubNanode::connect(char *id, char* willTopic, uint8_t willQos, uint8_t willRetain, char* willMessage) {
	if (!connected()) {
		//if (/* _client.connect() */) {
			nextMsgId = 1;
			uint8_t d[9] = {0x00,0x06,'M','Q','I','s','d','p',MQTTPROTOCOLVERSION};
			uint8_t length = 0;
			int j;
			for (j = 0;j<9;j++) {
				buffer[length++] = d[j];
			}
			if (willTopic) {
				buffer[length++] = 0x06|(willQos<<3)|(willRetain<<5);
			} else {
				buffer[length++] = 0x02;
			}
			buffer[length++] = 0;
			buffer[length++] = (KEEPALIVE/1000);
			length = writeString(id,buffer,length);
			if (willTopic) {
				length = writeString(willTopic,buffer,length);
				length = writeString(willMessage,buffer,length);
			}
			write(MQTTCONNECT,buffer,length);
			loop();
			// lastOutActivity = millis();
			// lastInActivity = millis();
			while (resultWaitFlag) {
				loop();
				long t= millis();
				if (t-lastInActivity > KEEPALIVE) {
					// EtherSheild does not have a stop
					/* _client.stop(); */
					// failed to connect
					rc = 0;
					return 0;
				}
			}
			// uint8_t len = readPacket();
			
			/* if (len == 4 && buffer[3] == 0) {
				lastInActivity = millis();
				pingOutstanding = false;
				// connected
				rc = 1;
				return 1;
			} */
			// will check from callback
		    if (rc == 1) {
				return 1;
			}
		// } // end if (_client.connect())
		// EtherSheild does not have a stop
		/* _client.stop(); */
		// failed to connect
		rc = 0;
	}
	return 0;
}


static uint8_t PubSubNanode::resultCallback(uint8_t fd, uint8_t statuscode,uint16_t data_start_pos_in_buf, uint16_t len_of_data) {
	if (!statuscode && len_of_data >0) {
		uint8_t len = readPacket(data_start_pos_in_buf, len_of_data);
		
		lastInActivity = millis();
		uint8_t type = buffer[0]&0xF0;
		if (type == MQTTPUBLISH) {
			if (callback) {
				uint8_t tl = (buffer[2]<<3)+buffer[3];
				char topic[tl+1];
				for (int i=0;i<tl;i++) {
					topic[i] = buffer[4+i];
				}
				topic[tl] = 0;
				// ignore msgID - only support QoS 0 subs
				uint8_t *payload = buffer+4+tl;
				callback(topic,payload,len-4-tl);
			}
		} else if (type == MQTTPINGREQ) {
			/* _client.write(MQTTPINGRESP); */
			/* _client.write((uint8_t)0); */
			
			// plan B
			write(MQTTPINGRESP, 0, 0);
			
		} else if (type == MQTTPINGRESP) {
			pingOutstanding = false;
		} else if (type == MQTTCONNACK) {
			if (len_of_data == 4 && buffer[3] == 0) {
				pingOutstanding = false;
				// connected
				rc = 1;
			}
		}
	}
}
	
// modify for result_callback use
uint8_t PubSubNanode::readPacket(uint16_t pos, uint16_t len_of_data) {
	
	uint8_t len = 0;
	buffer[len++] = esBuf[pos++];  // copy byte 1 of fixed heder
	uint8_t multiplier = 1;
	uint8_t length = 0;
	uint8_t digit = 0;
	// calculate remmaing length see spec for byte encoding of lenght over 127
	do {
		digit = esBuf[pos++];
		buffer[len++] = digit;
		length += (digit & 127) * multiplier;
		multiplier *= 128;
	} while ((digit & 128) != 0);
	// copy payload 
	for (int i = 0;i<length;i++)
	{
		if (len < MAX_PACKET_SIZE) {
			buffer[len++] = esBuf[pos++];
		} else {
			pos++;
			len = 0; // This will cause the packet to be ignored.
		}
	}
	
	return len; 
	
}

int PubSubNanode::loop() {
	if (connected()) {
		uint16_t dat_p;
        // handle ping and wait for a tcp packet
		dat_p=_es.ES_packetloop_icmp_tcp(esBuf,_es.ES_enc28j60PacketReceive(BUFFER_SIZE, esBuf));
		
		// some thing to send?
		if (tcpReqFlag == 1) {
			_es.ES_client_tcp_req(&resultCallback, &queRead, port);
			tcpReqFlag = 0;
			resultWaitFlag = 1;
		} 
		
		if(dat_p==0) {
			// we are idle here
			if (_es.ES_client_waiting_gw() ){
				return 1;
			}
			
			
		}

			
			
			
		long t = millis();
		if ((t - lastInActivity > KEEPALIVE) || (t - lastOutActivity > KEEPALIVE)) {
			if (pingOutstanding) {
				/* _client.stop(); */
				return 0;
			} else {
				/* _client.write(MQTTPINGREQ); */
				/* _client.write((uint8_t)0); */
				// plan B
				write(MQTTPINGREQ, 0, 0);
				// lastOutActivity = t;
				// lastInActivity = t;
				pingOutstanding = true;
			}
		}
		// moved to readPacket (callback)
		return 1;
	}
	return 0;
}

int PubSubNanode::publish(char* topic, char* payload) {
	return publish(topic,(uint8_t*)payload,strlen(payload));
}

int PubSubNanode::publish(char* topic, uint8_t* payload, uint8_t plength) {
	return publish(topic, payload, plength, 0);
}

int PubSubNanode::publish(char* topic, uint8_t* payload, uint8_t plength, uint8_t retained) {
	if (connected()) {
		uint8_t length = writeString(topic,buffer,0);
		int i;
		for (i=0;i<plength;i++) {
			buffer[length++] = payload[i];
		}
		uint8_t header = MQTTPUBLISH;
		if (retained != 0) {
			header |= 1;
		}
		write(header,buffer,length);
		return 1;
	}
	return 0;
}


int PubSubNanode::write(uint8_t header, uint8_t* buf, uint8_t length) {
	/* _client.write(header); */
	/* _client.write(length); */
	/* _client.write(buf,length); */
	// lastOutActivity = millis();
	
	// check for room in que ret 1 if full
	// copy header to que tail (fixed header)
	// copy length (remaind length)
	// copy buf to que tail (variable header followed by payload)
	// store lenght
	// inc tail and count
	// return 
	if (queCount < QUE_SIZE) {
		que[queTail][0] = header;
		que[queTail][1] = length;
		if (length != 0) {		// plan B check
			for (uint8_t i=2 ; i <= length+2; i++) {
				que[queTail][i] = buf[i];
			}
		}
		queLength[queTail] = length+2;
		if (queTail == QUE_SIZE) {
			queTail = 0;
		} else {
			queTail++;
		}
		queCount++;
		if (queCount == 1 && resultWaitFlag == 0) {
			tcpReqFlag = 1;
		}
		return(1);
	} else {
		// que full
		return(0);
	}
	
	
	
	return 0;
}


void PubSubNanode::subscribe(char* topic) {
	if (connected()) {
		uint8_t length = 2;
		nextMsgId++;
		buffer[0] = nextMsgId >> 8;
		buffer[1] = nextMsgId - (buffer[0]<<8);
		length = writeString(topic, buffer,length);
		buffer[length++] = 0; // Only do QoS 0 subs
		write(MQTTSUBSCRIBE,buffer,length);
	}
}

void PubSubNanode::disconnect() {
	/* _client.write(MQTTDISCONNECT); */
	/* _client.write((uint8_t)0); */
	
	/* plan A 
	 unint8_t length = 0;
	length = writeString(MQTTDISCONNECT, buffer, 0);	
	length = writeString((uint8_t)0, buffer, length);
	queWrite(buffer, length); 
	 */
	
	// plan B 
	write(MQTTDISCONNECT, 0, 0);
	
	
	// EtherSheild does not have a stop
	/* _client.stop(); */
	rc = 0;
	
	// handel these else were now
	// lastInActivity = millis();
	// lastOutActivity = millis();
}

uint8_t PubSubNanode::writeString(char* string, uint8_t* buf, uint8_t pos) {
	char* idp = string;
	uint8_t i = 0;
	pos += 2;
	while (*idp) {
		buf[pos++] = *idp++;
		i++;
	}
	buf[pos-i-2] = 0;
	buf[pos-i-1] = i;
	return pos;
}


int PubSubNanode::connected() {
	// just return tracking that we have done a .connect call
	return rc;
}

// plan A
/*
int PubSubNanode::queWrite(uint8_t* buf, uint8_t length){
	// check for room in que ret 1 if full
	// copy buf to que tail
	// store lenght
	// inc tail and count
	// return
	if (queCount < QUE_SIZE) {
		for (int i=0 ; i <= length; i++) {
			que[queTail][i] = buf[i];
		}
		queLenght[queTail] = lenght;
		if (queTail == QUE_SIZE) {
			queTail = 0;
		} else {
			queTail++;
		}
		queCount++;
		if (queCount == 1 && resultWaitFlag == 0) {
			tcpReqFlag = 1;
		}
		return(1);
	} else {
		// que full
		return(0);
	}
	
}
*/

static uint16_t PubSubNanode::queRead(uint8_t fd){
	// check theres something in the que, if not ret 0
	// copy head to to buf
	// inc head, dec count
	// return plen
	uint16_t plen = 0; //packet lenght
	if (queCount != 0) {
		
		plen = _es.ES_fill_tcp_data_len(esBuf, 0, (char*)que[queHead], queLength[queHead]);
		
		//Serial.println("Data Fill")
		if (queHead == QUE_SIZE) {
			queHead = 0;
		} else {
			queHead++;
		}
		queCount--;
	}
	lastOutActivity = millis();

	return(plen);
}


