/*
 Basic MQTT example 
 
  - connects to an MQTT server
  - publishes "hello world" to the topic "outTopic"
  - subscribes to the topic "inTopic"
*/

#include <EtherShield.h>
#include <PubSubNanode.h>

// Please modify the following lines. mac and ip have to be unique
// in your local area network. You can not have the same numbers in
// two devices:
// how did I get the mac addr? Translate the first 3 numbers into ascii is: TUX
static uint8_t mymac[6] = {
  0x54,0x55,0x58,0x10,0x00,0x25};

// The arduinos IP address
static uint8_t myip[4] = {
  192,168,0,180};

// The dest server IP address
static uint8_t server[4] = {
  192,168,0,210};
  
// The Gateway Ip address
// Can be the same as the server if on same subnet  
static uint8_t gwip[4] ={ 
  192,168,0,210};

#define MQTT_PORT 1883

void callback(char* topic, byte* payload,int length) {
  // handle message arrived
}


PubSubNanode client(server, MQTT_PORT, callback);

void setup()
{
  client.init(mymac, myip, gwip, MQTT_PORT);
  if (client.connect("arduinoClient")) {
    client.publish("outTopic","hello world");
    client.subscribe("inTopic");
  }
}

void loop()
{
  client.loop();
}

