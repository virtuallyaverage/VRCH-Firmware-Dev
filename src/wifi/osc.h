#include <ArduinoOSCWiFi.h>


// esp8266 specific defines:
#if defined(ESP8266)
    #include <ESP8266mDNS.h>
    #include <ESP8266WiFi.h>
#else 
    #include <WiFi.h>
    #include <WiFiUdp.h>
    #include <ESPmDNS.h>
#endif

#include "software_defines.h"
#include "globals.h"
#include "config/config.h"
#include "logging/Logger.h"
#include "wifi/callbacks.h"

#ifndef OSC_H
#define OSC_H

namespace Haptics {
namespace Wireless {

// OSC client to send messages back to the hosts
inline OscWiFiClient oscClient;
inline WiFiUDP udpClient;
inline String hostIP_str;
inline IPAddress hostIP;
inline uint16_t sendPort;
inline String broadcastMessage;

inline uint8_t keepaliveTicker; // counts intervals between radio management points.
inline uint8_t otaTicker;

//publisher references
inline OscPublishElementRef heartbeatPublisher;

// we need to get host ip first
inline String selfMac = WiFi.macAddress();
inline uint32_t recvPort = RECIEVE_PORT;

inline Logging::Logger logger("WIFI");

void StartHeartBeat( String hostIP, uint16_t sendPort);
void handlePing(const OscMessage& message);

void Broadcast();
void Start(Haptics::Conf::Config *conf);
bool WiFiConnected();
void Tick();
void printRawPacket();
void printMetrics();

} // namespace Wireless
} // namespace Haptics

#endif // OSC_H