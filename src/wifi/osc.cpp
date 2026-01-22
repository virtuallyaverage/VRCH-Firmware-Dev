#include "wifi/callbacks.h"
#include "logging/Logger.h"
#include "wifi/osc.h"
#include "globals.h"
#include "ota.h"

namespace Haptics
{
    namespace Wireless
    {

        void printRawPacket()
        {
            printRaw();
        }

        /// @brief start mDNS and OSC
        /// Must be able to return early if wifi doens't connect without issues.
        void Start(Haptics::Conf::Config *conf)
        {

            WiFi.mode(WIFI_STA);
            //WiFi.setSleep(false); Disabled in favor of fake packet sending.

            // Start WiFi connection
            WiFi.begin(conf->wifi_ssid, conf->wifi_password);
            logger.debug("Connecting to: %s", conf->wifi_ssid);

            // Wait for connection
            unsigned long startAttemptTime = millis();

            while (WiFi.status() != WL_CONNECTED && millis() - startAttemptTime < 10000)
            {
                delay(100); // Yield to background tasks
            }
            if (WiFi.status() != WL_CONNECTED)
            {
                logger.warn("WiFi connect failed");
                return;
            }

            switch (conf->transmit_power)
            {
            case 0:
#if defined(ESP8266)
                WiFi.setOutputPower(8.5);  // ESP8266 power setting
                logger.trace("Wi-Fi Power: LOW (8.5 dBm)");
#else
                WiFi.setTxPower(WIFI_POWER_8_5dBm);
                logger.trace("Wi-Fi Power: LOW (8.5 dBm)");
#endif
                break;
            case 1:
#if defined(ESP8266)
                WiFi.setOutputPower(15);   // ESP8266 power setting
                logger.trace("Wi-Fi Power: MEDIUM (15 dBm)");
#else
                WiFi.setTxPower(WIFI_POWER_15dBm);
                logger.trace("Wi-Fi Power: MEDIUM (15 dBm)");
#endif
                break;
            case 2:
#if defined(ESP8266)
                WiFi.setOutputPower(20.5); // ESP8266 max power
                logger.trace("Wi-Fi Power: HIGH (20.5 dBm)");
#else
                WiFi.setTxPower(WIFI_POWER_19_5dBm);
                logger.trace("Wi-Fi Power: HIGH (19.5 dBm)");
#endif
                break;
            }

            // Print the IP address
            const String selfIP = WiFi.localIP().toString();
            logger.debug("Connected @ %s", selfIP);

            OTA::otaSetup(OTA_PASS);

            // Start listening for OSC server
            OscWiFi.subscribe(RECIEVE_PORT, PING_ADDRESS, &handlePing);
            logger.debug("Server started on port: %d", RECIEVE_PORT);

            String mac = WiFi.macAddress();
            String ip = WiFi.localIP().toString();
            String name = conf->mdns_name;

            broadcastMessage = "{";
            broadcastMessage += "\"mac\":\"" + mac + "\",";
            broadcastMessage += "\"ip\":\"" + ip + "\",";
            broadcastMessage += "\"name\":\"" + name + "\",";
            broadcastMessage += "\"port\":" + String(recvPort);
            broadcastMessage += "}";

            // ESP8266 beginMulticast requires interface address
#if defined(ESP8266)
            udpClient.beginMulticast(WiFi.localIP(), IPAddress(MULTICAST_GROUP), MULTICAST_PORT);
#else
            udpClient.beginMulticast(IPAddress(MULTICAST_GROUP), MULTICAST_PORT);
#endif
            Broadcast(); // broadcast first time
        }

        void Broadcast()
        {
            // Send broadcast
            logger.debug("broadcast message: %s", broadcastMessage.c_str());
            udpClient.beginPacket(IPAddress(MULTICAST_GROUP), MULTICAST_PORT);
            udpClient.write((uint8_t *)broadcastMessage.c_str(), broadcastMessage.length()); // Send data
            udpClient.endPacket();                                                           // Ensure packet is sent
        }

        bool WiFiConnected()
        {
            return WiFi.status() == WL_CONNECTED;
        }

        void StartHeartBeat(String hostIP, uint16_t sendPort)
        {
            // Publish heart beat on one second intervals
            OscWiFi.publish(hostIP, sendPort, HEARTBEAT_ADDRESS)
                ->setFrameRate(2.);
            heartbeatPublisher = OscWiFi.getPublishElementRef(hostIP, sendPort, HEARTBEAT_ADDRESS);

            if (!heartbeatPublisher)
            {
                logger.warn("Heartbeat wasn't established");
            }
        }

        void handlePing(const OscMessage &message)
        {
            // if we recieve a ping and we were already setup, it is likely a server restart.
            // In that case the port, ip, and other values should be reinited

            sendPort = message.arg<uint16_t>(0); // Get the host's port from the message
            hostIP_str = message.remoteIP();         // Get the host's IP address
            hostIP.fromString(hostIP_str);

            // create our own recieving server
            OscWiFi.subscribe(RECIEVE_PORT, MOTOR_ADDRESS, &motorMessage_callback);
            OscWiFi.subscribe(RECIEVE_PORT, COMMAND_ADDRESS, &commandMessageCallback);

            logger.debug("Received ping from: %s", hostIP_str);

            // sending client
            oscClient = OscWiFi.getClient();

            // Respond to ping
            OscMessage pingResponse(PING_ADDRESS);
            pingResponse.pushInt32(RECIEVE_PORT);
            pingResponse.pushString(WiFi.macAddress());
            oscClient.send(hostIP_str, sendPort, pingResponse);
            logger.debug("Sending hrtbt to %s:%d", hostIP_str, sendPort);

            StartHeartBeat(hostIP_str, sendPort);
            globals.beenPinged = true;
        }

        /// @brief Push and pull OSC updates
        void Tick()
        {

            if (WiFi.status() != WL_CONNECTED) return;

            // run ota ticker
            if (otaTicker*WIRELESS_TICK_MS >= OTA_UPDATE_MS) {
                OTA::otaUpdate();
                otaTicker = 0;
            } else otaTicker += 1;

            // run keep alive packet sending
            switch (Haptics::Conf::conf.radio_keepalive_level) {
                case RADIO_KEEPALIVE_OFF:
                    break;
                case RADIO_KEEPALIVE_BALANCED:
                    if (keepaliveTicker*WIRELESS_TICK_MS >= RADIO_KEEPALIVE_BALANCED_MS) {
                        udpClient.beginPacket(hostIP, sendPort);
                        udpClient.endPacket();
                        keepaliveTicker = 0;
                    } else keepaliveTicker += 1;
                    break;
                case RADIO_KEEPALIVE_AGGRESSIVE:
                    if (keepaliveTicker*WIRELESS_TICK_MS >= RADIO_KEEPALIVE_AGGRESSIVE_MS) {
                        udpClient.beginPacket(hostIP, sendPort);
                        udpClient.endPacket();
                        keepaliveTicker = 0;
                    } else keepaliveTicker += 1;
                    break;
            }

            OscWiFi.update(); // should be called to subscribe + publish osc
        }

        /// @brief logs the usual wifi metrics to teh console.
        void printMetrics()
        {
            logger.debug("RSSI: %d dBm, Channel: %u",
                         WiFi.RSSI(),
                         WiFi.channel());
        }

    } // namespace Wireless
} // namespace Haptics
