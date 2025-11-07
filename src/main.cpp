#include <Arduino.h>
#include "Wire.h"
#include "LittleFS.h"
#if defined(ESP8266)
#include <ESP8266WiFi.h>
// ESP8266 doesn't have Bluetooth
#else
#include <esp_wifi.h>
#include <esp_bt.h>
#endif

#include "ota.h"

// main config files
#include "globals.h"
#include "config/config.h"
#include "config/config_parser.h"
#include "logging/Logger.h"

// import modules
#include "wifi/osc.h"
#include "wifi/callbacks.h"
#include "PWM/PCA/pca.h"
#include "PWM/LEDC/ledc.h"
#include "serial/serial.h"

// testing
#include "testing/rampPWM.hpp"
#include "PWM/PWMUtils.hpp"

// Main Logger instance
Haptics::Logging::Logger logger("Main");

void setup()
{
	Serial.begin(115200);

#ifdef DEV_MODE
	// wait for serial if we are developing
	delay(100);
#endif

// Initialize LittleFS
#if defined(ESP8266)
	if (!LittleFS.begin())
	{ // ESP8 doesnt? have the format fucntion i guess
		logger.error("LittleFS mount failed, please restart");
		return;
	}
#else
	if (!LittleFS.begin(true))
	{
		logger.error("LittleFS mount failed, please restart");
		return;
	}
#endif

	Haptics::Conf::loadConfig();
	Haptics::initGlobals();

	Haptics::Wireless::Start(&Haptics::Conf::conf);
	OTA::otaSetup(OTA_PASS);
	Haptics::PCA::start(&Haptics::Conf::conf);
	Haptics::LEDC::start(&Haptics::Conf::conf);
}

void enterLimp()
{
	logger.warn("Thermal limit reached, entering limp mode");

	// kill radios
#if defined(ESP8266)
	WiFi.mode(WIFI_OFF);  // ESP8266 Wi-Fi off
	// ESP8266 doesn't have Bluetooth
	system_update_cpu_freq(80);  // ESP8266 throttle to 80MHz
	// ESP8266 sleep mode
	wifi_set_sleep_type(LIGHT_SLEEP_T);
#else
	esp_wifi_stop();			 // Wi‑Fi off
	esp_bt_controller_disable(); // BT off
	setCpuFrequencyMhz(80);      // throttle
	esp_light_sleep_start();     // enter sleep mode
#endif

	// wait until cool
	while (true)
	{
		delay(1000);
#if defined(ESP8266)
		// ESP8266 doesn't have built-in temperature sensor
		// Use a simple timeout instead of temperature monitoring
		static unsigned long limpStartTime = millis();
		if (millis() - limpStartTime > 60000) { // Cool down for 60 seconds
			ESP.restart();
		}
#else
		float temp = temperatureRead();

		// if temp isn't going down put into deep-sleep
		if (temp > MAX_TEMP + 5)
		{
			esp_deep_sleep(80000000000); // timeout in one day.
			esp_deep_sleep_start();
		}
		else if (temp < MIN_TEMP_COOLDOWN)
		{
			ESP.restart();
		}
#endif
	}
}

uint32_t ticks = 0;
time_t now = 0;
time_t lastSerialPush = millis();
time_t lastWifiTick = millis();
time_t lastOtaTick = millis();

// Profiler setup
#define TIMER_START uint32_t dwStart = ESP.getCycleCount();
#define TIMER_END Haptics::profiler.digitalWriteCycles += (ESP.getCycleCount() - dwStart);
uint32_t loopStart = 0;
uint32_t loopTotal = 0;
bool messageRecieved = false;

void loop()
{
	if (now - lastOtaTick >= OTA_UPDATE_MS) {
		OTA::otaUpdate();
		lastOtaTick = millis();
	}

	if (Haptics::globals.reinitLEDC)
	{ // prevents not defined error
		Haptics::LEDC::start(&Haptics::Conf::conf);
		logger.debug("Restarted LEDC");
		Haptics::globals.reinitLEDC = false;
	}

	Haptics::PCA::setPcaDuty(&Haptics::globals, &Haptics::Conf::conf);
	Haptics::SerialComm::tick();

	// Moves heavy lifting out of ISR's
	if (Haptics::globals.updatedMotors)
	{
		Haptics::globals.updatedMotors = false;
		Haptics::Wireless::updateMotorVals();
		#ifdef ESP8266
		Haptics::LEDC::tick(); // only needed on esp8266
		#endif
	}

	// Handle commands (like changing the config, not setting motor values.)
	if (Haptics::globals.processOscCommand)
	{
		// if we were sent a command over OSC
		messageRecieved = true;
		const String response = Haptics::Conf::Parser::parseInput(Haptics::globals.commandToProcess);
		OscMessage commandResponse(COMMAND_ADDRESS);
		commandResponse.pushString(response);
		Haptics::Wireless::oscClient.send(Haptics::Wireless::hostIP, Haptics::Wireless::sendPort, commandResponse);
		Haptics::globals.commandToProcess = "";
		Haptics::globals.processOscCommand = false;
	}
	else if (Haptics::globals.processSerCommand)
	{
		// If we were sent a command over serial
		String response = Haptics::Conf::Parser::parseInput(Haptics::globals.commandToProcess);
		Serial.println(response);
		Haptics::globals.commandToProcess = "";
		Haptics::globals.processSerCommand = false;
	}

	ticks += 1;
	now = millis();
	if (now - lastWifiTick >= 7)
	{ // Roughly 150hz
		Haptics::Wireless::Tick();
		lastWifiTick = now;
	}

	if (now - lastSerialPush >= 1000)
	{
		logger.debug("Loop/sec: %d", ticks);
		Haptics::Wireless::printMetrics();
		Haptics::PwmUtils::printAllDuty();
		logger.debug("THIS IS MY FORKED VERSION");

#if !defined(ESP8266)
		// ESP32 has temperature sensor
		float temp = temperatureRead();
		logger.debug("Temp: %.2f °C", temp);
		if (temp >= MAX_TEMP)
		{
			enterLimp();
		}
#endif

		// we should recieve atleast one message over a second if we are connected/
		// if we arent connected we should broadcast each second
		if (now - Haptics::lastPacketMs > 1000)
		{
			// Reset all motors to zero if we don't have a connection.
			for (uint16_t i = 0; i < MAX_MOTORS; i++) {
				Haptics::globals.allMotorVals[i] = 0;
			}
			Haptics::Wireless::Broadcast();
		}

		lastSerialPush = now;
		ticks = 0;
	}
}