/*
 * Name:		V-Lift.ino
 * Created:	27 August 2025
 * Author:		David Carrel
 */

// DAVE -  todo -- ADD duty-cycle
// DAVE - _version
// can softAP do more than 4?
// scanNetworks - make async - do I need to call it?

#include <bit>
#include <bitset>
#include <cstdint>
#include <iostream>
// Supporting files
#include "Definitions.h"
#include <Arduino.h>
#include <driver/rtc_io.h>
#include <WiFi.h>
#include <WiFiClient.h>
#ifdef USE_SSL
#include <WiFiClientSecure.h>
#endif // USE_SSL
#include <NetworkUdp.h>
#include <WebServer.h>
#ifndef MP_XIAO_ESP32C6
#define LED_BUILTIN 2
#endif // ! MP_XIAO_ESP32C6
#include <DNSServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <PubSubClient.h>
#include <ElegantOTA.h>
#ifdef USE_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif // USE_DISPLAY

#define popcount __builtin_popcount
void setButtonLEDs(int freq = 0);

// Device parameters
char _version[VERSION_STR_LEN] = "v2.04";
char myUniqueId[17];
char statusTopic[128];

int myPodNum = -1;
podState *myPod;
podState pods[NUM_PODS + 1];  // 0 is (virtual) system

// WiFi parameters
#ifdef USE_SSL
WiFiClientSecure _wifi;
const char* root_ca = ROOT_CA;
#else // USE_SSL
WiFiClient _wifi;
#endif // USE_SSL
#ifdef DAVE_WIFI_POWER
wifi_power_t wifiPower = WIFI_POWER_11dBm; // Will bump to max before setting
#endif // DAVE_WIFI_POWER

// MQTT parameters
PubSubClient _mqtt(_wifi);
char* _mqttPayload = NULL;
bool resendAllData = false;

// OTA setup
WebServer *otaServer = NULL;

NetworkUDP udp;
#ifdef DEBUG_UDP
unsigned int udpPacketsSent = 0;
unsigned int udpPacketsSentErrors = 0;
unsigned int udpPacketsReceived = 0;
#endif // DEBUG_UDP

volatile boolean configButtonPressed = false;
buttonState frontButtons = buttonState::nothingPressed;
buttonState remoteButtons = buttonState::nothingPressed;

#ifdef USE_DISPLAY
// OLED variables
char _oledOperatingIndicator = '*';
char _oledLine2[OLED_CHARACTER_WIDTH] = "";
char _oledLine3[OLED_CHARACTER_WIDTH] = "";
char _oledLine4[OLED_CHARACTER_WIDTH] = "";
#endif // USE_DISPLAY

// Config handling
Config config;

unsigned int wifiTries = 0;
#ifdef DEBUG_WIFI
uint32_t wifiReconnects = 0;
#endif // DEBUG_WIFI
#ifdef DEBUG_CALLBACKS
uint32_t receivedCallbacks = 0;
uint32_t unknownCallbacks = 0;
uint32_t badCallbacks = 0;
#endif // DEBUG_CALLBACKS

/*
 * Home Assistant auto-discovered values
 */
static struct mqttState _mqttAllEntities[] =
{
	// Entity,                                "Name",           allPods, Subscribe, Retain, HA Class
#ifdef DEBUG_FREEMEM
	{ mqttEntityId::entityFreemem,            "freemem",           false, false, false, homeAssistantClass::haClassInfo },
#endif
#ifdef DEBUG_CALLBACKS
	{ mqttEntityId::entityCallbacks,          "Callbacks",         false, false, false, homeAssistantClass::haClassInfo },
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_WIFI_DAVE_HACK
	{ mqttEntityId::entityRSSI,               "RSSI",              false, false, false, homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityBSSID,              "BSSID",             false, false, false, homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityTxPower,            "TX_Power",          false, false, false, homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityWifiRecon,          "reconnects",        false, false, false, homeAssistantClass::haClassInfo },
#endif // DEBUG_WIFI
#ifdef DEBUG_UPTIME
	{ mqttEntityId::entityUptime,             "Uptime",            false, false, false, homeAssistantClass::haClassDuration },
#endif // DEBUG_UPTIME
	{ mqttEntityId::entityVersion,            "Version",           true,  false, true,  homeAssistantClass::haClassInfo },
	{ mqttEntityId::entitySystemMode,         "System_Mode",       false, true,  true,  homeAssistantClass::haClassSelect },
	{ mqttEntityId::entityPodMode,            "Mode",              true,  false, true,  homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityPodAction,          "Action",            true,  false, true,  homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityPodPosition,        "Position",          true,  false, true,  homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityPodBatPct,          "Battery",           true,  false, true,  homeAssistantClass::haClassBattery },
	{ mqttEntityId::entityPodBatVlt,          "Battery_Voltage",   false, false, true,  homeAssistantClass::haClassVoltage },
	{ mqttEntityId::entityPodTopSensor,       "Top_Sensor",        true,  false, true,  homeAssistantClass::haClassMoisture },
	{ mqttEntityId::entityPodBotSensor,       "Bottom_Sensor",     true,  false, true,  homeAssistantClass::haClassMoisture }
};

// These timers are used in the main loop.
#define RUNSTATE_INTERVAL 2000
#define INTERVAL_ONE_SECOND 1000
#define INTERVAL_FIVE_SECONDS 5000
#define INTERVAL_TEN_SECONDS 10000
#define INTERVAL_THIRTY_SECONDS 30000
#define INTERVAL_ONE_MINUTE 60000
#define INTERVAL_FIVE_MINUTE 300000
#define INTERVAL_ONE_HOUR 3600000
#define INTERVAL_ONE_DAY 86400000
#define UPDATE_STATUS_BAR_INTERVAL 500
#define WIFI_RECONNECT_INTERVAL INTERVAL_ONE_MINUTE  // DAVE - change to 5 ??
#define STATUS_INTERVAL INTERVAL_TEN_SECONDS
#define DATA_INTERVAL INTERVAL_THIRTY_SECONDS
#define POD2POD_DATA_INTERVAL INTERVAL_FIVE_SECONDS

#define POD_DATA_IS_FRESH(_podNum) ((pods[_podNum].lastUpdate != 0) && (millis() - pods[_podNum].lastUpdate) < (4 * POD2POD_DATA_INTERVAL))

#ifdef USE_DISPLAY
// Pins GPIO22 and GPIO21 (SCL/SDA) if ESP32
// Pins GPIO23 and GPIO22 (SCL/SDA) if XIAO ESP32C6
Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 0);
#endif // USE_DISPLAY

/*
 * setup
 *
 * The setup function runs once when you press reset or power the board
 */
void setup()
{
	Preferences preferences;

	pinMode(UP_RELAY_PIN, OUTPUT);		// Configure pin for controlling relay
	pinMode(DOWN_RELAY_PIN, OUTPUT);	// Configure pin for controlling relay
	digitalWrite(UP_RELAY_PIN, RELAY_OFF);	// ... and turn off!!
	digitalWrite(DOWN_RELAY_PIN, RELAY_OFF);

	pinMode(LED_BUILTIN, OUTPUT);		// Configure LED for output
	pinMode(CONFIG_BUTTON_PIN, INPUT);	// Configure the config push button
	attachInterrupt(CONFIG_BUTTON_PIN, configButtonISR, FALLING);

#ifdef DEBUG_OVER_SERIAL
	Serial.begin(9600);
#endif // DEBUG_OVER_SERIAL

	// Wire.setClock(10000);

#ifdef USE_DISPLAY
	// Display time
	_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  // initialize OLED
	_display.clearDisplay();
	_display.display();
	updateOLED(false, myUniqueId, "Starting...", _version);
#endif // USE_DISPLAY

#ifdef DEBUG_OVER_SERIAL
	Serial.printf("Starting %s\n", myUniqueId);
	delay(500);
#endif

	for (int podIdx = 0; podIdx <= NUM_PODS; podIdx++) {
		pods[podIdx].lastUpdate = 0;
	}

	preferences.begin(DEVICE_NAME, true); // RO
	config.wifiSSID = preferences.getString(PREF_NAME_SSID, "");
	config.wifiPass = preferences.getString(PREF_NAME_PASS, "");
	config.mqttSrvr = preferences.getString(PREF_NAME_MQTT_SRV, "");
	config.mqttPort = preferences.getInt(PREF_NAME_MQTT_PORT, 0);
	config.mqttUser = preferences.getString(PREF_NAME_MQTT_USER, "");
	config.mqttPass = preferences.getString(PREF_NAME_MQTT_PASS, "");
#ifdef MP_XIAO_ESP32C6
	config.extAntenna = preferences.getBool(PREF_NAME_EXT_ANT, false);
#endif // MP_XIAO_ESP32C6
	config.podNumber = preferences.getInt(PREF_NAME_POD_NUM, -1);
	preferences.end();

	if ((config.podNumber < 1) || (config.podNumber > NUM_PODS)) {
		myPodNum = getPodNumFromButton();
		myPod = &pods[myPodNum];
		config.podNumber = myPodNum;
		preferences.begin(DEVICE_NAME, false); // RW
		preferences.putInt(PREF_NAME_POD_NUM, myPodNum);
		preferences.end();
	} else {
		myPodNum = config.podNumber;
		myPod = &pods[config.podNumber];
	}

//	WiFi.persistent(false);
//DAVE - DAVE
	WiFi.disconnect(true, true);
	WiFi.mode(WIFI_MODE_NULL);
	delay(100);

	// If config is not setup, then enter config mode
	if (myPodNum == 1) {
		uint8_t mac[6];
		// Set our unique identity.
		WiFi.mode(WIFI_AP_STA); // Kick WiFi just enough so MAC is readable
		WiFi.macAddress(mac);
		snprintf(myUniqueId, sizeof(myUniqueId), DEVICE_NAME "-%0X%02X%02X", mac[3] & 0xf, mac[4], mac[5]);
		snprintf(statusTopic, sizeof(statusTopic), DEVICE_NAME "/%s/status", myUniqueId);

		if ((config.wifiSSID == "") ||
		    (config.wifiPass == "") ||
		    (config.mqttSrvr == "") ||
		    (config.mqttPort == 0) ||
		    (config.mqttUser == "") ||
		    (config.mqttPass == "")) {
			configLoop();
			ESP.restart();
		}
	} else {
		WiFi.mode(WIFI_STA);
		snprintf(myUniqueId, sizeof(myUniqueId), DEVICE_NAME "-pod%d", myPodNum);
	}
	WiFi.hostname(myUniqueId);
#ifdef USE_DISPLAY
	{
		char line3[OLED_CHARACTER_WIDTH];
		snprintf(line3, sizeof(line3), "Pod # %d", myPodNum);
		updateOLED(false, "Config is set", line3, _version);
		delay(1000); // DAVE
	}
#endif // USE_DISPLAY

	pinMode(BAT_READ_PIN, INPUT);		// Configure pin for battery voltage
	pinMode(TOP_SENSOR_PIN, SENSOR_PIN_MODE);
	pinMode(BOT_SENSOR_PIN, SENSOR_PIN_MODE);

	pinMode(UP_BUTTON_PIN, BUTTON_MODE);
	pinMode(DOWN_BUTTON_PIN, BUTTON_MODE);
	pinMode(UP_LED_PIN, OUTPUT);
	pinMode(DOWN_LED_PIN, OUTPUT);

#ifdef MP_XIAO_ESP32C6
	pinMode(WIFI_ENABLE, OUTPUT);
	digitalWrite(WIFI_ENABLE, LOW);
	delay(100);
	pinMode(WIFI_ANT_CONFIG, OUTPUT);
	digitalWrite(WIFI_ANT_CONFIG, config.extAntenna ? HIGH : LOW);
#endif // MP_XIAO_ESP32C6

	// Configure WIFI
	checkWifiStatus(true);

	if (myPodNum == 1) {
#ifdef USE_SSL
		_wifi.setCACert(root_ca);
#endif // USE_SSL
		// Configure MQTT to the address and port specified above
		_mqtt.setServer(config.mqttSrvr.c_str(), config.mqttPort);
		_mqtt.setBufferSize(MAX_MQTT_PAYLOAD_SIZE + MQTT_HEADER_SIZE);
		_mqttPayload = new char[MAX_MQTT_PAYLOAD_SIZE];
		emptyPayload();

		// And any messages we are subscribed to will be pushed to the mqttCallback function for processing
		_mqtt.setCallback(mqttCallback);

		// Connect to MQTT
		if (WiFi.status() == WL_CONNECTED) {
			setupMqtt();
		}
	}

	// Initialize data.
	// myPodNum was set above.
	strlcpy(myPod->version, &_version[1], sizeof(myPod->version)); // skip initial 'v'
	readBattery();
	readPodState();
	pods[0].mode = modeUp;		// System mode
	pods[0].action = actionStop;	// System action (not used)
	pods[0].forceMode = false;
	pods[0].lastUpdate = millis();
	myPod->mode = modeUp;		// Always go to UP state when we boot.
	myPod->action = actionStop;	// and stop until loop reads position

#ifdef USE_DISPLAY
	updateOLED(false, myUniqueId, "setup complete", _version);
#endif // USE_DISPLAY
	for (int i = 0; i < 10; i++) {
		flashBuiltinLed(500);
		if (myPodNum == 1) {
			setButtonLEDs(500);
		}
		delay(500);
	}
}

int
getPodNumFromButton (void)
{
	int podNum = config.podNumber;

	if ((podNum < 1) || (podNum > NUM_PODS)) {
		podNum = 1;
	}

	for (;;) {
#ifdef USE_DISPLAY
		updateOLED(false, "Press button", "to start", "setting pod num.");
#endif // USE_DISPLAY
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Press button to start setting pod num.");
#endif // DEBUG_OVER_SERIAL
		if (configButtonPressed) {
			configButtonPressed = false;
			break;
		}
		delay(500);
	}
#define POD_NUM_SEC 15
	for (int sec = POD_NUM_SEC; sec > 0; sec--) {
		if (configButtonPressed) {
			configButtonPressed = false;
			podNum++;
			if (podNum > NUM_PODS) {
				podNum = 1;
			}
			sec = POD_NUM_SEC;
		}
#ifdef USE_DISPLAY
		char line3[OLED_CHARACTER_WIDTH], line4[OLED_CHARACTER_WIDTH];
		snprintf(line3, sizeof(line3), "        %d", podNum);
		snprintf(line4, sizeof(line4), "Saving in %d seconds", sec);
		updateOLED(false, "** Set Pod number **", line3, line4);
#endif // USE_DISPLAY
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Pod # %d.  Saving this value in %d seconds.\n", podNum, sec);
#endif // DEBUG_OVER_SERIAL
		delay(1000);
	}
	return podNum;
}

#ifdef DAVE_EXT_ANT
boolean
getExtAntFromButton (void)
{
	boolean extAnt = config.extAntenna;

#define EXT_ANT_SEC 15
	for (int sec = EXT_ANT_SEC; sec > 0; sec--) {
		if (configButtonPressed) {
			configButtonPressed = false;
			extAnt = !extAnt;
			sec = EXT_ANT_SEC;
		}
#ifdef USE_DISPLAY
		char line3[OLED_CHARACTER_WIDTH], line4[OLED_CHARACTER_WIDTH];
		snprintf(line3, sizeof(line3), "        %s", extAnt ? "On" : "Off");
		snprintf(line4, sizeof(line4), "Saving in %d seconds", sec);
		updateOLED(false, "** Set Ext Antenna *", line3, line4);
#endif // USE_DISPLAY
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Ext Antenna: %s.  Saving this value in %d seconds.\n", extAnt ? "On" : "Off", sec);
#endif // DEBUG_OVER_SERIAL
		delay(1000);
	}
	return extAnt;
}
#endif // DAVE_EXT_ANT

void
configLoop (void)
{
#ifdef USE_DISPLAY
	bool flip = false;
#endif // USE_DISPLAY

	for (int i = 0; ; i++) {
#ifdef USE_DISPLAY
		char line4[OLED_CHARACTER_WIDTH];

		if (i % 100 == 0) flip = !flip;
		snprintf(line4, sizeof(line4), "%d", i);
		if (flip) {
			updateOLED(false, "Config", "not set.", line4);
		} else {
			updateOLED(false, "Push", "button.", line4);
		}
#endif // USE_DISPLAY
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Configuration is not set. Push button.");
#endif

		flashBuiltinLed(250);
		if (myPodNum == 1) {
			setButtonLEDs(250);
		}

		// Read button state
		if (configButtonPressed) {
			configButtonPressed = false;
			break;
		}

		delay(30);
	}

	configHandler();
}

void
configHandler(void)
{
	Preferences preferences;
	char mqttPort[8];

	delete otaServer;
	otaServer = NULL;
	WiFi.disconnect();
	WiFi.softAPdisconnect();
	WiFi.mode(WIFI_MODE_NULL);
	delay(100);

	WiFiManager wifiManager;
	wifiManager.setBreakAfterConfig(true);
	wifiManager.setTitle(DEVICE_NAME);
//	wifiManager.setShowInfoUpdate(false);
//	wifiManager.setShowInfoErase(false);
	WiFiManagerParameter p_lineBreak_text("<p>MQTT settings:</p>");
	WiFiManagerParameter custom_mqtt_server("server", "MQTT server", config.mqttSrvr.c_str(), 40);
	if (config.mqttPort == 0) {
#ifdef USE_SSL
		strcpy(mqttPort, "8883");
#else // USE_SSL
		strcpy(mqttPort, "1883");
#endif // USE_SSL
	} else {
		snprintf(mqttPort, sizeof(mqttPort), "%d", config.mqttPort);
	}
	WiFiManagerParameter custom_mqtt_port("port", "MQTT port", mqttPort, 6);
	WiFiManagerParameter custom_mqtt_user("user", "MQTT user", config.mqttUser.c_str(), 32);
	WiFiManagerParameter custom_mqtt_pass("mpass", "MQTT password", "", 32);
#ifdef MP_XIAO_ESP32C6
	const char _customHtml_checkbox[] = "type=\"checkbox\"";
	WiFiManagerParameter custom_ext_ant("ext_antenna", "Use external WiFi antenna\n", "T", 2, _customHtml_checkbox, WFM_LABEL_AFTER);
#endif // MP_XIAO_ESP32C6

#ifdef MP_XIAO_ESP32C6
	wifiManager.addParameter(&custom_ext_ant);
#endif // MP_XIAO_ESP32C6
	wifiManager.addParameter(&p_lineBreak_text);
	wifiManager.addParameter(&custom_mqtt_server);
	wifiManager.addParameter(&custom_mqtt_port);
	wifiManager.addParameter(&custom_mqtt_user);
	wifiManager.addParameter(&custom_mqtt_pass);

#ifdef DEBUG_OVER_SERIAL
	Serial.println("Web config is active.");
#endif // DEBUG_OVER_SERIAL
#ifdef USE_DISPLAY
	updateOLED(false, "Web", "config", "active");
#endif // USE_DISPLAY

	if (!wifiManager.startConfigPortal(myUniqueId)) {
#ifdef DEBUG_OVER_SERIAL
		Serial.println("failed to connect and hit timeout");
#endif
#ifdef USE_DISPLAY
		updateOLED(false, "Web", "config", "failed");
#endif // USE_DISPLAY
		delay(3000);
		//reset and try again
		ESP.restart();
	}

	//if you get here you have connected to the WiFi
#ifdef DEBUG_OVER_SERIAL
	Serial.println("connected...yay :)");
#endif
#ifdef USE_DISPLAY
	updateOLED(false, "Web", "config", "succeeded");
#endif // USE_DISPLAY

	preferences.begin(DEVICE_NAME, false); // RW
	preferences.putString(PREF_NAME_SSID, wifiManager.getWiFiSSID());
	preferences.putString(PREF_NAME_PASS, wifiManager.getWiFiPass());
	preferences.putString(PREF_NAME_MQTT_SRV, custom_mqtt_server.getValue());
	{
		int port = strtol(custom_mqtt_port.getValue(), NULL, 10);
		if (port < 0 || port > SHRT_MAX)
			port = 0;
		preferences.putInt(PREF_NAME_MQTT_PORT, port);
	}
	preferences.putString(PREF_NAME_MQTT_USER, custom_mqtt_user.getValue());
	preferences.putString(PREF_NAME_MQTT_PASS, custom_mqtt_pass.getValue());
#ifdef MP_XIAO_ESP32C6
	{
		const char *extAnt = custom_ext_ant.getValue();
		preferences.putBool(PREF_NAME_EXT_ANT, extAnt[0] == 'T');
	}
#endif // MP_XIAO_ESP32C6
	preferences.end();

	delay(1000);
	ESP.restart();
}

/*
 * loop
 *
 * The loop function runs over and over again until power down or reset
 */
void
loop ()
{
#ifdef USE_DISPLAY
	static unsigned long lastRunDisplay = 0;
#endif // USE_DISPLAY
	static unsigned long lastRunStatus = 0;
	static unsigned long lastRunSendData = 0;
	static unsigned long lastRunPodData = 0;
	boolean wifiIsOn = false;
	boolean mqttIsOn = false;

	// Read button state
	if (configButtonPressed) {
		Preferences preferences;
		configButtonPressed = false;
		myPodNum = getPodNumFromButton();
#if defined(MP_XIAO_ESP32C6) && defined(DAVE_EXT_ANT)
		boolean extAnt = getExtAntFromButton();
#endif // MP_XIAO_ESP32C6 && DAVE_EXT_ANT
		preferences.begin(DEVICE_NAME, false); // RW
		preferences.putInt(PREF_NAME_POD_NUM, myPodNum);
#if defined(MP_XIAO_ESP32C6) && defined(DAVE_EXT_ANT)
		preferences.putBool(PREF_NAME_EXT_ANT, extAnt);
#endif // MP_XIAO_ESP32C6 && DAVE_EXT_ANT
		preferences.end();
		if (myPodNum == 1) {
//			preferences.begin(DEVICE_NAME, false); // RW
//			preferences.putString(PREF_NAME_SSID, "SFYCMEMBERS");
//			preferences.putString(PREF_NAME_PASS, "SFYC1869");
//			preferences.end();
// DAVE - hack
			configHandler();
		}
		ESP.restart();
	}

	// Make sure WiFi is good
	wifiIsOn = checkWifiStatus(false);

	if (myPodNum == 1) {
		// make sure mqtt is still connected
		if (wifiIsOn) {
			mqttIsOn = _mqtt.connected();
			if (!mqttIsOn || !_mqtt.loop()) {
				mqttIsOn = setupMqtt();
			}
		} else {
			mqttIsOn = false;
		}
	}

	// Handle OTA
	if (otaServer != NULL) {
		otaServer->handleClient();
		ElegantOTA.loop();
	}

	// Flash board LED (2 seconds)
	flashBuiltinLed(2000);

#ifdef USE_DISPLAY
	// Refresh LED Screen, will cause the status asterisk to flicker
	updateOLED(true, "", "", "");
#endif // USE_DISPLAY

	// Update the pod state
	readPodButtons();
	readBattery();
	readPodState();        // reads/sets sensors, and sets myPod->position

	// MQTT/P2P might have asynchronously updated pods[0].mode or buttons
	if (myPodNum == 1) {
		pods[0].batteryVolts = pods[1].batteryVolts;  // DAVE - hack.  Clean up
		getRemotePodStatus();
		readSystemModeFromButtons();  // Set system (pods[0]) mode based on buttons and system action
	} else {
		getSystemModeFromNumberOne();
		if (frontButtons != buttonState::nothingPressed) {
			remoteButtons = frontButtons;
			frontButtons = buttonState::nothingPressed;
			resendAllData = true;
		}
	}
	myPod->mode = pods[0].mode;
	// On remote pods, try waiting for state from #1 before activating the bootup default
	if ((myPodNum == 1) || pods[0].lastUpdate || (getUptimeSeconds() > 120)) {
		setPodAction();		// Set myPod->action
		engagePodAction();	// activate relays
	}

	if (resendAllData) {
		// Read and transmit all entity & HA data to MQTT
		lastRunSendData = 0;
		lastRunPodData = 0;
		lastRunDisplay = 0;
		resendAllData = false;
	}

	setButtonLEDs();

	if (myPodNum == 1) {
		if (checkTimer(&lastRunPodData, POD2POD_DATA_INTERVAL)) {
			sendCommandsToRemotePods();
		}

		// Send status/keepalive
		if (mqttIsOn && checkTimer(&lastRunStatus, STATUS_INTERVAL)) {
			sendStatus();
		}

		if (mqttIsOn && checkTimer(&lastRunSendData, DATA_INTERVAL)) {
			sendHaData();
			sendData();
		}
	} else {
		if (wifiIsOn && checkTimer(&lastRunPodData, POD2POD_DATA_INTERVAL)) {
			sendPodInfoToNumberOne();
		}
	}

#ifdef USE_DISPLAY
	// Check and display the runstate on the display
	if (checkTimer(&lastRunDisplay, RUNSTATE_INTERVAL)) {
		updateDisplayInfo();
	}
#endif // USE_DISPLAY
}

int
parseInt (char *data, const char *key)
{
	char *valueStr, *end, endOrig;
	int ret;

	valueStr = strstr(data, key);
	if (valueStr == NULL) {
		return -1;
	}
	valueStr += strlen(key);

	end = strchr(valueStr, ',');
	if (end != NULL) {
		endOrig = *end;
		*end = '\0';
	}

	ret = atoi(valueStr);

	if (end != NULL) {
		*end = endOrig;
	}

	return ret;
}

void
parseStr (char *data, const char *key, char *dest, size_t destLen)
{
	char *valueStr, *end, endOrig;

	valueStr = strstr(data, key);
	if (valueStr == NULL) {
		dest[0] = '\0';
		return;
	}
	valueStr += strlen(key);

	end = strchr(valueStr, ',');
	if (end != NULL) {
		endOrig = *end;
		*end = '\0';
	}

	strlcpy(dest, valueStr, destLen);

	if (end != NULL) {
		*end = endOrig;
	}
}

bool
parseBool (char *data, const char *key)
{
	char *valueStr;

	valueStr = strstr(data, key);
	if (valueStr == NULL) {
		return false;
	}
	valueStr += strlen(key);

	return *valueStr == '1';
}

#define UDP_BUF_SIZ 128
void
getSystemModeFromNumberOne (void)
{
	char buf[UDP_BUF_SIZ];
	int packetSize = udp.parsePacket();

	while (packetSize > 0) {
#if defined(DEBUG_UDP) && defined(DEBUG_OVER_SERIAL)
		Serial.print("Received packet of size ");
		Serial.println(packetSize);
		Serial.print("From ");
		IPAddress remoteIp = udp.remoteIP();
		Serial.print(remoteIp);
		Serial.print(", port ");
		Serial.println(udp.remotePort());
#endif // DEBUG_UDP && DEBUG_OVER_SERIAL

		// read the packet into buffer
		int podNum, len = udp.read(buf, sizeof(buf));
		if (len > 0) {
			buf[len] = 0;
#if defined(DEBUG_UDP) && defined(DEBUG_OVER_SERIAL)
			Serial.printf("Data is: %s\n", buf);
#endif // DEBUG_UDP && DEBUG_OVER_SERIAL
#ifdef DEBUG_UDP
			udpPacketsReceived++;
#endif // DEBUG_UDP
			podNum = parseInt(buf, "PN=");
			if (podNum == 0) {
				liftModes mode = (liftModes)parseInt(buf, "MO=");
				if (mode != pods[podNum].mode) {
					resendAllData = true;
				}
				pods[podNum].mode = mode;
//				pods[podNum].action = (liftActions)parseInt(buf, "AC=");
				pods[podNum].forceMode = parseBool(buf, "FM=");
				pods[podNum].lastUpdate = millis();
			}
		}
		packetSize = udp.parsePacket(); // Try to get another.
	}
}

void
sendCommandsToRemotePods (void)
{
	for (int podNum = 2; podNum <= NUM_PODS; podNum++) {
		IPAddress podIP(192, 168, PRIV_WIFI_SUBNET, podNum);

		udp.beginPacket(podIP, PRIV_UDP_PORT);
		udp.printf("PN=0,MO=%d,FM=%c", pods[0].mode, pods[0].forceMode ? '1' : '0');
#ifndef DEBUG_UDP
		udp.endPacket();
#else // ! DEBUG_UDP
		int err = udp.endPacket();
		if (err == 0) {
			udpPacketsSentErrors++;
		} else {
			udpPacketsSent++;
		}
#endif // ! DEBUG_UDP
	}
}

void
getRemotePodStatus (void)
{
	char buf[UDP_BUF_SIZ];
	int packetSize = udp.parsePacket();

	while (packetSize) {
#if defined(DEBUG_UDP) && defined(DEBUG_OVER_SERIAL)
		Serial.print("Received packet of size ");
		Serial.println(packetSize);
		Serial.print("From ");
		IPAddress remoteIp = udp.remoteIP();
		Serial.print(remoteIp);
		Serial.print(", port ");
		Serial.println(udp.remotePort());
#endif // DEBUG_OVER_UDP && DEBUG_OVER_SERIAL

		// read the packet into buffer
		int podNum, len = udp.read(buf, sizeof(buf));
		if (len > 0) {
			buf[len] = 0;
#if defined(DEBUG_UDP) && defined(DEBUG_OVER_SERIAL)
			Serial.printf("Data is: %s\n", buf);
#endif // DEBUG_UDP && DEBUG_OVER_SERIAL
#ifdef DEBUG_UDP
			udpPacketsReceived++;
#endif // DEBUG_UDP
			podNum = parseInt(buf, "PN=");
			if (podNum > 1 && podNum <= NUM_PODS) {
				pods[podNum].batteryPct = parseInt(buf, "BP=");
				{
					liftModes mode = (liftModes)parseInt(buf, "MO=");
					if (mode != pods[podNum].mode) resendAllData = true;
					pods[podNum].mode = mode;
				}
				{
					liftActions action = (liftActions)parseInt(buf, "AC=");
					if (action != pods[podNum].action) resendAllData = true;
					pods[podNum].action = action;
				}
				{
					liftPositions position = (liftPositions)parseInt(buf, "PO=");
					if (position != pods[podNum].position) resendAllData = true;
					pods[podNum].position = position;
				}
				{
					bool ts = parseBool(buf, "TS=");
					if (ts != pods[podNum].topSensorWet) resendAllData = true;
					pods[podNum].topSensorWet = ts;
				}
				{
					bool bs = parseBool(buf, "BS=");
					if (bs != pods[podNum].botSensorWet) resendAllData = true;
					pods[podNum].botSensorWet = bs;
				}
				{
					buttonState tmpButtons = (buttonState)parseInt(buf, "FB=");
					switch (tmpButtons) {
					case buttonState::upPressed:
					case buttonState::upLongPressed:
					case buttonState::downPressed:
					case buttonState::downLongPressed:
					case buttonState::bothPressed:
						remoteButtons = tmpButtons;
#ifdef DEBUG_OVER_SERIAL
						Serial.printf("Remote buttons set to %d\n", remoteButtons);
#endif // DEBUG_OVER_SERIAL
						break;
					case buttonState::nothingPressed:
					default:
						break;
					}
				}
				parseStr(buf, "VV=", pods[podNum].version, sizeof(pods[podNum].version));
				pods[podNum].lastUpdate = millis();
			}
		}
		packetSize = udp.parsePacket(); // Try to get another.
	}
}

void
sendPodInfoToNumberOne (void)
{
	IPAddress numOneIP(192, 168, PRIV_WIFI_SUBNET, 1);

	udp.beginPacket(numOneIP, PRIV_UDP_PORT);
	udp.printf("PN=%d,BP=%d,MO=%d,AC=%d,PO=%d,TS=%c,BS=%c,FB=%d,VV=%s", myPodNum, myPod->batteryPct,
		   myPod->mode, myPod->action, myPod->position, myPod->topSensorWet ? '1' : '0', myPod->botSensorWet ? '1' : '0',
		   remoteButtons, myPod->version);
	if (remoteButtons != buttonState::nothingPressed) {
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Sending Front buttons (%d) to #1.\n", remoteButtons);
#endif // DEBUG_OVER_SERIAL
		remoteButtons = buttonState::nothingPressed;
	}
#ifndef DEBUG_UDP
	udp.endPacket();
#else // ! DEBUG_UDP
	int err = udp.endPacket();
	if (err == 0) {
		udpPacketsSentErrors++;
	} else {
		udpPacketsSent++;
	}
#endif // ! DEBUG_UDP
}

// freq defaults to 0
void
setButtonLEDs (int freq)
{
	static unsigned long lastLed = 0;
	static boolean toggle = false;

	if (freq) {
		// if INIT, then fast alternate
		if (checkTimer(&lastLed, freq)) {
			digitalWrite(UP_LED_PIN, toggle ? BUTTON_LED_ON : BUTTON_LED_OFF);
			digitalWrite(DOWN_LED_PIN, toggle ? BUTTON_LED_OFF : BUTTON_LED_ON);
			toggle = !toggle;
		}
	} else if (myPod->mode == modeOff) {
		// if mode == off, then slow flash both
		if (checkTimer(&lastLed, 2000)) {
			digitalWrite(UP_LED_PIN, toggle ? BUTTON_LED_ON : BUTTON_LED_OFF);
			digitalWrite(DOWN_LED_PIN, toggle ? BUTTON_LED_ON : BUTTON_LED_OFF);
			toggle = !toggle;
		}
	} else if (myPod->action == actionRaise) {
		// if action raise, then turn on UP
		if (pods[0].forceMode) {
			if (checkTimer(&lastLed, 500)) {
				digitalWrite(UP_LED_PIN, toggle ? BUTTON_LED_ON : BUTTON_LED_OFF);
				toggle = !toggle;
			}
		} else {
			digitalWrite(UP_LED_PIN, BUTTON_LED_ON);
		}
		digitalWrite(DOWN_LED_PIN, BUTTON_LED_OFF);
	} else if (myPod->action == actionLower) {
		// if action lower, then turn on DOWN
		digitalWrite(UP_LED_PIN, BUTTON_LED_OFF);
		if (pods[0].forceMode) {
			if (checkTimer(&lastLed, 500)) {
				digitalWrite(DOWN_LED_PIN, toggle ? BUTTON_LED_ON : BUTTON_LED_OFF);
				toggle = !toggle;
			}
		} else {
			digitalWrite(DOWN_LED_PIN, BUTTON_LED_ON);
		}
	} else {
		// turn off
		digitalWrite(UP_LED_PIN, BUTTON_LED_OFF);
		digitalWrite(DOWN_LED_PIN, BUTTON_LED_OFF);
	}
}

void
setPodAction (void)
{
	liftActions action = actionStop;

	// Set pod relay actions
	if (pods[0].forceMode) {
		switch (pods[0].mode) {
		case modeUp:
			action = actionRaise;
			break;
		case modeDown:
			action = actionLower;
			break;
		case modeOff:
		default:
			action = actionStop;
			break;
		}
	} else {
		if ((myPod->mode == modeUp) || (myPod->mode == modeDown)) {
			switch (myPod->position) {
			case positionUp:
				action = (myPod->mode == modeUp) ? actionStop : actionLower;
				break;
			case positionAlmostUp:
			case positionMiddle:
			case positionAlmostDown:
				action = (myPod->mode == modeUp) ? actionRaise : actionLower;
				break;
			case positionDown:
				action = (myPod->mode == modeUp) ? actionRaise : actionStop;
				break;
			}
		}
	}
#ifdef DEBUG_OVER_SERIAL
	{
		static liftActions lastAction = actionStop;
		if (action != lastAction) {
			Serial.printf("Changing action from %d to %d.\n", lastAction, action);
			lastAction = action;
		}
	}
#endif // DEBUG_OVER_SERIAL
	myPod->action = action;
}

void
engagePodAction (void)
{
	switch(myPod->action) {
	case actionRaise:
		digitalWrite(UP_RELAY_PIN, RELAY_ON);
		digitalWrite(DOWN_RELAY_PIN, RELAY_OFF);
		break;
	case actionLower:
		digitalWrite(UP_RELAY_PIN, RELAY_OFF);
		digitalWrite(DOWN_RELAY_PIN, RELAY_ON);
		break;
	case actionStop:
	default:  // Shouldn't happen.
		digitalWrite(UP_RELAY_PIN, RELAY_OFF);
		digitalWrite(DOWN_RELAY_PIN, RELAY_OFF);
		break;
	}
}

// Read the buttons and save
void
readSystemModeFromButtons(void)
{
	buttonState buttons = frontButtons;

	if (buttons == buttonState::nothingPressed) {
		buttons = remoteButtons;
	}
	frontButtons = buttonState::nothingPressed;
	remoteButtons = buttonState::nothingPressed;

	switch (buttons) {
	case buttonState::nothingPressed:
		break;
	case buttonState::upPressed:
		pods[0].mode = modeUp;
		pods[0].forceMode = false;
		pods[0].lastUpdate = millis();
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Up button press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState::downPressed:
		pods[0].mode = modeDown;
		pods[0].forceMode = false;
		pods[0].lastUpdate = millis();
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Down button press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState::upLongPressed:
		pods[0].mode = modeUp;
		pods[0].forceMode = true;
		pods[0].lastUpdate = millis();
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Up button LONG press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState::downLongPressed:
		pods[0].mode = modeDown;
		pods[0].forceMode = true;
		pods[0].lastUpdate = millis();
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Down button LONG press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState:: bothPressed:
		pods[0].mode = modeOff;
		pods[0].forceMode = false;
		pods[0].lastUpdate = millis();
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Both buttons press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	}
}

// Read pod water sensors and determine position
// Sets topSensorWet, botSensorWet, and position
void
readPodState(void)
{
	boolean topWet = false, botWet = false;
	int bottomCount = 0;
	uint32_t now;
	static uint32_t almostDownTime = 0, almostUpTime = 0;
	liftPositions newPosition = myPod->position;

#define SENSOR_NUM_SAMPLES 16
	for (int i = 0; i < SENSOR_NUM_SAMPLES; i++) {
		if (digitalRead(TOP_SENSOR_PIN) == SENSOR_WET) {
			// If ANY sample is true, set it as "wet"
			topWet = true;
		}
		if (digitalRead(BOT_SENSOR_PIN) == SENSOR_WET) {
			// If ANY sample is true, set it as "wet"
			bottomCount++;
		}
	}
	if (bottomCount > (SENSOR_NUM_SAMPLES / 2)) {
		// If half the readings are wet, then we are wet.
		botWet = true;
	}

#ifdef DEBUG_OVER_SERIAL
	if (myPod->topSensorWet != topWet) {
		Serial.printf("Top sensor changed from %s to %s.\n", myPod->topSensorWet ? "WET" : "DRY", topWet ? "WET" : "DRY");
	}
	if (myPod->botSensorWet != botWet) {
		Serial.printf("Bottom sensor changed from %s to %s.\n", myPod->botSensorWet ? "WET" : "DRY", botWet ? "WET" : "DRY");
	}
#endif // DEBUG_OVER_SERIAL
	myPod->topSensorWet = topWet;
	myPod->botSensorWet = botWet;

	now = millis();
	if (topWet) {
		if (myPod->position == positionMiddle) {
			newPosition = positionAlmostDown;
			almostDownTime = now;
		} else if ((myPod->position == positionAlmostDown) &&
			   ((almostDownTime + ALMOST_DOWN_DELAY) > now)) {
			// Do nothing until delay passes
		} else {
			newPosition = positionDown;
			almostDownTime = 0;
		}
		almostUpTime = 0;
	} else if (botWet) {
		newPosition = positionMiddle;
		almostDownTime = 0;
		almostUpTime = 0;
	} else /* neither wet */ {
		if (myPod->position == positionMiddle) {
			newPosition = positionAlmostUp;
			almostUpTime = now;
		} else if ((myPod->position == positionAlmostUp) &&
			   ((almostUpTime + ALMOST_UP_DELAY) > now)) {
			// Do nothing until delay passes
		} else {
			newPosition = positionUp;
			almostUpTime = 0;
		}
		almostDownTime = 0;
	}
#ifdef DEBUG_OVER_SERIAL
	if (myPod->position != newPosition) {
		Serial.printf("Position changed from %d to %d.\n", myPod->position, newPosition);
	}
#endif // #ifdef DEBUG_OVER_SERIAL
	myPod->position = newPosition;
	myPod->lastUpdate = now;
}

void
readPodButtons(void)
{
	static unsigned long lastRun = 0;
	static unsigned long upPressedMillis = 0, downPressedMillis = 0;
#define BUTTON_INTERVAL 50
#define LONG_PRESS_MILLIS 3000 // 3 seconds

	if (checkTimer(&lastRun, BUTTON_INTERVAL)) {
		boolean upPressed = (digitalRead(UP_BUTTON_PIN) == BUTTON_PRESSED);
		boolean downPressed = (digitalRead(DOWN_BUTTON_PIN) == BUTTON_PRESSED);
		unsigned long now = millis();
		if (upPressed && downPressed && ((upPressedMillis == 0) || (downPressedMillis == 0))) {
			upPressedMillis = now;
			downPressedMillis = now;
		} else if (upPressed && (upPressedMillis == 0)) {
			upPressedMillis = now;
		} else if (downPressed && (downPressedMillis == 0)) {
			downPressedMillis = now;
		}
		if ((upPressedMillis != 0) && (downPressedMillis != 0)) {
			if (!upPressed && !downPressed) {
				frontButtons = buttonState::bothPressed;
				upPressedMillis = 0;
				downPressedMillis = 0;
			}
		} else if (upPressedMillis != 0) {
			if (!upPressed) {
				if ((now - upPressedMillis) > LONG_PRESS_MILLIS) {
					frontButtons = buttonState::upLongPressed;
				} else {
					frontButtons = buttonState::upPressed;
				}
				upPressedMillis = 0;
			}
		} else if (downPressedMillis != 0) {
			if (!downPressed) {
				if ((now - downPressedMillis) > LONG_PRESS_MILLIS) {
					frontButtons = buttonState::downLongPressed;
				} else {
					frontButtons = buttonState:: downPressed;
				}
				downPressedMillis = 0;
			}
		} else {
#ifdef DEBUG_OVER_SERIAL
			if (frontButtons != buttonState::nothingPressed) {
				Serial.printf("Front buttons unset from %d\n", frontButtons);
			}
#endif // DEBUG_OVER_SERIAL
			frontButtons = buttonState::nothingPressed;
		}
#ifdef DEBUG_OVER_SERIAL
		if (frontButtons != buttonState::nothingPressed) {
			Serial.printf("Front buttons set to %d\n", frontButtons);
		}
#endif // DEBUG_OVER_SERIAL
	}
}

// Read battery state
void
readBattery(void)
{
	float volts, pct;
	uint32_t mvBatt = 0;

#define BAT_NUM_SAMPLES 16
	for (int i = 0; i < BAT_NUM_SAMPLES; i++) {
		mvBatt += analogReadMilliVolts(BAT_READ_PIN); // Read and accumulate ADC voltage
	}
	volts = ((BAT_VOLTAGE_DIVIDER * mvBatt) / BAT_NUM_SAMPLES) / 1000.0;     // Adjust for divider and convert to volts
#ifdef BAT_MULTIPLIER
	volts *= BAT_MULTIPLIER;
#endif // BAT_MULTIPLIER

	pct = ((volts - BAT_MIN) / (BAT_MAX - BAT_MIN)) * 100.0;
	if (pct > 100) {
		pct = 100;
	} else if (pct < 0) {
		pct = 0;
	}

	myPod->batteryPct = pct;
	myPod->batteryVolts = volts;
}


uint32_t
getUptimeSeconds(void)
{
	static uint32_t uptimeSeconds = 0;
	static uint32_t uptimeWrapSeconds = 0;
	uint32_t nowSeconds = millis() / 1000;

	if (nowSeconds < uptimeSeconds) {
		// We wrapped
		uptimeWrapSeconds += (UINT32_MAX / 1000);;
	}
	uptimeSeconds = nowSeconds;
	return uptimeWrapSeconds + uptimeSeconds;
}

/*
 * setupWifi
 *
 * Connect to WiFi
 */
void
setupWifi(/*unsigned int tries*/)
{
	IPAddress privIP(192, 168, PRIV_WIFI_SUBNET, myPodNum); // IP for the AP
	IPAddress privGateway(192, 168, PRIV_WIFI_SUBNET, 1); // Gateway IP (same as local_IP for AP)
	IPAddress privSubnet(255, 255, 255, 0); // Subnet mask

	if (myPodNum == 1) {
		WiFi.disconnect();
		WiFi.softAPdisconnect();
		WiFi.mode(WIFI_MODE_NULL);
		delay(100);

		// Set up in AP & Station Mode
		WiFi.mode(WIFI_AP_STA);
		delay(100);
		WiFi.hostname(myUniqueId);
		// Helps when multiple APs for our SSID
//		WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
//		WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
//		WiFi.softAP(PRIV_WIFI_SSID, PRIV_WIFI_PASS);
		WiFi.softAPConfig(privIP, privGateway, privSubnet);
		delay(100);
		WiFi.softAP(PRIV_WIFI_SSID, PRIV_WIFI_PASS, MY_WIFI_CHANNEL, 0, NUM_PODS);
		delay(100);

		// Set these up here for pod #1  because SoftAP never goes away
		udp.begin(privIP, PRIV_UDP_PORT);
		otaServer = new WebServer(privIP, 80);
		ElegantOTA.begin(otaServer);    // Start ElegantOTA
		otaServer->begin();

//		// Helps when multiple APs for our SSID
//		WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
//		WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
//		WiFi.enableLongRange(true);
//		delay(100);
		WiFi.begin(config.wifiSSID.c_str(), config.wifiPass.c_str(), MY_WIFI_CHANNEL);
	} else {
		WiFi.disconnect();
		WiFi.mode(WIFI_MODE_NULL);
		delay(100);

		// Set up in Station Mode - Will be connecting to #1's access point
		WiFi.mode(WIFI_STA);
		delay(100);
		WiFi.hostname(myUniqueId);
		WiFi.config(privIP, privGateway, privGateway, privSubnet);
		delay(100);
		WiFi.begin(PRIV_WIFI_SSID, PRIV_WIFI_PASS);
	}
	WiFi.setAutoReconnect(true);

#ifdef DAVE_WIFI_POWER
	if (tries != 0) { // Don't change/set power the first time through.
		switch (wifiPower) {
		case WIFI_POWER_19_5dBm:
			wifiPower = WIFI_POWER_19dBm;
			break;
		case WIFI_POWER_19dBm:
			wifiPower = WIFI_POWER_18_5dBm;
			break;
		case WIFI_POWER_18_5dBm:
			wifiPower = WIFI_POWER_17dBm;
			break;
		case WIFI_POWER_17dBm:
			wifiPower = WIFI_POWER_15dBm;
			break;
		case WIFI_POWER_15dBm:
			wifiPower = WIFI_POWER_13dBm;
			break;
		case WIFI_POWER_13dBm:
			wifiPower = WIFI_POWER_11dBm;
			break;
		case WIFI_POWER_11dBm:
		default:
			wifiPower = WIFI_POWER_19_5dBm;
			break;
		}
		WiFi.setTxPower(wifiPower);
	}
#endif // DAVE_WIFI_POWER
}

boolean
checkWifiStatus(boolean initialConnect)
{
	static unsigned long lastWifiTry = 0;
	static uint8_t previousWifiStatus = 0;
	uint8_t status;

	status = WiFi.status();
	if (initialConnect || (status != WL_CONNECTED)) {
		if (previousWifiStatus == WL_CONNECTED) {
			if (myPodNum != 1) {
				udp.clear();
				udp.stop();
				if (otaServer) {
					delete otaServer;
					otaServer = NULL;
				}
			}
		}
		if (checkTimer(&lastWifiTry, WIFI_RECONNECT_INTERVAL)) {
			if (initialConnect) {
#ifdef DEBUG_OVER_SERIAL
				Serial.printf("Connecting to %s\n", myPodNum == 1 ? config.wifiSSID.c_str() : PRIV_WIFI_SSID);
#endif // DEBUG_OVER_SERIAL
				setupWifi(/*wifiTries*/);
			} else {
#ifdef DEBUG_OVER_SERIAL
				Serial.printf("Reconnect to %s\n", myPodNum == 1 ? config.wifiSSID.c_str() : PRIV_WIFI_SSID);
#endif // DEBUG_OVER_SERIAL
//				WiFi.disconnect();
//				delay(100);
				WiFi.reconnect();
			}
			wifiTries++;
		}
	}

	if (status == WL_CONNECTED) {
		wifiTries = 0;
		lastWifiTry = millis();
	}
	if ((status == WL_CONNECTED) && (previousWifiStatus != WL_CONNECTED)) {
		IPAddress privIP(192, 168, PRIV_WIFI_SUBNET, myPodNum);
#ifdef DEBUG_WIFI
		wifiReconnects++;
#endif // DEBUG_WIFI
		if (myPodNum != 1) {
// DAVE - can any of these block?
			udp.begin(privIP, PRIV_UDP_PORT);
			otaServer = new WebServer(privIP, 80);
			ElegantOTA.begin(otaServer);    // Start ElegantOTA
			otaServer->begin();
		}
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("WiFi IP is %s (%s)\n", WiFi.localIP().toString().c_str(), WiFi.SSID().c_str());
		byte *bssid = WiFi.BSSID();
		Serial.printf("WiFi BSSID is %02X:%02X:%02X:%02X:%02X:%02X\n", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		Serial.printf("WiFi RSSI: %hhd\n", WiFi.RSSI());
		Serial.printf("WiFi channel: %ld\n", WiFi.channel());
		if (myPodNum == 1) {
			Serial.printf("softAP IP address: %s : %s (%s)\n", WiFi.softAPIP().toString().c_str(),
				      WiFi.softAPSubnetMask().toString().c_str(), WiFi.softAPSSID().c_str());
		}
#endif // DEBUG_OVER_SERIAL
	}
	previousWifiStatus = status;

	return status == WL_CONNECTED;
}

/*
 * checkTimer
 *
 * Check to see if the elapsed interval has passed since the passed in millis() value. If it has, return true and update the lastRun.
 * Note that millis() overflows after 50 days, so we need to deal with that too... in our case we just zero the last run, which means the timer
 * could be shorter but it's not critical... not worth the extra effort of doing it properly for once in 50 days.
 */
bool
checkTimer(unsigned long *lastRun, unsigned long interval)
{
	unsigned long now = millis();

	if (*lastRun > now)
		*lastRun = 0;

	if (*lastRun == 0 || now >= *lastRun + interval) {
		*lastRun = now;
		return true;
	}

	return false;
}

void
flashBuiltinLed(int freq)
{
	static unsigned long lastLed = 0;

	if (checkTimer(&lastLed, freq)) {
		digitalWrite(LED_BUILTIN, LOW);  // On
		delay(25);
		digitalWrite(LED_BUILTIN, HIGH); // Off
	}
}

#ifdef USE_DISPLAY
#define CURSOR_LINE_1 0
#define CURSOR_LINE_2 ((SCREEN_HEIGHT / 4) * 1)
#define CURSOR_LINE_3 ((SCREEN_HEIGHT / 4) * 2)
#define CURSOR_LINE_4 ((SCREEN_HEIGHT / 4) * 3)

/*
 * updateOLED
 *
 * Update the OLED. Use "NULL" for no change to a line or "" for an empty line.
 * Three parameters representing each of the three lines available for status indication - Top line functionality fixed
 */
void
updateOLED(bool justStatus, const char* line2, const char* line3, const char* line4)
{
	static unsigned long updateStatusBar = 0;
	int8_t rssi;

	_display.clearDisplay();
	_display.setTextSize(1);
	_display.setTextColor(WHITE);
	_display.setCursor(0, CURSOR_LINE_1);

	char line1[OLED_CHARACTER_WIDTH];

	// Only update the operating indicator once per half second.
	if (checkTimer(&updateStatusBar, UPDATE_STATUS_BAR_INTERVAL)) {
		// Simply swap between space and asterisk every time we come here to give some indication of activity
		_oledOperatingIndicator = (_oledOperatingIndicator == '*') ? ' ' : '*';
	}

	rssi = WiFi.RSSI();
	// There's 20 characters we can play with, width wise.
	{
		char wifiStatus, mqttStatus;

		wifiStatus = mqttStatus = ' ';
		wifiStatus = WiFi.status() == WL_CONNECTED ? 'W' : ' ';
		mqttStatus = _mqtt.connected() && _mqtt.loop() ? 'M' : ' ';
		snprintf(line1, sizeof(line1), "Pod%d %c%c%c         %3hhd",
			 myPodNum, _oledOperatingIndicator, wifiStatus, mqttStatus, rssi);
	}
	_display.println(line1);
	printWifiBars(rssi);

	// Next line
	_display.setCursor(0, CURSOR_LINE_2);
	if (!justStatus && line2) {
		strcpy(_oledLine2, line2);
	}
	_display.println(_oledLine2);

	_display.setCursor(0, CURSOR_LINE_3);
	if (!justStatus && line3) {
		strcpy(_oledLine3, line3);
	}
	_display.println(_oledLine3);

	_display.setCursor(0, CURSOR_LINE_4);
	if (!justStatus && line4) {
		strcpy(_oledLine4, line4);
	}
	_display.println(_oledLine4);

	// Refresh the display
	_display.display();
}

#define WIFI_X_POS 75 //102
void
printWifiBars(int rssi)
{
	if (rssi >= -55) { 
		_display.fillRect((WIFI_X_POS + 0),7,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.fillRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.fillRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.fillRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -55 && rssi > -65) {
		_display.fillRect((WIFI_X_POS + 0),7,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.fillRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.fillRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -65 && rssi > -75) {
		_display.fillRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.fillRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,2,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -75 && rssi > -85) {
		_display.fillRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.fillRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.drawRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else if (rssi < -85 && rssi > -96) {
		_display.fillRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.drawRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.drawRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	} else {
		_display.drawRect((WIFI_X_POS + 0),8,4,1, WHITE);
		_display.drawRect((WIFI_X_POS + 5),6,4,2, WHITE);
		_display.drawRect((WIFI_X_POS + 10),4,4,4, WHITE);
		_display.drawRect((WIFI_X_POS + 15),2,4,6, WHITE);
		_display.drawRect((WIFI_X_POS + 20),0,4,8, WHITE);
	}
}

/*
 * updateDisplayInfo
 *
 * Determines a few things about the sytem and updates the display
 * Things updated - Dispatch state discharge/charge, battery power, battery percent
 */
void
updateDisplayInfo()
{
	static int dbgIdx = 0;
	char line2[OLED_CHARACTER_WIDTH] = "";
	char line3[OLED_CHARACTER_WIDTH] = "";
	char line4[OLED_CHARACTER_WIDTH] = "";

	// Get system status info for line 2
	{
		const char *mode;
		int8_t activeSys = 0, activePeers = 0, activeWiFi = 0;

		switch (pods[0].mode) {
		case modeUp:
			mode = "Up  ";
			break;
		case modeDown:
			mode = "Down";
			break;
		case modeOff:
			mode = "Off ";
			break;
		default:
			mode = "XXXX";
			break;
		}
		if (myPodNum == 1) {
			activeSys = 1;  // Pod 1 always knows system state.
			for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
				if (POD_DATA_IS_FRESH(podNum)) {
					activePeers++;
				}
			}
			activeWiFi = WiFi.softAPgetStationNum();
		} else {
			activeSys = POD_DATA_IS_FRESH(0) ? 1 : 0;
			activePeers = POD_DATA_IS_FRESH(myPodNum) ? 1 : 0;
			activeWiFi = WiFi.status() == WL_CONNECTED ? 1 : 0;
		}
		snprintf(line2, sizeof(line2), "%s : %hhd/%hhd/%hhd : %s", mode, activeSys, activePeers, activeWiFi,
			 pods[0].forceMode ? "FORCE" : "NORM"); // 4 + 3 + 5 + 3 + 5
	}

	// Pod status for line 3
	{
		const char *mode, *action, *position;

		switch (myPod->mode) {
		case modeUp:
			mode = "Up  ";
			break;
		case modeDown:
			mode = "Down";
			break;
		case modeOff:
			mode = "Off ";
			break;
		default:
			mode = "XXXX";
			break;
		}
		switch (myPod->action) {
		case actionStop:
			action = "Stop ";
			break;
		case actionRaise:
			action = "Raise";
			break;
		case actionLower:
			action = "Lower";
			break;
		default:
			action = "XXXXX";
			break;
		}
		switch (myPod->position) {
		case positionUp:
			position = "Up   ";
			break;
		case positionAlmostUp:
			position = "~Up  ";
			break;
		case positionMiddle:
			position = "Mid  ";
			break;
		case positionAlmostDown:
			position = "~Down";
			break;
		case positionDown:
			position = "Down ";
			break;
		default:
			position = "XXXXX";
			break;
		}
		snprintf(line3, sizeof(line3), "%s : %s : %s", mode, action, position);  // 4 + 3 + 5 + 3 + 5
	}

	if (dbgIdx < 1) {
		snprintf(line4, sizeof(line4), "ID: %s", myUniqueId);
		dbgIdx = 1;
#ifdef DEBUG_FREEMEM
	} else if (dbgIdx < 2) {
		snprintf(line4, sizeof(line4), "Mem: %u", freeMemory());
		dbgIdx = 2;
#endif // DEBUG_FREEMEM
#ifdef DEBUG_WIFI
	} else if (dbgIdx < 3) {
		snprintf(line4, sizeof(line4), "WiFi recon: %lu", wifiReconnects);
		dbgIdx = 3;
	} else if (dbgIdx < 4) {
		snprintf(line4, sizeof(line4), "WiFi TX: %0.01fdBm", (int)WiFi.getTxPower() / 4.0f);
		dbgIdx = 4;
	} else if (dbgIdx < 5) {
		snprintf(line4, sizeof(line4), "WiFi tries: %d", wifiTries);
		dbgIdx = 5;
#endif // DEBUG_WIFI
#ifdef DEBUG_CALLBACKS
	} else if (dbgIdx < 6) {
		snprintf(line4, sizeof(line4), "Callbacks: %lu", receivedCallbacks);
		dbgIdx = 6;
	} else if (dbgIdx < 7) {
		snprintf(line4, sizeof(line4), "Unk CBs: %lu", unknownCallbacks);
		dbgIdx = 7;
	} else if (dbgIdx < 8) {
		snprintf(line4, sizeof(line4), "Bad CBs: %lu", badCallbacks);
		dbgIdx = 8;
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_UPTIME
	} else if (dbgIdx < 9) {
		snprintf(line4, sizeof(line4), "Uptime: %lu", getUptimeSeconds());
		dbgIdx = 9;
#endif // DEBUG_UPTIME
#ifdef DEBUG_UDP
	} else if (dbgIdx < 10) {
		snprintf(line4, sizeof(line4), "UDP: S=%u/%u  R=%u", udpPacketsSent, udpPacketsSentErrors, udpPacketsReceived);
		dbgIdx = 10;
#endif // DEBUG_UDP
	} else if (dbgIdx < 11) {
		snprintf(line4, sizeof(line4), "Bat: %d%%  %0.02fV", myPod->batteryPct, myPod->batteryVolts);
		dbgIdx = 11;
#ifdef DAVE_EXT_ANT
	} else if (dbgIdx < 12) {
		snprintf(line4, sizeof(line4), "Ext Ant: %s", config.extAntenna ? "On" : "Off");
		dbgIdx = 12;
#endif // DAVE_EXT_ANT
	} else { // Must be last
		snprintf(line4, sizeof(line4), "Version: %s", _version);
		dbgIdx = 0;
	}

	updateOLED(false, line2, line3, line4);
}
#endif // USE_DISPLAY

/*
 * setupMqtt
 *
 * This function reconnects to the MQTT broker
 */
boolean
setupMqtt(void)
{
	bool subscribed = false;
	char subscriptionDef[100];
	static int tries = 0;
	static boolean cleanSession = true;

	_mqtt.disconnect();		// Just in case.

	// Wait 2 seconds before retrying
	if (tries != 0) {
		delay(2000);
	}

#ifdef DEBUG_OVER_SERIAL
	Serial.println("Attempting MQTT connection...");
#endif

	// Attempt to connect
	char lwt[256];
	strcpy(lwt, "{ \"systemStatus\": \"offline\"");
	for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
		char podLwt[32];
		sprintf(podLwt, ", \"pod%dStatus\": \"unavailable\"", podNum);
		strlcat(lwt, podLwt, sizeof(lwt));
	}
	strlcat(lwt, " }", sizeof(lwt));
	if (_mqtt.connect(myUniqueId, config.mqttUser.c_str(), config.mqttPass.c_str(), statusTopic, 0, true, lwt, cleanSession)) {
		int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Connected MQTT");
#endif

		// Special case for Home Assistant
		sprintf(subscriptionDef, "%s", MQTT_SUB_HOMEASSISTANT);
		subscribed = _mqtt.subscribe(subscriptionDef, MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Subscribed to \"%s\" : %d\n", subscriptionDef, subscribed);
#endif

		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].subscribe) {
				sprintf(subscriptionDef, DEVICE_NAME "/%s/%s/command", myUniqueId, _mqttAllEntities[i].mqttName);
				subscribed = subscribed && _mqtt.subscribe(subscriptionDef, MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG_OVER_SERIAL
				Serial.printf("Subscribed to \"%s\" : %d\n", subscriptionDef, subscribed);
#endif
			}
		}
	}

	// subscribed indicates MQTT overall setup status
	if (subscribed) {
		cleanSession = false;  // Once we set up the first time, any reconnects should be persistent sessions
		tries = 0;
	} else {
		tries++;
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("MQTT Failed: RC is %d\n", _mqtt.state());
#endif
	}
	return subscribed;
}

mqttState *
lookupSubscription(char *entityName)
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
	for (int i = 0; i < numberOfEntities; i++) {
		if (_mqttAllEntities[i].subscribe &&
		    !strcmp(entityName, _mqttAllEntities[i].mqttName)) {
			return &_mqttAllEntities[i];
		}
	}
	return NULL;
}

mqttState *
lookupEntity(mqttEntityId entityId)
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
	for (int i = 0; i < numberOfEntities; i++) {
		if (_mqttAllEntities[i].entityId == entityId) {
			return &_mqttAllEntities[i];
		}
	}
	return NULL;
}

void
readEntity(mqttState *singleEntity, char *value, int podNum)
{
	strcpy(value, "Unknown");

	switch (singleEntity->entityId) {
	case mqttEntityId::entitySystemMode:
	case mqttEntityId::entityPodMode:
		switch (pods[podNum].mode) {
		case modeUp:
			strcpy(value, "Up");
			break;
		case modeDown:
			strcpy(value, "Down");
			break;
		case modeOff:
			strcpy(value, "Off");
			break;
		}
		break;
	case mqttEntityId::entityPodAction:
		switch (pods[podNum].action) {
		case actionStop:
			strcpy(value, "Stop");
			break;
		case actionRaise:
			strcpy(value, "Raise");
			break;
		case actionLower:
			strcpy(value, "Lower");
			break;
		}
		break;
	case mqttEntityId::entityPodPosition:
		switch (pods[podNum].position) {
		case positionUp:
			strcpy(value, "Up");
			break;
		case positionAlmostUp:
			strcpy(value, "Almost Up");
			break;
		case positionMiddle:
			strcpy(value, "Middle");
			break;
		case positionAlmostDown:
			strcpy(value, "Almost Down");
			break;
		case positionDown:
			strcpy(value, "Down");
			break;
		}
		break;
	case mqttEntityId::entityPodBatPct:
		sprintf(value, "%d", pods[podNum].batteryPct);
		break;
	case mqttEntityId::entityPodBatVlt:
		sprintf(value, "%0.02f", pods[podNum].batteryVolts);
		break;
	case mqttEntityId::entityPodTopSensor:
		sprintf(value, "%s", pods[podNum].topSensorWet ? "Wet" : "Dry");
		break;
	case mqttEntityId::entityPodBotSensor:
		sprintf(value, "%s", pods[podNum].botSensorWet ? "Wet" : "Dry");
		break;
#ifdef DEBUG_CALLBACKS
	case mqttEntityId::entityCallbacks:
		sprintf(value, "%lu", receivedCallbacks);
		break;
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_FREEMEM
	case mqttEntityId::entityFreemem:
		sprintf(value, "%lu", freeMemory());
		break;
#endif // DEBUG_FREEMEM
#ifdef DEBUG_UPTIME
	case mqttEntityId::entityUptime:
		sprintf(value, "%lu", getUptimeSeconds());
		break;
#endif // DEBUG_UPTIME
	case mqttEntityId::entityVersion:
		sprintf(value, "%s", pods[podNum].version);
		break;
#ifdef DEBUG_WIFI_DAVE_HACK
	case mqttEntityId::entityRSSI:
		sprintf(value, "%d", WiFi.RSSI());
		break;
	case mqttEntityId::entityBSSID:
		{
			byte *bssid = WiFi.BSSID();
			sprintf(value, "%02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		}
		break;
	case mqttEntityId::entityTxPower:
		sprintf(value, "%0.1f", (float)WiFi.getTxPower() / 4.0);
		break;
	case mqttEntityId::entityWifiRecon:
		sprintf(value, "%lu", wifiReconnects);
		break;
#endif // DEBUG_WIFI
	}

	if (singleEntity->allPods && !POD_DATA_IS_FRESH(podNum)) {
		strcpy(value, "unavailable");
	}
}

/*
 * addState
 *
 * Query the handled entity in the usual way, and add the cleansed output to the buffer
 */
boolean
addState(mqttState *singleEntity, int podNum)
{
	char response[MAX_FORMATTED_DATA_VALUE_LENGTH];
	boolean result;

	// Read the register(s)/data
	readEntity(singleEntity, &response[0], podNum);

	result = addToPayload(response);

	return result;
}

void
sendStatus(void)
{
	char stateAddition[512] = "";

	emptyPayload();

	snprintf(stateAddition, sizeof(stateAddition), "{ \"systemStatus\": \"online\"");
	for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
		char moreState[32];
		snprintf(moreState, sizeof(moreState), ", \"pod%dStatus\": \"%s\"", podNum,
			 POD_DATA_IS_FRESH(podNum) ? "online" : "offline");
		strlcat(stateAddition, moreState, sizeof(stateAddition));
	}
	strlcat(stateAddition, " }", sizeof(stateAddition));
	if (!addToPayload(stateAddition)) {
		return;
	}

	sendMqtt(statusTopic, MQTT_RETAIN);
}

boolean
addConfig(mqttState *singleEntity, int podNum)
{
	char stateAddition[1024] = "";
	char prettyName[MAX_MQTT_NAME_LENGTH + 8], mqttName[MAX_MQTT_NAME_LENGTH + 8];

	if (singleEntity->allPods) {
		snprintf(mqttName, sizeof(mqttName), "Pod%d_%s", podNum, singleEntity->mqttName);
	} else {
		strlcpy(mqttName, singleEntity->mqttName, sizeof(mqttName));
	}

	sprintf(stateAddition, "{");
	if (!addToPayload(stateAddition)) {
		return false;
	}

	switch (singleEntity->haClass) {
	case homeAssistantClass::haClassBox:
//	case homeAssistantClass::haClassNumber:
		sprintf(stateAddition, "\"component\": \"number\"");
		break;
	case homeAssistantClass::haClassSelect:
		sprintf(stateAddition, "\"component\": \"select\"");
		break;
	case homeAssistantClass::haClassMoisture:
		sprintf(stateAddition, "\"component\": \"binary_sensor\"");
		break;
	default:
		sprintf(stateAddition, "\"component\": \"sensor\"");
		break;
	}
	if (!addToPayload(stateAddition)) {
		return false;
	}

	snprintf(stateAddition, sizeof(stateAddition),
		 ", \"device\": {"
		 " \"name\": \"%s\", \"model\": \"V-Lift\", \"manufacturer\": \"Sunstream\","
		 " \"identifiers\": [\"%s\"]}",
		 myUniqueId, myUniqueId);
	if (!addToPayload(stateAddition)) {
		return false;
	}

	strlcpy(prettyName, mqttName, sizeof(prettyName));
	while(char *ch = strchr(prettyName, '_')) {
		*ch = ' ';
	}
	snprintf(stateAddition, sizeof(stateAddition), ", \"name\": \"%s\"", prettyName);
	if (!addToPayload(stateAddition)) {
		return false;
	}

	snprintf(stateAddition, sizeof(stateAddition), ", \"unique_id\": \"%s_%s\"", myUniqueId, mqttName);
	if (!addToPayload(stateAddition)) {
		return false;
	}

	switch (singleEntity->haClass) {
	case homeAssistantClass::haClassMoisture:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"Moisture\""
			 ", \"payload_on\": \"Wet\""
			 ", \"payload_off\": \"Dry\""
#ifdef MQTT_FORCE_UPDATE
			 ", \"force_update\": \"true\""
#endif // MQTT_FORCE_UPDATE
			 ", \"entity_category\": \"diagnostic\"");
		break;
	case homeAssistantClass::haClassBattery:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"battery\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"%%\""
#ifdef MQTT_FORCE_UPDATE
			 ", \"force_update\": \"true\""
#endif // MQTT_FORCE_UPDATE
			);
		break;
	case homeAssistantClass::haClassVoltage:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"voltage\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"V\""
#ifdef MQTT_FORCE_UPDATE
			 ", \"force_update\": \"true\""
#endif // MQTT_FORCE_UPDATE
			);
		break;
	case homeAssistantClass::haClassDuration:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"duration\""
			 ", \"state_class\": \"measurement\""
			 ", \"unit_of_measurement\": \"s\""
			 ", \"entity_category\": \"diagnostic\"");
		break;
	case homeAssistantClass::haClassBox:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"mode\": \"box\"");
		break;
	case homeAssistantClass::haClassInfo:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"entity_category\": \"diagnostic\"");
		break;
	case homeAssistantClass::haClassSelect:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"device_class\": \"enum\""
			);
		break;
//	case homeAssistantClass::haClassNumber:
//		snprintf(stateAddition, sizeof(stateAddition),
//			 ", \"entity_category\": \"diagnostic\""
//			 ", \"entity_type\": \"number\"");
//		break;
	default:
		strcpy(stateAddition, "");
		break;
	}
	if (strlen(stateAddition) != 0) {
		if (!addToPayload(stateAddition)) {
			return false;
		}
	}

	switch (singleEntity->entityId) {
	case mqttEntityId::entitySystemMode:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"icon\": \"mdi:swap-vertical-bold\""
			 ", \"options\": [ \"Up\", \"Down\", \"Off\" ]");
		break;
	case mqttEntityId::entityPodMode:
	case mqttEntityId::entityPodAction:
	case mqttEntityId::entityPodPosition:
		sprintf(stateAddition, ", \"icon\": \"mdi:swap-vertical-bold\"");
		break;
	case mqttEntityId::entityPodBatVlt:
		sprintf(stateAddition,
			", \"icon\": \"mdi:battery-heart\""
			", \"suggested_display_precision\": 2");
		break;
#ifdef DEBUG_WIFI_DAVE_HACK
	case mqttEntityId::entityRSSI:
	case mqttEntityId::entityBSSID:
	case mqttEntityId::entityTxPower:
	case mqttEntityId::entityWifiRecon:
		sprintf(stateAddition, ", \"icon\": \"mdi:wifi\"");
		break;
#endif // DEBUG_WIFI
	case mqttEntityId::entityVersion:
		sprintf(stateAddition, ", \"icon\": \"mdi:numeric\"");
		break;
#ifdef DEBUG_FREEMEM
	case mqttEntityId::entityFreemem:
		sprintf(stateAddition, ", \"icon\": \"mdi:memory\"");
		break;
#endif // DEBUG_FREEMEM
#ifdef DEBUG_CALLBACKS
	case mqttEntityId::entityCallbacks:
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_UPTIME
	case mqttEntityId::entityUptime:
#endif // DEBUG_UPTIME
	case mqttEntityId::entityPodBatPct:
	case mqttEntityId::entityPodTopSensor:
	case mqttEntityId::entityPodBotSensor:
		// Use default icon
		strcpy(stateAddition, "");
		break;
	}
	if (strlen(stateAddition) != 0) {
		if (!addToPayload(stateAddition)) {
			return false;
		}
	}

	if (singleEntity->subscribe) {
		sprintf(stateAddition, ", \"qos\": %d", MQTT_SUBSCRIBE_QOS);
		if (!addToPayload(stateAddition)) {
			return false;
		}
	}

	switch (singleEntity->entityId) {
	default:
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"state_topic\": \"" DEVICE_NAME "/%s/%s/state\"",
			 myUniqueId, mqttName);
		break;
	}
	if (!addToPayload(stateAddition)) {
		return false;
	}

	if (singleEntity->subscribe) {
		sprintf(stateAddition, ", \"command_topic\": \"" DEVICE_NAME "/%s/%s/command\"",
			myUniqueId, mqttName);
		if (!addToPayload(stateAddition)) {
			return false;
		}
	}

	if (singleEntity->allPods) {
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"availability_template\": \"{{ \\\"online\\\" if value_json.pod%dStatus == \\\"online\\\" else \\\"offline\\\" }}\""
			 ", \"availability_topic\": \"%s\"", podNum, statusTopic);
	} else {
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"availability_template\": \"{{ value_json.systemStatus | default(\\\"\\\") }}\""
			 ", \"availability_topic\": \"%s\"", statusTopic);
	}
	if (!addToPayload(stateAddition)) {
		return false;
	}

	strcpy(stateAddition, "}");
	if (!addToPayload(stateAddition)) {
		return false;
	}

	return true;
}


boolean
addToPayload(const char* addition)
{
	int targetRequestedSize = strlen(_mqttPayload) + strlen(addition);

	if (targetRequestedSize > (MAX_MQTT_PAYLOAD_SIZE - 1)) {
		snprintf(_mqttPayload, MAX_MQTT_PAYLOAD_SIZE, "{\r\n    \"mqttError\": \"Length of payload exceeds %d bytes.  Length would be %d bytes.\"\r\n}",
			 (MAX_MQTT_PAYLOAD_SIZE - 1), targetRequestedSize);
		return false;
	} else {
		strlcat(_mqttPayload, addition, MAX_MQTT_PAYLOAD_SIZE);
		return true;
	}
}


void
sendData()
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);

	for (int i = 0; i < numberOfEntities; i++) {
		sendDataFromMqttState(&_mqttAllEntities[i], false);
	}
}

void
sendHaData()
{
	int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);

	for (int i = 0; i < numberOfEntities; i++) {
		sendDataFromMqttState(&_mqttAllEntities[i], true);
	}
}

void
sendDataFromMqttState(mqttState *singleEntity, bool doHomeAssistant)
{
	char topic[256];
	boolean result;
	int loopStart, loopLast;

	if (singleEntity == NULL)
		return;

	loopStart = singleEntity->allPods ? 1 : 0;
	loopLast =  singleEntity->allPods ? NUM_PODS : 0;
	for (int podNum = loopStart; podNum <= loopLast; podNum++) {
		emptyPayload();

		if (doHomeAssistant) {
			const char *entityType;
			switch (singleEntity->haClass) {
			case homeAssistantClass::haClassBox:
//			case homeAssistantClass::haClassNumber:
				entityType = "number";
				break;
			case homeAssistantClass::haClassSelect:
				entityType = "select";
			break;
			case homeAssistantClass::haClassMoisture:
				entityType = "binary_sensor";
				break;
			default:
				entityType = "sensor";
				break;
			}

			if (singleEntity->allPods) {
				snprintf(topic, sizeof(topic), "homeassistant/%s/%s/Pod%d_%s/config", entityType, myUniqueId, podNum, singleEntity->mqttName);
			} else {
				snprintf(topic, sizeof(topic), "homeassistant/%s/%s/%s/config", entityType, myUniqueId, singleEntity->mqttName);
			}
			result = addConfig(singleEntity, podNum);
		} else {
			if (singleEntity->allPods) {
				snprintf(topic, sizeof(topic), DEVICE_NAME "/%s/Pod%d_%s/state", myUniqueId, podNum, singleEntity->mqttName);
			} else {
				snprintf(topic, sizeof(topic), DEVICE_NAME "/%s/%s/state", myUniqueId, singleEntity->mqttName);
			}
			result = addState(singleEntity, podNum);
		}

		if (result) {
			// And send
			sendMqtt(topic, singleEntity->retain ? MQTT_RETAIN : false);
		}
	}
}


/*
 * mqttCallback()
 *
 * This function is executed when an MQTT message arrives on a topic that we are subscribed to.
 */
void mqttCallback(char* topic, byte* message, unsigned int length)
{
	char mqttIncomingPayload[64] = ""; // Should be enough to cover command requests
	mqttState *mqttEntity = NULL;

#ifdef DEBUG_OVER_SERIAL
	Serial.printf("Topic: %s\n", topic);
#endif

#ifdef DEBUG_CALLBACKS
	receivedCallbacks++;
#endif // DEBUG_CALLBACKS

	if ((length == 0) || (length >= sizeof(mqttIncomingPayload))) {
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("mqttCallback: bad length: %d\n", length);
#endif
#ifdef DEBUG_CALLBACKS
		badCallbacks++;
#endif // DEBUG_CALLBACKS
		return; // We won't be doing anything
	} else {
		// Get the payload (ensure NULL termination)
		strlcpy(mqttIncomingPayload, (char *)message, length + 1);
	}
#ifdef DEBUG_OVER_SERIAL
	Serial.printf("Payload: %d\n", length);
	Serial.println(mqttIncomingPayload);
#endif

	// Special case for Home Assistant itself
	if (strcmp(topic, MQTT_SUB_HOMEASSISTANT) == 0) {
		if (strcmp(mqttIncomingPayload, "online") == 0) {
			resendAllData = true;
		} else {
#ifdef DEBUG_OVER_SERIAL
			Serial.println("Unknown homeassistant/status: ");
#endif
		}
		return; // No further processing needed.
	} else {
		// match to DEVICE_NAME "/SERIAL#/MQTT_NAME/command"
		char matchPrefix[64];

		snprintf(matchPrefix, sizeof(matchPrefix), DEVICE_NAME "/%s/", myUniqueId);
		if (!strncmp(topic, matchPrefix, strlen(matchPrefix)) &&
		    !strcmp(&topic[strlen(topic) - strlen("/command")], "/command")) {
			char topicEntityName[64];
			int topicEntityLen = strlen(topic) - strlen(matchPrefix) - strlen("/command");
			if (topicEntityLen < sizeof(topicEntityName)) {
				strlcpy(topicEntityName, &topic[strlen(matchPrefix)], topicEntityLen + 1);
				mqttEntity = lookupSubscription(topicEntityName);
			}
		}
		if (mqttEntity == NULL) {
#ifdef DEBUG_CALLBACKS
			unknownCallbacks++;
#endif // DEBUG_CALLBACKS
			return; // No further processing possible.
		}
	}

	// Update system!!!
	{
//		int32_t singleInt32 = -1;
		char *singleString;
//		char *endPtr = NULL;
		bool valueProcessingError = false;

		// First, process value.
		switch (mqttEntity->entityId) {
		case mqttEntityId::entitySystemMode:
			singleString = mqttIncomingPayload;
			break;
		default:
#ifdef DEBUG_OVER_SERIAL
			Serial.printf("Trying to update an unhandled entity! %d\n", mqttEntity->entityId);
#endif
#ifdef DEBUG_CALLBACKS
			unknownCallbacks++;
#endif // DEBUG_CALLBACKS
			return; // No further processing possible.
		}

		if (valueProcessingError) {
#ifdef DEBUG_OVER_SERIAL
			Serial.printf("Callback for %s with bad value.\n", mqttEntity->mqttName);
#endif
#ifdef DEBUG_CALLBACKS
			badCallbacks++;
#endif // DEBUG_CALLBACKS
		} else {
			// Now set the value and take appropriate action(s)
			switch (mqttEntity->entityId) {
			case mqttEntityId::entitySystemMode:
				if (!strncmp(singleString, "Up", 3)) {
					pods[0].mode = liftModes::modeUp;
					pods[0].forceMode = false;
					pods[0].lastUpdate = millis();
				} else if (!strncmp(singleString, "Down", 5)) {
					pods[0].mode = liftModes::modeDown;
					pods[0].forceMode = false;
					pods[0].lastUpdate = millis();
				} else if (!strncmp(singleString, "Off", 4)) {
					pods[0].mode = liftModes::modeOff;
					pods[0].forceMode = false;
					pods[0].lastUpdate = millis();
#ifdef DEBUG_CALLBACKS
				} else {
					badCallbacks++;
#endif // DEBUG_CALLBACKS
				}
				resendAllData = true;
				break;
			default:
#ifdef DEBUG_OVER_SERIAL
				Serial.printf("Trying to write an unhandled entity! %d\n", mqttEntity->entityId);
#endif
				break;
			}
		}
	}

	// Don't send updated state here.  We send all state regularly.
	return;
}


/*
 * sendMqtt
 *
 * Sends whatever is in the modular level payload to the specified topic.
 */
void sendMqtt(const char *topic, bool retain)
{
	// Attempt a send
	if (!_mqtt.publish(topic, _mqttPayload, retain)) {
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("MQTT publish failed to %s\n", topic);
		Serial.println(_mqttPayload);
#endif
	} else {
#ifdef DEBUG_OVER_SERIAL
		//Serial.printf("MQTT publish success\n");
#endif
	}

	// Empty payload for next use.
	emptyPayload();
	return;
}

/*
 * emptyPayload
 *
 * Clears so we start at beginning.
 */
void emptyPayload()
{
	_mqttPayload[0] = '\0';
}

void
configButtonISR(void)
{
	configButtonPressed = true;
}

#ifdef DEBUG_FREEMEM
uint32_t freeMemory()
{
	return ESP.getFreeHeap();
}
#endif // DEBUG_FREEMEM
