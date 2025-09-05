/*
Name:		V-Lift.ino
Created:	27 August 2025
Author:		David Carrel

*/

#include <bit>
#include <bitset>
#include <cstdint>
#include <iostream>
// Supporting files
#include "Definitions.h"
#include <Arduino.h>
#include <driver/rtc_io.h>
#include <WiFi.h>
#include <NetworkUdp.h>
#include <WebServer.h>
#ifndef MP_XIAO_ESP32C6
#define LED_BUILTIN 2
#endif // ! MP_XIAO_ESP32C6
#include <DNSServer.h>
#include <WiFiManager.h>
#include <Preferences.h>
#include <PubSubClient.h>
#ifdef USE_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif // USE_DISPLAY

#define popcount __builtin_popcount
void setButtonLEDs(int freq = 0);

// Device parameters
char _version[VERSION_STR_LEN] = "v1.05";
char myUniqueId[17];
char statusTopic[128];

int myPodNum = -1;
podState *myPod;
podState pods[NUM_PODS];

// WiFi parameters
WiFiClient _wifi;
wifi_power_t wifiPower = WIFI_POWER_11dBm; // Will bump to max before setting

// MQTT parameters
PubSubClient _mqtt(_wifi);
char* _mqttPayload = NULL;
bool resendAllData = false;

NetworkUDP udp;
#ifdef DEBUG_UDP
unsigned int udpPacketsSent = 0;
unsigned int udpPacketsSentErrors = 0;
unsigned int udpPacketsReceived = 0;
#endif // DEBUG_UDP

#ifdef USE_BUTTON_INTERRUPTS
volatile boolean configButtonPressed = false;
volatile boolean upButtonChanged = false;
volatile boolean downButtonChanged = false;
#endif // USE_BUTTON_INTERRUPTS

#ifdef USE_DISPLAY
// OLED variables
char _oledOperatingIndicator = '*';
char _oledLine2[OLED_CHARACTER_WIDTH] = "";
char _oledLine3[OLED_CHARACTER_WIDTH] = "";
char _oledLine4[OLED_CHARACTER_WIDTH] = "";
#endif // USE_DISPLAY

// Config handling
Config config;

#ifdef DEBUG_OVER_SERIAL
// Fixed char array for messages to the serial port
char _debugOutput[128];
#endif // DEBUG_OVER_SERIAL

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
#ifdef DEBUG_WIFI
	{ mqttEntityId::entityRSSI,               "RSSI",              false, false, false, homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityBSSID,              "BSSID",             false, false, false, homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityTxPower,            "TX_Power",          false, false, false, homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityWifiRecon,          "reconnects",        false, false, false, homeAssistantClass::haClassInfo },
#endif // DEBUG_WIFI
#ifdef DEBUG_UPTIME
	{ mqttEntityId::entityUptime,             "Uptime",            false, false, false, homeAssistantClass::haClassDuration },
#endif // DEBUG_UPTIME
	{ mqttEntityId::entityVersion,            "Version",           true,  false, true,  homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityBatPct,             "Battery",           true,  false, true,  homeAssistantClass::haClassBattery },
	{ mqttEntityId::entityBatVlt,             "Battery_Voltage",   false, false, true,  homeAssistantClass::haClassVoltage },
	{ mqttEntityId::entityTopSensor,          "Top_Sensor",        true,  false, true,  homeAssistantClass::haClassMoisture },
	{ mqttEntityId::entityBotSensor,          "Bottom_Sensor",     true,  false, true,  homeAssistantClass::haClassMoisture }
};

// These timers are used in the main loop.
#define RUNSTATE_INTERVAL 2000
#define STATUS_INTERVAL_ONE_SECOND 1000
#define STATUS_INTERVAL_FIVE_SECONDS 5000
#define STATUS_INTERVAL_TEN_SECONDS 10000
#define STATUS_INTERVAL_THIRTY_SECONDS 30000
#define STATUS_INTERVAL_ONE_MINUTE 60000
#define STATUS_INTERVAL_FIVE_MINUTE 300000
#define STATUS_INTERVAL_ONE_HOUR 3600000
#define STATUS_INTERVAL_ONE_DAY 86400000
#define UPDATE_STATUS_BAR_INTERVAL 500
#define WIFI_RECONNECT_INTERVAL 500  // 1/2 second
#define STATUS_INTERVAL STATUS_INTERVAL_TEN_SECONDS
#define DATA_INTERVAL STATUS_INTERVAL_THIRTY_SECONDS
#define POD2POD_DATA_INTERVAL STATUS_INTERVAL_FIVE_SECONDS

#define POD_DATA_IS_FRESH(_podNum) ((pods[_podNum - 1].lastUpdate != 0) && (millis() - pods[_podNum - 1].lastUpdate) < (4 * DATA_INTERVAL))

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

#ifdef DEBUG_OVER_SERIAL
	Serial.begin(9600);
#endif // DEBUG_OVER_SERIAL

	pinMode(LED_BUILTIN, OUTPUT);		// Configure LED for output
	pinMode(CONFIG_BUTTON_PIN, INPUT);	// Configure the user push button
#ifdef USE_BUTTON_INTERRUPTS
	attachInterrupt(CONFIG_BUTTON_PIN, configButtonISR, FALLING);
#endif // USE_BUTTON_INTERRUPTS

	// Wire.setClock(10000);

#ifdef USE_DISPLAY
	// Display time
	_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  // initialize OLED
	_display.clearDisplay();
	_display.display();
	updateOLED(false, myUniqueId, "Starting...", _version);
#endif // USE_DISPLAY

#ifdef DEBUG_OVER_SERIAL
	sprintf(_debugOutput, "Starting %s", myUniqueId);
	Serial.println(_debugOutput);
	delay(500);
#endif

	for (int podIdx = 0; podIdx < NUM_PODS; podIdx++) {
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
#ifdef DAVE_FORCE_POD_NUM
		myPodNum = DAVE_FORCE_POD_NUM;
#else // DAVE_FORCE_POD_NUM
		myPodNum = getPodNumFromButton();
#endif // DAVE_FORCE_POD_NUM
		myPod = &pods[myPodNum - 1];
		preferences.begin(DEVICE_NAME, false); // RW
		preferences.putInt(PREF_NAME_POD_NUM, myPodNum);
		preferences.end();
	} else {
		myPodNum = config.podNumber;
		myPod = &pods[config.podNumber - 1];
	}

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
#ifdef USE_DISPLAY
	{
		char line3[OLED_CHARACTER_WIDTH];
		snprintf(line3, sizeof(line3), "Pod # %d", myPodNum);
		updateOLED(false, "Config is set", line3, _version);
		delay(2000); // DAVE
	}
#endif // USE_DISPLAY

	pinMode(BAT_READ_PIN, INPUT);		// Configure pin for battery voltage
	pinMode(TOP_SENSOR_PIN, INPUT);
	pinMode(BOT_SENSOR_PIN, INPUT);
	pinMode(UP_RELAY_PIN, OUTPUT);		// Configure pin for controlling relay
	pinMode(DOWN_RELAY_PIN, OUTPUT);	// Configure pin for controlling relay

	if (myPodNum == 1) {
		pinMode(UP_BUTTON_PIN, INPUT_PULLUP);
		pinMode(DOWN_BUTTON_PIN, INPUT_PULLUP);
#ifdef USE_BUTTON_INTERRUPTS
		attachInterrupt(UP_BUTTON_PIN, upButtonISR, CHANGE);
		attachInterrupt(DOWN_BUTTON_PIN, downButtonISR, CHANGE);
#endif // USE_BUTTON_INTERRUPTS
		pinMode(UP_LED_PIN, OUTPUT);
		pinMode(DOWN_LED_PIN, OUTPUT);
#ifdef MP_XIAO_ESP32C6
		pinMode(WIFI_ENABLE, OUTPUT);
		digitalWrite(WIFI_ENABLE, LOW);
		pinMode(WIFI_ANT_CONFIG, OUTPUT);
		digitalWrite(WIFI_ANT_CONFIG, config.extAntenna ? HIGH : LOW);
#endif // MP_XIAO_ESP32C6
	}

	// Configure WIFI
	checkWifiStatus(true);

	if (myPodNum == 1) {
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
	int podNum = 1;

	for (;;) {
		updateOLED(false, "Press button", "to start", "setting pod num.");
#ifdef USE_BUTTON_INTERRUPTS
		if (configButtonPressed) {
			configButtonPressed = false;
			break;
		}
#else // USE_BUTTON_INTERRUPTS
#error Not handled
#endif // USE_BUTTON_INTERRUPTS
		delay(500);
	}
#define POD_NUM_SEC 15
	for (int sec = POD_NUM_SEC; sec > 0; sec--) {
		char line3[OLED_CHARACTER_WIDTH], line4[OLED_CHARACTER_WIDTH];
#ifdef USE_BUTTON_INTERRUPTS
		if (configButtonPressed) {
			configButtonPressed = false;
			podNum++;
			if (podNum > NUM_PODS) {
				podNum = 1;
			}
			sec = POD_NUM_SEC;
		}
#else // USE_BUTTON_INTERRUPTS
#error Not handled
#endif // USE_BUTTON_INTERRUPTS
		snprintf(line3, sizeof(line3), "        %d", podNum);
		snprintf(line4, sizeof(line4), "Saving in %d seconds", sec);
		updateOLED(false, "** Set Pod number **", line3, line4);
		delay(1000);
	}
	return podNum;
}

void
configLoop (void)
{
#ifdef USE_DISPLAY
	bool flip = false;
#endif // USE_DISPLAY

#ifdef DEBUG_OVER_SERIAL
	Serial.println("Configuration is not set.");
#endif

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
		flashBuiltinLed(250);
		if (myPodNum == 1) {
			setButtonLEDs(250);
		}

		// Read button state
#ifdef USE_BUTTON_INTERRUPTS
		if (configButtonPressed) {
			configButtonPressed = false;
			break;
		}
#else // USE_BUTTON_INTERRUPTS
		if (digitalRead(CONFIG_BUTTON_PIN) == LOW) {
			break;
		}
#endif // USE_BUTTON_INTERRUPTS

		delay(30);
	}

	configHandler();
}

void
configHandler(void)
{
	Preferences preferences;
	WiFiManager wifiManager;

	wifiManager.setBreakAfterConfig(true);
	wifiManager.setTitle(DEVICE_NAME);
	wifiManager.setShowInfoUpdate(false);
	WiFiManagerParameter p_lineBreak_text("<p>MQTT settings:</p>");
	WiFiManagerParameter custom_mqtt_server("server", "MQTT server", "", 40);
	WiFiManagerParameter custom_mqtt_port("port", "MQTT port", "1883", 6);
	WiFiManagerParameter custom_mqtt_user("user", "MQTT user", "", 32);
	WiFiManagerParameter custom_mqtt_pass("mpass", "MQTT password", "", 32);
#ifdef MP_XIAO_ESP32C6
	const char _customHtml_checkbox[] = "type=\"checkbox\"";
	WiFiManagerParameter custom_ext_ant("ext_antenna", "Use external WiFi antenna\n", "T", 2, _customHtml_checkbox, WFM_LABEL_AFTER);
#endif // MP_XIAO_ESP32C6

	WiFi.disconnect(true, true); // Disconnect and erase saved WiFi config
#ifdef USE_DISPLAY
	updateOLED(false, "Web", "config", "active");
#endif // USE_DISPLAY

#ifdef MP_XIAO_ESP32C6
	wifiManager.addParameter(&custom_ext_ant);
#endif // MP_XIAO_ESP32C6
	wifiManager.addParameter(&p_lineBreak_text);
	wifiManager.addParameter(&custom_mqtt_server);
	wifiManager.addParameter(&custom_mqtt_port);
	wifiManager.addParameter(&custom_mqtt_user);
	wifiManager.addParameter(&custom_mqtt_pass);

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
	boolean wifiIsOn = false;
	boolean mqttIsOn = false;

	// Read button state
#ifdef USE_BUTTON_INTERRUPTS
	if (configButtonPressed) {
		Preferences preferences;
		int podNum;
		configButtonPressed = false;
		podNum = getPodNumFromButton();
		myPodNum = podNum;
		preferences.begin(DEVICE_NAME, false); // RW
		preferences.putInt(PREF_NAME_POD_NUM, podNum);
		preferences.end();
		if (podNum == 1) {
			configHandler();
		}
		ESP.restart();
	}
#else // USE_BUTTON_INTERRUPTS
#error Not handled
#endif // USE_BUTTON_INTERRUPTS

	// Make sure WiFi is good
	wifiIsOn = checkWifiStatus(false);

	if (myPodNum == 1) {
		// make sure mqtt is still connected
		if (wifiIsOn) {
			mqttIsOn = _mqtt.connected();
			if (!mqttIsOn || !_mqtt.loop()) {
				mqttIsOn = setupMqtt();
			}
		}
	}

	// Flash board LED (2 seconds)
	flashBuiltinLed(2000);

#ifdef USE_DISPLAY
	// Refresh LED Screen, will cause the status asterisk to flicker
	updateOLED(true, "", "", "");
#endif // USE_DISPLAY

	// Update the pod state
        readBattery();
        readPodState();        // reads/sets sensors, and sets myPod->position
	// MQTT/Zigbee might have asynchronously updated myPod->mode
	if (myPodNum == 1) {
		readModeFromButtons();  // Set myPod->mode based on buttons and current action
#ifdef USE_ZIGBEE
		readModeFromRemote();  // Set myPod->mode based on zigbee remote and current action
#endif // USE_ZIGBEE
	}
	setPodAction();	// Set myPod->action and activate relays
	myPod->lastUpdate = millis();

	if (resendAllData) {
		// Read and transmit all entity & HA data to MQTT
		lastRunSendData = 0;
		resendAllData = false;
	}

	if (myPodNum == 1) {
		setButtonLEDs();

		if (wifiIsOn) {
			getRemotePodStatus();
		}

		// Send status/keepalive
// DAVE - adjust frequencies

		if (mqttIsOn && checkTimer(&lastRunStatus, STATUS_INTERVAL)) {
			sendStatus();
		}

		if (mqttIsOn && checkTimer(&lastRunSendData, DATA_INTERVAL)) {
// DAVE - do I check received zigbee data here??
			sendHaData();
			sendData();
		}

		if (wifiIsOn && checkTimer(&lastRunSendData, POD2POD_DATA_INTERVAL)) {
// DAVE			sendCommandsToRemotePods();
		}
	} else {
		if (wifiIsOn && checkTimer(&lastRunSendData, POD2POD_DATA_INTERVAL)) {
			sendPodInfoToNumberOne();
		}

// DAVE		getCommandsFromNumberOne();
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
		return -1;
	}
	valueStr += strlen(key);

	return *valueStr == '1';
}

#define UDP_BUF_SIZ 128
void
getRemotePodStatus (void)
{
	char buf[UDP_BUF_SIZ];
	int packetSize = udp.parsePacket();

	while (packetSize) {
#ifdef DEBUG_OVER_SERIAL
		Serial.print("Received packet of size ");
		Serial.println(packetSize);
		Serial.print("From ");
		IPAddress remoteIp = udp.remoteIP();
		Serial.print(remoteIp);
		Serial.print(", port ");
		Serial.println(udp.remotePort());
#endif // DEBUG_OVER_SERIAL

		// read the packet into buffer
		int podNum, podIdx, len = udp.read(buf, sizeof(buf));
		if (len > 0) {
			buf[len] = 0;
#ifdef DEBUG_OVER_SERIAL
			Serial.print("Data is: ");
			Serial.println(buf);
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_UDP
			udpPacketsReceived++;
#endif // DEBUG_UDP
			podNum = parseInt(buf, "PN=");
			if (podNum > 1 && podNum <= NUM_PODS) {
				podIdx = podNum - 1;
				pods[podIdx].batteryPct = parseInt(buf, "BP=");
				pods[podIdx].topSensor = parseBool(buf, "TS=");
				pods[podIdx].botSensor = parseBool(buf, "BS=");
				parseStr(buf, "VV=", pods[podIdx].version, sizeof(pods[podIdx].version));
				pods[podIdx].lastUpdate = millis();
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
	udp.printf("PN=%1d,BP=%03d,TS=%c,BS=%c,VV=%s", myPodNum, myPod->batteryPct, myPod->topSensor ? '1' : '0', myPod->botSensor ? '1' : '0', myPod->version);
#ifndef DEBUG_UDP
	udp.endPacket();
#else // ! DEBUG_UDP
	int err = udp.endPacket();
	if (err == 0) {
		udpPacketsSentErrors++;
	}
	udpPacketsSent++;
#endif // ! DEBUG_UDP
}

// freq defaults to 0
void
setButtonLEDs (int freq)
{
	static unsigned long lastLed = 0;
	static boolean toggle = false;

#define LED_ON  HIGH
#define LED_OFF LOW

	if (freq) {
		// if INIT, then fast alternate
		if (checkTimer(&lastLed, freq)) {
			digitalWrite(UP_LED_PIN, toggle ? LED_ON : LED_OFF);
			digitalWrite(DOWN_LED_PIN, toggle ? LED_OFF : LED_ON);
			toggle = !toggle;
		}
	} else if (myPod->mode == modeOff) {
		// if mode == off, then slow flash both
		if (checkTimer(&lastLed, 2000)) {
			digitalWrite(UP_LED_PIN, toggle ? LED_ON : LED_OFF);
			digitalWrite(DOWN_LED_PIN, toggle ? LED_ON : LED_OFF);
			toggle = !toggle;
		}
	} else if (myPod->action == actionRaise) {
		// if action raise, then turn on UP
		digitalWrite(UP_LED_PIN, LED_ON);
		digitalWrite(DOWN_LED_PIN, LED_OFF);
	} else if (myPod->action == actionLower) {
		// if action lower, then turn on DOWN
		digitalWrite(UP_LED_PIN, LED_OFF);
		digitalWrite(DOWN_LED_PIN, LED_ON);
	} else {
		// turn off
		digitalWrite(UP_LED_PIN, LED_OFF);
		digitalWrite(DOWN_LED_PIN, LED_OFF);
	}
}

void
setPodAction (void)
{
	// Set pod relay actions
	if (myPod->mode == modeOff) {
		myPod->action = actionStop;
	} else {
		switch (myPod->position) {
		case positionUp:
			myPod->action = (myPod->mode == modeUp) ? actionStop : actionLower;
			break;
		case positionAlmostUp:
		case positionMiddle:
		case positionAlmostDown:
			myPod->action = (myPod->mode == modeUp) ? actionRaise : actionLower;
			break;
		case positionDown:
			myPod->action = (myPod->mode == modeUp) ? actionRaise : actionStop;
			break;
		default:  // Shouldn't happen.
			myPod->action = actionStop;
			break;
		}
	}

#define RELAY_ON  HIGH
#define RELAY_OFF LOW

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
// DAVE - Can this be interrupt driven?
void
readModeFromButtons(void)
{
	boolean upPress = false, downPress = false;
	boolean upLongPress = false, downLongPress = false;
	static uint32_t upPressLastTime = 0, downPressLastTime = 0;

#define LONG_PRESS_TIME 3000    // 3 seconds

#ifdef USE_BUTTON_INTERRUPTS
	if (upButtonChanged) {
		noInterrupts();   // Atomically read and reset the flag
		boolean pressed = (digitalRead(UP_BUTTON_PIN) == LOW);
		boolean changed = upButtonChanged;
		upButtonChanged = false;
		interrupts();

		if (changed) {
			if (!pressed) { // Button released
				unsigned long pressDuration = millis() - upPressLastTime;
				if (upPressLastTime && (pressDuration >= LONG_PRESS_TIME)) {
					upLongPress = true;
				} else {
					upPress = true;
				}
				upPressLastTime = 0;
			} else { // Button pressed
				upPressLastTime = millis();
			}
		}
	}
	if (downButtonChanged) {
		noInterrupts();   // Atomically read and reset the flag
		boolean pressed = (digitalRead(DOWN_BUTTON_PIN) == LOW);
		boolean changed = downButtonChanged;
		downButtonChanged = false;
		interrupts();

		if (changed) {
			if (!pressed) { // Button released
				unsigned long pressDuration = millis() - downPressLastTime;
				if (downPressLastTime && (pressDuration >= LONG_PRESS_TIME)) {
					downLongPress = true;
				} else {
					downPress = true;
				}
				downPressLastTime = 0;
			} else { // Button pressed
				downPressLastTime = millis();
			}
		}
	}
#else // USE_BUTTON_INTERRUPTS
	if (digitalRead(UP_BUTTON_PIN) == LOW) {
		upPress = true;
		uint32_t now = millis();
		if (upPressLastTime) {
			if ((upPressLastTime + LONG_PRESS_TIME) > now) {
				upLongPress = true;
			}
		} else {
			upPressLastTime = now;
		}
	} else {
		upPressLastTime = 0;
	}

	if (digitalRead(DOWN_BUTTON_PIN) == LOW) {
		downPress = true;
		uint32_t now = millis();
		if (downPressLastTime) {
			if ((downPressLastTime + LONG_PRESS_TIME) > now) {
				downLongPress = true;
			}
		} else {
			downPressLastTime = now;
		}
	} else {
		downPressLastTime = 0;
	}
#endif // USE_BUTTON_INTERRUPTS

	if (upPress) {
		if (myPod->action == actionRaise) {
			myPod->mode = modeOff;
		} else {
			myPod->mode = modeUp;
		}
	} else if (downPress) {
		if (myPod->action == actionLower) {
			myPod->mode = modeOff;
		} else {
			myPod->mode = modeDown;
		}
	}
}

// Read pod water sensors and determine position
// Sets topSensor, bottomSensor, and position
void
readPodState(void)
{
	boolean top = false, bottom = false;
	int bottomCount = 0;
	uint32_t now;
	static uint32_t almostDownTime = 0, almostUpTime = 0;

#define SENSOR_NUM_SAMPLES 16
	for (int i = 0; i < SENSOR_NUM_SAMPLES; i++) {
		if (digitalRead(TOP_SENSOR_PIN) == HIGH) {
			// If ANY sample is true, set it as "wet"
			top = true;
		}
		if (digitalRead(BOT_SENSOR_PIN) == HIGH) {
			// If ANY sample is true, set it as "wet"
			bottomCount++;
		}
	}
	if (bottomCount > (SENSOR_NUM_SAMPLES / 2)) {
		// If half the readings are wet, then we are wet.
		bottom = true;
	}

	myPod->topSensor = top;
	myPod->botSensor = bottom;

#define ALMOST_DOWN_DELAY 2000  // 2 seconds
#define ALMOST_UP_DELAY 5000    // 5  seconds
	now = millis();
	if (top) {
		if (myPod->position == positionMiddle) {
			myPod->position = positionAlmostDown;
			almostDownTime = now;
		} else if ((myPod->position == positionAlmostDown) &&
			   ((almostDownTime + ALMOST_DOWN_DELAY) > now)) {
			// Do nothing until delay passes
		} else {
			myPod->position = positionDown;
			almostDownTime = 0;
		}
		almostUpTime = 0;
	} else if (bottom) {
		myPod->position = positionMiddle;
		almostDownTime = 0;
		almostUpTime = 0;
	} else {
		if (myPod->position == positionMiddle) {
			myPod->position = positionAlmostUp;
			almostUpTime = now;
		} else if ((myPod->position == positionAlmostUp) &&
			   ((almostUpTime + ALMOST_UP_DELAY) > now)) {
			// Do nothing until delay passes
		} else {
			myPod->position = positionUp;
			almostUpTime = 0;
		}
		almostDownTime = 0;
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
setupWifi(bool initialConnect, unsigned int tries)
{
#ifdef DEBUG_OVER_SERIAL
	if (initialConnect) {
		sprintf(_debugOutput, "Connecting to %s", config.wifiSSID.c_str());
		Serial.println(_debugOutput);
	} else {
		sprintf(_debugOutput, "Reconnect to %s", config.wifiSSID.c_str());
		Serial.println(_debugOutput);
	}
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_WIFI
	if (!initialConnect && (tries == 0)) {
		wifiReconnects++;
	}
#endif // DEBUG_WIFI

	if (tries == 5000) {
		ESP.restart();
	}

	if (tries % 50 == 0) {
		IPAddress privIP(192, 168, PRIV_WIFI_SUBNET, myPodNum); // Desired IP for the ESP32 AP
		IPAddress privGateway(192, 168, PRIV_WIFI_SUBNET, 1); // Gateway IP (often the same as local_IP for AP)
		IPAddress privSubnet(255, 255, 255, 0); // Subnet mask

		WiFi.disconnect();
		WiFi.softAPdisconnect();
		WiFi.mode(WIFI_MODE_NULL);

		if (myPodNum == 1) {
			// Set up in AP & Station Mode
			WiFi.mode(WIFI_AP_STA);
			WiFi.hostname(myUniqueId);
			WiFi.softAP(PRIV_WIFI_SSID, PRIV_WIFI_PASS);
			WiFi.softAPConfig(privIP, privGateway, privSubnet);
			WiFi.softAP(PRIV_WIFI_SSID, PRIV_WIFI_PASS, 1, 0, NUM_PODS - 1);
			// Helps when multiple APs for our SSID
			WiFi.setSortMethod(WIFI_CONNECT_AP_BY_SIGNAL);
			WiFi.setScanMethod(WIFI_ALL_CHANNEL_SCAN);
			WiFi.begin(config.wifiSSID.c_str(), config.wifiPass.c_str());
		} else {
			// Set up in Station Mode - Will be connecting to an access point
			WiFi.mode(WIFI_STA);
			WiFi.hostname(myUniqueId);
			WiFi.config(privIP, privGateway, privGateway, privSubnet);
			WiFi.begin(PRIV_WIFI_SSID, PRIV_WIFI_PASS);
		}

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
	}
}

boolean
checkWifiStatus(boolean initialConnect)
{
	static unsigned long lastWifiTry = 0;
	static unsigned int wifiTries = 0;
	static uint8_t previousWifiStatus = 0;
	uint8_t status;
	boolean ret = false;

        if (initialConnect || (WiFi.status() != WL_CONNECTED)) {
		if (checkTimer(&lastWifiTry, WIFI_RECONNECT_INTERVAL)) {
			setupWifi(initialConnect, wifiTries);
#ifdef DEBUG_OVER_SERIAL
			sprintf(_debugOutput, "WiFi tries = %d", wifiTries);
			Serial.println(_debugOutput);
#endif // DEBUG_OVER_SERIAL
			wifiTries++;
		}
	}

	status = WiFi.status();
        if (status == WL_CONNECTED) {
		wifiTries = 0;
		lastWifiTry = 0;
		ret = true;
	}
	if ((status == WL_CONNECTED) && (previousWifiStatus != WL_CONNECTED)) {
		IPAddress myIP(192, 168, PRIV_WIFI_SUBNET, myPodNum);
		udp.begin(myIP, PRIV_UDP_PORT);
		resendAllData = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.print("WiFi connected, IP is ");
		Serial.print(WiFi.localIP());
		Serial.print(" : ");
		Serial.println(WiFi.SSID());
		byte *bssid = WiFi.BSSID();
		sprintf(_debugOutput, "WiFi BSSID is %02X:%02X:%02X:%02X:%02X:%02X", bssid[0], bssid[1], bssid[2], bssid[3], bssid[4], bssid[5]);
		Serial.println(_debugOutput);
		Serial.print("WiFi RSSI: ");
		Serial.println(WiFi.RSSI());
		if (myPodNum == 1) {
			Serial.print("softAP IP address: ");
			Serial.print(WiFi.softAPIP());
			Serial.print(" : ");
			Serial.print(WiFi.softAPSubnetMask());
			Serial.print(" : ");
			Serial.println(WiFi.softAPSSID());
		}
#endif // DEBUG_OVER_SERIAL
	}
	previousWifiStatus = status;

	return ret;
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
		char wifiStatus, mqttStatus, zigbeeStatus;

		wifiStatus = mqttStatus = zigbeeStatus = ' ';
#ifdef USE_ZIGBEE
		if (zigbeeActive) {
			zigbeeStatus = 'Z';
		}
#endif // USE_ZIGBEE
		wifiStatus = WiFi.status() == WL_CONNECTED ? 'W' : ' ';
		mqttStatus = _mqtt.connected() && _mqtt.loop() ? 'M' : ' ';
		snprintf(line1, sizeof(line1), "Lift %c%c%c%c        %3hhd",
			 _oledOperatingIndicator, wifiStatus, mqttStatus, zigbeeStatus, rssi);
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

	// Pod status for line 2
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
		snprintf(line2, sizeof(line2), "%s : %s : %s", mode, action, position);  // 4 + 3 + 5 + 3 + 5
	}

	// Get Pod status info for line 3
	if (myPodNum == 1) {
		int activePeers = 0;
		for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
			if (POD_DATA_IS_FRESH(podNum)) {
				activePeers++;
			}
		}
		snprintf(line3, sizeof(line3), "Pod 1: peers %d/%u", activePeers, WiFi.softAPgetStationNum());
	} else {
		snprintf(line3, sizeof(line3), "Pod %d", myPodNum);
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
		snprintf(line4, sizeof(line4), "WiFi RSSI: %d", WiFi.RSSI());
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
	Serial.print("Attempting MQTT connection...");
#endif

	// Attempt to connect
	if (_mqtt.connect(myUniqueId, config.mqttUser.c_str(), config.mqttPass.c_str(), statusTopic, 0, true, "offline", cleanSession)) {
		int numberOfEntities = sizeof(_mqttAllEntities) / sizeof(struct mqttState);
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Connected MQTT");
#endif

		// Special case for Home Assistant
		sprintf(subscriptionDef, "%s", MQTT_SUB_HOMEASSISTANT);
		subscribed = _mqtt.subscribe(subscriptionDef, MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG_OVER_SERIAL
		snprintf(_debugOutput, sizeof(_debugOutput), "Subscribed to \"%s\" : %d", subscriptionDef, subscribed);
		Serial.println(_debugOutput);
#endif

		for (int i = 0; i < numberOfEntities; i++) {
			if (_mqttAllEntities[i].subscribe) {
				sprintf(subscriptionDef, DEVICE_NAME "/%s/%s/command", myUniqueId, _mqttAllEntities[i].mqttName);
				subscribed = subscribed && _mqtt.subscribe(subscriptionDef, MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG_OVER_SERIAL
				snprintf(_debugOutput, sizeof(_debugOutput), "Subscribed to \"%s\" : %d", subscriptionDef, subscribed);
				Serial.println(_debugOutput);
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
		sprintf(_debugOutput, "MQTT Failed: RC is %d", _mqtt.state());
		Serial.println(_debugOutput);
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
readEntity(mqttState *singleEntity, char *value, int podIdx)
{
	value[0] = 0;

	switch (singleEntity->entityId) {
	case mqttEntityId::entityBatPct:
		sprintf(value, "%d", pods[podIdx].batteryPct);
		break;
	case mqttEntityId::entityBatVlt:
		sprintf(value, "%0.02f", pods[podIdx].batteryVolts);
		break;
	case mqttEntityId::entityTopSensor:
		sprintf(value, "%s", pods[podIdx].topSensor ? "Wet" : "Dry");
		break;
	case mqttEntityId::entityBotSensor:
		sprintf(value, "%s", pods[podIdx].botSensor ? "Wet" : "Dry");
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
		sprintf(value, "%s", pods[podIdx].version);
		break;
#ifdef DEBUG_WIFI
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

	if (singleEntity->allPods && !POD_DATA_IS_FRESH(podIdx + 1)) {
		strcpy(value, "unavailable");
	}
}

/*
 * addState
 *
 * Query the handled entity in the usual way, and add the cleansed output to the buffer
 */
boolean
addState(mqttState *singleEntity, int podIdx)
{
	char response[MAX_FORMATTED_DATA_VALUE_LENGTH];
	boolean result;

	// Read the register(s)/data
	readEntity(singleEntity, &response[0], podIdx);

	result = addToPayload(response);

	return result;
}

void
sendStatus(void)
{
	char stateAddition[128] = "";

	emptyPayload();

	snprintf(stateAddition, sizeof(stateAddition), "online");
	if (!addToPayload(stateAddition)) {
		return;
	}

	sendMqtt(statusTopic, MQTT_RETAIN);
}

boolean
addConfig(mqttState *singleEntity, int podIdx)
{
	char stateAddition[1024] = "";
	char prettyName[MAX_MQTT_NAME_LENGTH], mqttName[MAX_MQTT_NAME_LENGTH];

	if (singleEntity->allPods) {
		snprintf(mqttName, sizeof(mqttName), "Pod%d_%s", podIdx + 1, singleEntity->mqttName);
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
//	case homeAssistantClass::haClassSelect:
//		sprintf(stateAddition, "\"component\": \"select\"");
//		break;
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
//	case homeAssistantClass::haClassSelect:
//		snprintf(stateAddition, sizeof(stateAddition),
//			 ", \"device_class\": \"enum\""
//			);
//		break;
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
	case mqttEntityId::entityBatVlt:
		sprintf(stateAddition,
			", \"icon\": \"mdi:battery-heart\""
			", \"suggested_display_precision\": 2");
		break;
#ifdef DEBUG_WIFI
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
	case mqttEntityId::entityBatPct:
	case mqttEntityId::entityTopSensor:
	case mqttEntityId::entityBotSensor:
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

	snprintf(stateAddition, sizeof(stateAddition), ", \"availability_topic\": \"%s\"", statusTopic);
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
	int loopCount;

	if (singleEntity == NULL)
		return;

	loopCount = singleEntity->allPods ? NUM_PODS : 1;
	for (int podIdx = 0; podIdx < loopCount; podIdx++) {
		int podNum = podIdx + 1;
		emptyPayload();

		if (doHomeAssistant) {
			const char *entityType;
			switch (singleEntity->haClass) {
			case homeAssistantClass::haClassBox:
//			case homeAssistantClass::haClassNumber:
				entityType = "number";
				break;
//			case homeAssistantClass::haClassSelect:
//				entityType = "select";
//			break;
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
			result = addConfig(singleEntity, podIdx);
		} else {
			if (singleEntity->allPods) {
				snprintf(topic, sizeof(topic), DEVICE_NAME "/%s/Pod%d_%s/state", myUniqueId, podNum, singleEntity->mqttName);
			} else {
				snprintf(topic, sizeof(topic), DEVICE_NAME "/%s/%s/state", myUniqueId, singleEntity->mqttName);
			}
			result = addState(singleEntity, podIdx);
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
	sprintf(_debugOutput, "Topic: %s", topic);
	Serial.println(_debugOutput);
#endif

#ifdef DEBUG_CALLBACKS
	receivedCallbacks++;
#endif // DEBUG_CALLBACKS

	if ((length == 0) || (length >= sizeof(mqttIncomingPayload))) {
#ifdef DEBUG_OVER_SERIAL
		sprintf(_debugOutput, "mqttCallback: bad length: %d", length);
		Serial.println(_debugOutput);
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
	sprintf(_debugOutput, "Payload: %d", length);
	Serial.println(_debugOutput);
	Serial.println(mqttIncomingPayload);
#endif

	// Special case for Home Assistant itself
	if (strcmp(topic, MQTT_SUB_HOMEASSISTANT) == 0) {
		if (strcmp(mqttIncomingPayload, "online") == 0) {
			resendAllData = true;
		} else {
#ifdef DEBUG_OVER_SERIAL
			Serial.println("Unknown homeassistant/status: ");
			Serial.println(mqttIncomingPayload);
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
//		char *singleString;
//		char *endPtr = NULL;
		bool valueProcessingError = false;

		// First, process value.
		switch (mqttEntity->entityId) {
		default:
#ifdef DEBUG_OVER_SERIAL
			sprintf(_debugOutput, "Trying to update an unhandled entity! %d", mqttEntity->entityId);
			Serial.println(_debugOutput);
#endif
#ifdef DEBUG_CALLBACKS
			unknownCallbacks++;
#endif // DEBUG_CALLBACKS
			return; // No further processing possible.
		}

		if (valueProcessingError) {
#ifdef DEBUG_OVER_SERIAL
			snprintf(_debugOutput, sizeof(_debugOutput), "Callback for %s with bad value: ", mqttEntity->mqttName);
			Serial.print(_debugOutput);
			Serial.println(mqttIncomingPayload);
#endif
#ifdef DEBUG_CALLBACKS
			badCallbacks++;
#endif // DEBUG_CALLBACKS
		} else {
			// Now set the value and take appropriate action(s)
			switch (mqttEntity->entityId) {
			default:
#ifdef DEBUG_OVER_SERIAL
				sprintf(_debugOutput, "Trying to write an unhandled entity! %d", mqttEntity->entityId);
				Serial.println(_debugOutput);
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
		snprintf(_debugOutput, sizeof(_debugOutput), "MQTT publish failed to %s", topic);
		Serial.println(_debugOutput);
		Serial.println(_mqttPayload);
#endif
	} else {
#ifdef DEBUG_OVER_SERIAL
		//sprintf(_debugOutput, "MQTT publish success");
		//Serial.println(_debugOutput);
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

#ifdef USE_BUTTON_INTERRUPTS
void
configButtonISR(void)
{
	configButtonPressed = true;
}

void
upButtonISR(void)
{
	upButtonChanged = true;
}

void
downButtonISR(void)
{
	downButtonChanged = true;
}
#endif // USE_BUTTON_INTERRUPTS

#ifdef DEBUG_FREEMEM
uint32_t freeMemory()
{
	return ESP.getFreeHeap();
}
#endif // DEBUG_FREEMEM
