/*
 * Name:		V-Lift.ino
 * Created:	27 August 2025
 * Author:		David Carrel
 */

// DAVE -  todo --
// make mqtt disconnect/reconnect
// restart wifi if gotIp and mqtt is disconnected for too long
// pressing button a 2nd time doesn't stop.
// ADD duty-cycle
// handle top sensor going wet w/o bottom
// Add back web config

#define WIFI_TEST_POWER

#define WIFI_TEST_INACTIVE_TIME 15

#define REMOVE_WEB_CONFIG

#include <cstdint>
// Supporting files
#include "Definitions.h"
#include <Arduino.h>
#include <driver/rtc_io.h>
#include <esp_netif.h>
#include <esp_wifi.h>
#include <NetworkUdp.h>
#ifndef MP_XIAO_ESP32C6
#define LED_BUILTIN 2
#endif // ! MP_XIAO_ESP32C6
#ifndef REMOVE_WEB_CONFIG
#include <WiFiManager.h>
#endif // ! REMOVE_WEB_CONFIG
#include <Preferences.h>
#include <PsychicMqttClient.h>
#include <ElegantOTA.h>
#ifdef USE_DISPLAY
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#endif // USE_DISPLAY

void setButtonLEDs(int freq = 0);

// Device parameters
char _version[VERSION_STR_LEN] = "v2.78";
char myUniqueId[17];
char statusTopic[128];

int myPodNum = -1;
volatile podState *myPod;
volatile podState pods[NUM_PODS + 1];  // 0 is (virtual) system

// WiFi parameters
volatile enum myWifiState wifiState = myWifiState::stopped;
volatile bool wifiRestarted = false;
#ifdef DEBUG_WIFI
uint16_t wifiStoppedCnt = 0;
volatile uint16_t wifiConnectedCnt = 0;
uint16_t wifiGotIpCnt = 0;
#endif // DEBUG_WIFI

#ifdef USE_SSL
const char* rootCA = ROOT_CA;
#endif // USE_SSL

// MQTT parameters
PsychicMqttClient mqttClient;
typedef struct publish {
	char topic[128];
	char payload[MAX_MQTT_BUFFER_SIZE];
} publishEntry_t;
publishEntry_t globalPublishEntry;
#ifdef DEBUG_MQTT
uint32_t mqttRcvCallbacks = 0;
uint32_t mqttUnkCallbacks = 0;
uint32_t mqttBadCallbacks = 0;
uint32_t mqttPubCallbacks = 0;
uint32_t mqttConnects = 0;
uint32_t mqttPublishSuccess = 0;
uint32_t mqttPublishFail = 0;
#endif // DEBUG_MQTT
volatile bool mqttStartupMode = true;
volatile bool mqttCallbackRcvd = false;

// OTA setup
WebServer *otaServer = NULL;

NetworkUDP udp;
#define UDP_BUF_SIZ 128
#ifdef DEBUG_UDP
unsigned int udpPacketsSent = 0;
unsigned int udpPacketsSentErrors = 0;
unsigned int udpPacketsReceived = 0;
unsigned int udpPacketsReceivedErrors = 0;
#endif // DEBUG_UDP

volatile boolean configButtonPressed = false;
buttonState frontButtons = buttonState::nothingPressed;
buttonState remoteButtons = buttonState::nothingPressed;

#ifdef DEBUG_SENSORS
int topSensorChanges = 0;
int botSensorChanges = 0;
#endif // DEBUG_SENSORS

#ifdef USE_DISPLAY
// OLED variables
char _oledOperatingIndicator = '*';
char _oledLine2[OLED_CHARACTER_WIDTH] = "";
char _oledLine3[OLED_CHARACTER_WIDTH] = "";
char _oledLine4[OLED_CHARACTER_WIDTH] = "";
#endif // USE_DISPLAY

// Config handling
Config config;

/*
 * Home Assistant auto-discovered values
 */
static struct mqttState _mqttSysEntities[] =
{
	// Entity,                                "Name",           Subscribe, Retain, doEntity, HA Class
#ifdef DEBUG_WIFI_DAVE_HACK
	{ mqttEntityId::entityRSSI,               "RSSI",               false, false,  true,     homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityBSSID,              "BSSID",              false, false,  true,     homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityTxPower,            "TX_Power",           false, false,  true,     homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityWifiRecon,          "reconnects",         false, false,  true,     homeAssistantClass::haClassInfo },
#endif // DEBUG_WIFI
	{ mqttEntityId::entitySystemMode,         "System_Mode",        true,  true,   true,     homeAssistantClass::haClassSelect },
};

static struct mqttState _mqttPodEntities[] =
{
	// Entity,                                "Name",           Subscribe, Retain, doEntity, HA Class
	{ mqttEntityId::entityVersion,            "Version",            false, true,   false,    homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityUptime,             "Uptime",             false, false,  false,    homeAssistantClass::haClassDuration },
	{ mqttEntityId::entityPodMode,            "Mode",               false, true,   false,    homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityPodAction,          "Action",             false, true,   false,    homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityPodPosition,        "Position",           false, true,   true,     homeAssistantClass::haClassInfo },
	{ mqttEntityId::entityPodBatPct,          "Battery",            false, true,   true,     homeAssistantClass::haClassBattery },
	{ mqttEntityId::entityPodBatVlt,          "Voltage",            false, true,   false,    homeAssistantClass::haClassVoltage },
	{ mqttEntityId::entityPodTopSensor,       "Top_Sensor",         false, true,   true,     homeAssistantClass::haClassMoisture },
	{ mqttEntityId::entityPodBotSensor,       "Bottom_Sensor",      false, true,   true,     homeAssistantClass::haClassMoisture }
};

// These timers are used in the main loop.
#define INTERVAL_ONE_SECOND 1000
#define INTERVAL_FIVE_SECONDS 5000
#define INTERVAL_TEN_SECONDS 10000
#define INTERVAL_THIRTY_SECONDS 30000
#define INTERVAL_ONE_MINUTE 60000
#define INTERVAL_FIVE_MINUTE 300000
#define INTERVAL_ONE_HOUR 3600000
#define INTERVAL_ONE_DAY 86400000
#define RUNSTATE_INTERVAL (INTERVAL_ONE_SECOND * 2)
#define UPDATE_STATUS_BAR_INTERVAL (INTERVAL_ONE_SECOND / 2)
#define STATUS_INTERVAL INTERVAL_TEN_SECONDS
#define DATA_INTERVAL_URGENT INTERVAL_FIVE_SECONDS
#define DATA_INTERVAL_NORMAL INTERVAL_ONE_MINUTE
#define HA_DATA_INTERVAL_INITIAL (INTERVAL_TEN_SECONDS * 2)
#define HA_DATA_INTERVAL_NORMAL INTERVAL_FIVE_MINUTE
#define POD2POD_DATA_INTERVAL_URGENT INTERVAL_ONE_SECOND
#define POD2POD_DATA_INTERVAL_NORMAL INTERVAL_FIVE_SECONDS
#define RELAY_MAX_FREQ INTERVAL_FIVE_SECONDS

#define POD_DATA_IS_FRESH(_podNum) ((pods[_podNum].lastUpdate != 0) && (getUptimeSeconds() - pods[_podNum].lastUpdate) < (4 * (POD2POD_DATA_INTERVAL_NORMAL / 1000)))

#ifdef USE_DISPLAY
// Pins GPIO22 and GPIO21 (SCL/SDA) if ESP32
// Pins GPIO23 and GPIO22 (SCL/SDA) if XIAO ESP32C6
Adafruit_SSD1306 _display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, 0);
#endif // USE_DISPLAY

#define MY_ERROR_CHECK(_func) ESP_ERROR_CHECK_WITHOUT_ABORT(_func)

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
	Serial.println("Starting...");
	delay(500);
#endif

	// Wire.setClock(10000);

#ifdef USE_DISPLAY
	// Display time
	_display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);  // initialize OLED
	_display.clearDisplay();
	_display.display();
	updateOLED(false, "", "Starting...", _version);
#endif // USE_DISPLAY

	for (int podIdx = 0; podIdx <= NUM_PODS; podIdx++) {
		pods[podIdx].lastUpdate = 0;
	}

	preferences.begin(DEVICE_NAME, true); // RO
	config.wifiSSID = preferences.getString(PREF_NAME_SSID, "");
	config.wifiPass = preferences.getString(PREF_NAME_PASS, "");
#ifdef DAVE_TEST_SSID
	config.wifiSSID = DAVE_TEST_SSID;
	config.wifiPass = DAVE_TEST_PASS;
#endif // DAVE_TEST_SSID
	config.mqttSrvr = preferences.getString(PREF_NAME_MQTT_SRV, "");
	config.mqttPort = preferences.getInt(PREF_NAME_MQTT_PORT, 0);
	config.mqttUser = preferences.getString(PREF_NAME_MQTT_USER, "");
	config.mqttPass = preferences.getString(PREF_NAME_MQTT_PASS, "");
#ifdef MP_XIAO_ESP32C6
	config.extAntenna = preferences.getBool(PREF_NAME_EXT_ANT, false);
#ifdef DAVE_TEST_EXT_ANT
	config.extAntenna = DAVE_TEST_EXT_ANT;
#endif // DAVE_TEST_EXT_ANT
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

	MY_ERROR_CHECK(esp_wifi_stop());
	delay(100);
	MY_ERROR_CHECK(esp_netif_init());
	MY_ERROR_CHECK(esp_event_loop_create_default());
	/* Register Event handler */
	MY_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
							     &wifiEventHandler, NULL, NULL));
	MY_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
							     &wifiEventHandler, NULL, NULL));
	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	cfg.nvs_enable = false;
	MY_ERROR_CHECK(esp_wifi_init(&cfg));
	MY_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

	// Set WiFi mode and our unique identity.
	if (myPodNum == 1) {
		uint8_t mac[6];
		MY_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));
		MY_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
		snprintf(myUniqueId, sizeof(myUniqueId), DEVICE_NAME "-%0X%02X%02X", mac[3] & 0xf, mac[4], mac[5]);
		snprintf(statusTopic, sizeof(statusTopic), DEVICE_NAME "/%s/status", myUniqueId);

		// If config is not setup, then enter config mode
		if ((config.wifiSSID == "") ||
//		    (config.wifiPass == "") ||
		    (config.mqttSrvr == "") ||
		    (config.mqttPort == 0) ||
		    (config.mqttUser == "") ||
		    (config.mqttPass == "")) {
			configLoop();
			ESP.restart();
		}
	} else {
		MY_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
		snprintf(myUniqueId, sizeof(myUniqueId), DEVICE_NAME "-pod%d", myPodNum);
	}
#ifdef USE_DISPLAY
	{
		char line3[OLED_CHARACTER_WIDTH];
		snprintf(line3, sizeof(line3), "Pod # %d", myPodNum);
		updateOLED(false, "Config is set", line3, _version);
		delay(750);
	}
#endif // USE_DISPLAY

	pinMode(BAT_READ_PIN, INPUT);		// Configure pin for battery voltage
	pinMode(TOP_SENSOR_PIN, SENSOR_PIN_MODE);
	pinMode(BOT_SENSOR_PIN, SENSOR_PIN_MODE);

	if ((myPodNum == 1) || (myPodNum == 3)) {
		pinMode(UP_BUTTON_PIN, POD_13_BUTTON_MODE);
		pinMode(DOWN_BUTTON_PIN, POD_13_BUTTON_MODE);
	} else {
		pinMode(UP_BUTTON_PIN, POD_2456_BUTTON_MODE);
		pinMode(DOWN_BUTTON_PIN, POD_2456_BUTTON_MODE);
	}
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
	wifiState = myWifiState::stopped;
	checkWifiStatus();

	// Initialize data.
	// myPodNum was set above.
	strlcpy((char *)myPod->version, &_version[1], sizeof(myPod->version)); // skip initial 'v'
	readBattery();
	readPodState();
	pods[0].mode = modeUp;		// System mode
	pods[0].action = actionStop;	// System action (not used)
	pods[0].forceMode = false;
	pods[0].lastUpdate = getUptimeSeconds();
	myPod->mode = modeUp;		// Always go to UP state when we boot.
	myPod->action = actionStop;	// and stop until loop reads position
	myPod->forceMode = false;

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
#endif // DEBUG_OVER_SERIAL

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

#ifdef REMOVE_WEB_CONFIG
#ifdef USE_DISPLAY
	updateOLED(false, "Config", "not set.", "No web config!");
#endif // USE_DISPLAY
#ifdef DEBUG_OVER_SERIAL
	Serial.println("Configuration is not set and web config is not available.");
#endif // DEBUG_OVER_SERIAL
	delay(3000);
#else // REMOVE_WEB_CONFIG
	configHandler();
#endif // REMOVE_WEB_CONFIG
}

#ifndef REMOVE_WEB_CONFIG
void
configHandler(void)
{
	Preferences preferences;
	char mqttPort[8];

	delete otaServer;
	otaServer = NULL;
	esp_wifi_stop();
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
#endif // ! REMOVE_WEB_CONFIG

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
	static unsigned long lastRunSendHaData = 0;
	static unsigned long lastRunSendData = 0;
	static unsigned long lastRunPodData = 0;
	static unsigned long pod2podDataInterval = POD2POD_DATA_INTERVAL_NORMAL;
	static bool mqttUrgent = false;
	bool pod2podUrgent = false;
	boolean wifiIsOn = false;
	boolean mqttIsOn = false;

	// Read button state
	if (configButtonPressed) {
		Preferences preferences;
		configButtonPressed = false;
		myPodNum = getPodNumFromButton();
#if defined(MP_XIAO_ESP32C6)
		boolean extAnt = getExtAntFromButton();
#endif // MP_XIAO_ESP32C6
		preferences.begin(DEVICE_NAME, false); // RW
		preferences.putInt(PREF_NAME_POD_NUM, myPodNum);
#if defined(MP_XIAO_ESP32C6)
		preferences.putBool(PREF_NAME_EXT_ANT, extAnt);
#endif // MP_XIAO_ESP32C6
#ifdef DAVE_TEST_SSID
		preferences.putString(PREF_NAME_SSID, DAVE_TEST_SSID);
		preferences.putString(PREF_NAME_PASS, DAVE_TEST_PASS);
#endif // DAVE_TEST_SSID
		preferences.end();
		if (myPodNum == 1) {
#ifndef REMOVE_WEB_CONFIG
			configHandler();
#endif // ! REMOVE_WEB_CONFIG
		}
		ESP.restart();
	}

	// Make sure WiFi is good
	wifiIsOn = checkWifiStatus();

	if (myPodNum == 1) {
		// make sure mqtt is still connected
		if (wifiIsOn) {
			mqttIsOn = mqttClient.connected();
		} else {
			mqttIsOn = false;
		}
	}

	// Handle OTA
	if ((otaServer != NULL) &&
	    ((wifiState == myWifiState::gotIp) ||
	     ((myPodNum == 1) && (wifiState == myWifiState::started)))) {
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
	if (readPodState()) {        // reads/sets sensors, and sets myPod->position
		mqttUrgent = true;
		pod2podUrgent = true;
	}

	// MQTT/P2P might have asynchronously updated pods[0].mode or buttons
	if (getPodMessages()) {
		mqttUrgent = true;
	}
	if (myPodNum == 1) {
		if (setSystemModeFromButtons()) {
			// Set system (pods[0]) mode based on buttons and system action
			pod2podUrgent = true;
		}
		// setSystemModeFromButtons() always consumes both buttons so clear them now.
		frontButtons = buttonState::nothingPressed;
		remoteButtons = buttonState::nothingPressed;
	} else {
		if (frontButtons != buttonState::nothingPressed) {
			remoteButtons = frontButtons; // Save state in remoteButtons to be consumed by sendPodInfo()
			frontButtons = buttonState::nothingPressed;
			pod2podUrgent = true;
		}
	}
	if (myPod->mode != pods[0].mode) {
		myPod->mode = pods[0].mode;
		mqttUrgent = true;
		pod2podUrgent = true;
	}
	myPod->forceMode = pods[0].forceMode;
	// On remote pods, try waiting for state from #1 before activating the bootup default
	if ((myPodNum == 1) || pods[0].lastUpdate || (getUptimeSeconds() > 120)) {
		setPodAction();		// Set myPod->action
		engagePodAction();	// activate relays
	}

	if (mqttCallbackRcvd) {
		mqttUrgent = true;
		mqttCallbackRcvd = false;
	}
	if (wifiRestarted) {
		pod2podUrgent = true;
		wifiRestarted = false;
	}
	if (pod2podUrgent) {
		// Read and transmit all state to other pods
		pod2podDataInterval = POD2POD_DATA_INTERVAL_URGENT;
		pod2podUrgent = false;
	}

	setButtonLEDs();

	if (myPodNum == 1) {
		if (((wifiState == myWifiState::started) || (wifiState == myWifiState::gotIp)) &&
		    checkTimer(&lastRunPodData, pod2podDataInterval)) {
			sendCommandsToRemotePods();
			sendPodInfo();
			pod2podDataInterval = POD2POD_DATA_INTERVAL_NORMAL;
		}

		if ((wifiState == myWifiState::gotIp) && mqttIsOn) {
			sendStatus(&lastRunStatus);	// Send status/keepalive
			sendHaData(&lastRunSendHaData, mqttStartupMode);
			sendData(&lastRunSendData, mqttUrgent || mqttStartupMode);
			mqttUrgent = false;
			mqttStartupMode = false;
		}
	} else {
		if ((wifiState == myWifiState::gotIp) && checkTimer(&lastRunPodData, pod2podDataInterval)) {
			sendPodInfo();
			remoteButtons = buttonState::nothingPressed; // consumed by sendPodinfo()
			pod2podDataInterval = POD2POD_DATA_INTERVAL_NORMAL;
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

int
parseBool (char *data, const char *key)
{
	char *valueStr;
	int ret;

	valueStr = strstr(data, key);
	if (valueStr == NULL) {
		return -1;
	}
	valueStr += strlen(key);

	if (*valueStr == '1') {
		ret = 1;
	} else if (*valueStr == '0') {
		ret = 0;
	} else {
		ret = -1;
	}

	return ret;
}

void
getSystemModeFromNumberOne (char *buf)
{
	liftModes mode = (liftModes)parseInt(buf, "MO=");
	int fm = parseBool(buf, "FM=");
	switch (mode) {
	case liftModes::modeUp:
	case liftModes::modeDown:
	case liftModes::modeOff:
		pods[0].mode = mode;
//		pods[0].action = (liftActions)parseInt(buf, "AC=");
		pods[0].forceMode = fm == 1 ? true : false;
		pods[0].lastUpdate = getUptimeSeconds();
	}
}

void
sendCommandsToRemotePods (void)
{
	IPAddress podIP(192, 168, PRIV_WIFI_SUBNET, 255);

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

bool
getPodMessages (void)
{
	char buf[UDP_BUF_SIZ];
	int packetSize = udp.parsePacket();
	int loops = 0;
	bool mqttUrgent = false;

	while (packetSize > 0) {
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
			if (podNum == 0) {
				getSystemModeFromNumberOne(&buf[0]);
			} else if (podNum >= 1 && podNum <= NUM_PODS) {
				if (podNum == myPodNum) {
#ifdef DEBUG_OVER_SERIAL
					Serial.println("Received pod update from self.  Ignoring ...");
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_UDP
					udpPacketsReceivedErrors++;
#endif // DEBUG_UDP
				} else if (getRemotePodStatus(podNum, &buf[0])) {
					mqttUrgent = true;
				}
			} else {
#ifdef DEBUG_UDP
				udpPacketsReceivedErrors++;
#endif // DEBUG_UDP
			}
		} else {
#ifdef DEBUG_UDP
			udpPacketsReceivedErrors++;
#endif // DEBUG_UDP
		}
		loops++;
		if (loops > NUM_PODS) {
			break;
		}
		packetSize = udp.parsePacket(); // Try to get another.
	}
	return mqttUrgent;
}

bool
getRemotePodStatus (int podNum, char *buf)
{
	bool goodUpdate = true;
	bool mqttUrgent = false;

	pods[podNum].batteryPct = parseInt(buf, "BP=");
	{
		int batteryMilliVolts = parseInt(buf, "BM=");
		if (batteryMilliVolts == -1) {
			pods[podNum].batteryVolts = (float)-1;
		} else {
			pods[podNum].batteryVolts = (float)batteryMilliVolts / 1000.0;
		}
	}
	{
		liftModes mode = (liftModes)parseInt(buf, "MO=");
		if (mode == -1) {
			goodUpdate = false;
		} else {
			if (mode != pods[podNum].mode) {
				if (myPodNum == 1) mqttUrgent = true;
				pods[podNum].mode = mode;
			}
		}
	}
	pods[podNum].forceMode = parseBool(buf, "FM=") == 1;
	{
		liftActions action = (liftActions)parseInt(buf, "AC=");
		if (action == -1) {
			goodUpdate = false;
		} else {
			if (action != pods[podNum].action) {
				if (myPodNum == 1) mqttUrgent = true;
				pods[podNum].action = action;
			}
		}
	}
	{
		liftPositions position = (liftPositions)parseInt(buf, "PO=");
		if (position == -1) {
			goodUpdate = false;
		} else {
			if (position != pods[podNum].position) {
				if (myPodNum == 1) mqttUrgent = true;
				pods[podNum].position = position;
			}
		}
	}
	{
		int val = parseBool(buf, "TS=");
		if (val == -1) {
			goodUpdate = false;
		} else {
			bool ts = (val == 1);
			if (ts != pods[podNum].topSensorWet) {
				if (myPodNum == 1) mqttUrgent = true;
				pods[podNum].topSensorWet = ts;
			}
		}
	}
	{
		int val = parseBool(buf, "BS=");
		if (val == -1) {
			goodUpdate = false;
		} else {
			bool bs = (val == 1);
			if (bs != pods[podNum].botSensorWet) {
				if (myPodNum == 1) mqttUrgent = true;
				pods[podNum].botSensorWet = bs;
			}
		}
	}
	if (myPodNum == 1) {
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
	parseStr(buf, "VV=", (char *)pods[podNum].version, sizeof(pods[podNum].version));
	pods[podNum].uptime = parseInt(buf, "UT=");
	if (goodUpdate) {
		pods[podNum].lastUpdate = getUptimeSeconds();
	}
	return mqttUrgent;
}

void
sendPodInfo (void)
{
	IPAddress bcastIP(192, 168, PRIV_WIFI_SUBNET, 255);
	buttonState tmpButtons = (myPodNum == 1) ? buttonState::nothingPressed : remoteButtons;

	udp.beginPacket(bcastIP, PRIV_UDP_PORT);
	udp.printf("PN=%d,BP=%d,BM=%d,MO=%d,FM=%c,AC=%d,PO=%d,TS=%c,BS=%c,FB=%d,VV=%s,UT=%ld", myPodNum, myPod->batteryPct, (int)(myPod->batteryVolts * 1000.0),
		   myPod->mode, myPod->forceMode ? '1' : '0', myPod->action, myPod->position, myPod->topSensorWet ? '1' : '0', myPod->botSensorWet ? '1' : '0',
		   tmpButtons, myPod->version, myPod->uptime);
#ifdef DEBUG_OVER_SERIAL
	if (tmpButtons != buttonState::nothingPressed) {
		Serial.printf("Sending buttons (%d) to #1.\n", remoteButtons);
	}
#endif // DEBUG_OVER_SERIAL
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
		if (myPod->forceMode) {
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
		if (myPod->forceMode) {
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
	static unsigned long lastAction = 0;

	// Set pod relay actions
	if (myPod->forceMode) {
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
	if (action != myPod->action) {
		if (checkTimer(&lastAction, RELAY_MAX_FREQ)) {
#ifdef DEBUG_OVER_SERIAL
			Serial.printf("Changing action from %d to %d.\n", myPod->action, action);
#endif // DEBUG_OVER_SERIAL
			myPod->action = action;
		}
	}
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
bool
setSystemModeFromButtons(void)
{
	bool updateUrgent = false;
	buttonState buttons = frontButtons;

	if (buttons == buttonState::nothingPressed) {
		buttons = remoteButtons;
	}

	switch (buttons) {
	case buttonState::nothingPressed:
		break;
	case buttonState::upPressed:
		pods[0].mode = modeUp;
		pods[0].forceMode = false;
		pods[0].lastUpdate = getUptimeSeconds();
		updateUrgent = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Up button press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState::downPressed:
		pods[0].mode = modeDown;
		pods[0].forceMode = false;
		pods[0].lastUpdate = getUptimeSeconds();
		updateUrgent = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Down button press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState::upLongPressed:
		pods[0].mode = modeUp;
		pods[0].forceMode = true;
		pods[0].lastUpdate = getUptimeSeconds();
		updateUrgent = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Up button LONG press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState::downLongPressed:
		pods[0].mode = modeDown;
		pods[0].forceMode = true;
		pods[0].lastUpdate = getUptimeSeconds();
		updateUrgent = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Down button LONG press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	case buttonState:: bothPressed:
		pods[0].mode = modeOff;
		pods[0].forceMode = false;
		pods[0].lastUpdate = getUptimeSeconds();
		updateUrgent = true;
#ifdef DEBUG_OVER_SERIAL
		Serial.println("Both buttons press consumed.");
#endif // DEBUG_OVER_SERIAL
		break;
	}

	return updateUrgent;
}

// Read pod water sensors and determine position
// Sets topSensorWet, botSensorWet, and position
bool
readPodState(void)
{
	bool topWet = false, botWet = false, updateUrgent = false;
	int topCount = 0, bottomCount = 0;
	uint32_t now;
	static uint32_t almostDownTime = 0, almostUpTime = 0;
	liftPositions newPosition = myPod->position;

#define SENSOR_NUM_SAMPLES 16
	for (int i = 0; i < SENSOR_NUM_SAMPLES; i++) {
		if (digitalRead(TOP_SENSOR_PIN) == SENSOR_WET) topCount++;
		if (digitalRead(BOT_SENSOR_PIN) == SENSOR_WET) bottomCount++;
	}
	if (topCount > (SENSOR_NUM_SAMPLES / 2)) { // If half the readings
		topWet = true;
	}
	if (bottomCount > (SENSOR_NUM_SAMPLES / 2)) { // If half the readings
		botWet = true;
	}

	if (myPod->topSensorWet != topWet) {
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Top sensor changed from %s to %s.\n", myPod->topSensorWet ? "WET" : "DRY", topWet ? "WET" : "DRY");
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_SENSORS
		topSensorChanges++;
#endif // DEBUG_SENSORS
		myPod->topSensorWet = topWet;
		updateUrgent = true;
	}
	if (myPod->botSensorWet != botWet) {
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Bottom sensor changed from %s to %s.\n", myPod->botSensorWet ? "WET" : "DRY", botWet ? "WET" : "DRY");
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_SENSORS
		botSensorChanges++;
#endif // DEBUG_SENSORS
		myPod->botSensorWet = botWet;
		updateUrgent = true;
	}

	now = millis();
	if (topWet) {
		if (myPod->position == positionMiddle) {
			newPosition = positionAlmostDown;
			almostDownTime = now;
		} else if ((myPod->position == positionAlmostDown) &&
			   ((almostDownTime + ALMOST_DOWN_DELAY) > now)) {
			// Do nothing until delay passes
		} else /* Up || AlmostUp || Down || (AlmostDown && delay) */ {
			newPosition = positionDown;
			almostDownTime = 0;
		}
		almostUpTime = 0;
	} else if (botWet) {
		newPosition = positionMiddle;
		almostDownTime = 0;
		almostUpTime = 0;
	} else /* neither wet */ {
		if ((myPod->position == positionMiddle) || (myPod->position == positionDown) || (myPod->position == positionAlmostDown)) {
			newPosition = positionAlmostUp;
			almostUpTime = now;
		} else if ((myPod->position == positionAlmostUp) &&
			   ((almostUpTime + ALMOST_UP_DELAY) > now)) {
			// Do nothing until delay passes
		} else /* Up || (AlmostUp && delay) */ {
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
	myPod->uptime = getUptimeSeconds();
	myPod->lastUpdate = getUptimeSeconds();

	return updateUrgent;
}

void
readPodButtons(void)
{
	static unsigned long lastRun = 0;
	static unsigned long upPressedMillis = 0, downPressedMillis = 0;
#define BUTTON_INTERVAL 50
#define LONG_PRESS_MILLIS 3000 // 3 seconds

	if (checkTimer(&lastRun, BUTTON_INTERVAL)) {
		boolean upPressed, downPressed;
		unsigned long now = millis();
		if ((myPodNum == 1) || (myPodNum == 3)) {
			upPressed = (digitalRead(UP_BUTTON_PIN) == POD_13_BUTTON_PRESSED);
			downPressed = (digitalRead(DOWN_BUTTON_PIN) == POD_13_BUTTON_PRESSED);
		} else {
			upPressed = (digitalRead(UP_BUTTON_PIN) == POD_2456_BUTTON_PRESSED);
			downPressed = (digitalRead(DOWN_BUTTON_PIN) == POD_2456_BUTTON_PRESSED);
		}
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
	float volts, pct, mvBatt = 0.0;

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
	volatile static uint32_t uptimeSeconds = 0;
	volatile static uint32_t uptimeWrapSeconds = 0;
	uint32_t nowSeconds = millis() / 1000;

	if (nowSeconds < uptimeSeconds) {
		// We wrapped
		uptimeWrapSeconds += (UINT32_MAX / 1000);;
	}
	uptimeSeconds = nowSeconds;
	return uptimeWrapSeconds + uptimeSeconds;
}

/*
 * wifiEventHandler()
 * wifi callback function
 */
static void wifiEventHandler(void *arg, esp_event_base_t eventBase,
                               int32_t eventId, void *eventData)
{
	if (eventBase == WIFI_EVENT) {
		switch (eventId) {
		case WIFI_EVENT_AP_STACONNECTED:
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			{
				wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *) eventData;
				Serial.printf("Remote WiFi " MACSTR " joined, AID=%d\n", MAC2STR(event->mac), event->aid);
			}
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			break;
		case WIFI_EVENT_AP_STADISCONNECTED:
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			{
				wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *) eventData;
				Serial.printf("Remote WiFi " MACSTR " left, AID=%d, reason:%d\n", MAC2STR(event->mac), event->aid, event->reason);
			}
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			break;
		case WIFI_EVENT_STA_STOP:
			if (wifiState == myWifiState::gotIp) {
				wifiState = myWifiState::stopped;
			} // else do not change
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			Serial.println("Station WiFi stopped");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			break;
		case WIFI_EVENT_STA_START:
			if (wifiState == myWifiState::started) {
				esp_wifi_connect();
			}
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			Serial.println("Station WiFi started");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			break;
		case WIFI_EVENT_STA_CONNECTED:
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			{
				wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *) eventData;
				Serial.printf("Station WiFi connected to [" MACSTR "] chan: %d\n", MAC2STR(event->bssid), event->channel);
			}
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
#ifdef DEBUG_WIFI
			wifiConnectedCnt = wifiConnectedCnt + 1;
#endif // DEBUG_WIFI
			break;
		case WIFI_EVENT_STA_DISCONNECTED:
			if (wifiState == myWifiState::gotIp) {
				wifiState = myWifiState::disconnected;
			}
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			Serial.println("Station WiFi disconnected");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			break;
		}
	} else if (eventBase == IP_EVENT) {
		if (eventId == IP_EVENT_STA_GOT_IP) {
			wifiState = myWifiState::gotIp;
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			ip_event_got_ip_t *event = (ip_event_got_ip_t *) eventData;
			Serial.printf("Got IP:" IPSTR "\n", IP2STR(&event->ip_info.ip));
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
		}
	}
}

int
wifiDoScans (uint8_t *chanPtr, uint8_t *bssid)
{
	int goodScans = 0;
	int32_t bestRSSI = -100;

	*chanPtr = 0;

	// Pick channel
	for (uint8_t chan = 1; chan <= 11; chan += 5) {  // Channels 1, 6, & 11
		esp_err_t ret1, ret2;
		wifi_ap_record_t scanRecord;
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
		unsigned long before;
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
		{
			wifi_scan_config_t *scanCfg = (wifi_scan_config_t *)calloc(1, sizeof(wifi_scan_config_t));
			scanCfg->ssid = (uint8_t *)config.wifiSSID.c_str();
			scanCfg->channel = chan;
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			before = millis();
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			ret1 = esp_wifi_scan_start(scanCfg, true);
			free(scanCfg);
		}
		if (ret1 == ESP_OK) {
			ret2 = esp_wifi_scan_get_ap_record(&scanRecord);
		} else {
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			Serial.println("scan_start failed.");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
			ret2 = -1;
		}
		esp_wifi_clear_ap_list();
		if (ret2 == ESP_OK) {
			goodScans++;
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
			Serial.printf("Scan %hhu returned %d/%d [" MACSTR "] (%lu msec) RSSI=%d -", chan, ret1, ret2,
				      MAC2STR(scanRecord.bssid), millis() - before, scanRecord.rssi);
			if (scanRecord.phy_11b) Serial.print(" B");
			if (scanRecord.phy_11g) Serial.print(" G");
			if (scanRecord.phy_11n) Serial.print(" N");
			if (scanRecord.phy_11ax) Serial.print(" AX");
			Serial.println(".");
		} else {
			Serial.printf("Scan %hhu record unavailable.\n", chan);
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
		}
		if ((ret2 == ESP_OK) && (scanRecord.rssi > bestRSSI) && (scanRecord.rssi < -10)) {
			bestRSSI = scanRecord.rssi;
			*chanPtr = chan;
			memcpy(bssid, scanRecord.bssid, 6);
		}
	}
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
	Serial.printf("Best channel is %hhd\n", *chanPtr);
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI

	return goodScans;
}

void
wifiSetupApSta(uint8_t chan, uint8_t *bssid, esp_netif_t *netifAp, esp_netif_t *netifSta)
{
	esp_netif_ip_info_t privIpInfo;
	IP4_ADDR(&privIpInfo.ip, 192, 168, PRIV_WIFI_SUBNET, myPodNum);
	IP4_ADDR(&privIpInfo.gw, 192, 168, PRIV_WIFI_SUBNET, 1);
	IP4_ADDR(&privIpInfo.netmask, 255, 255, 255, 0);

	esp_netif_dhcps_stop(netifAp);
	esp_netif_set_ip_info(netifAp, &privIpInfo);
	esp_netif_set_hostname(netifAp, myUniqueId);
	{
		wifi_config_t *wifiApConfig = (wifi_config_t *)calloc(1, sizeof(wifi_config_t));
		strlcpy((char *)wifiApConfig->ap.ssid, PRIV_WIFI_SSID, sizeof(wifiApConfig->ap.ssid));
		strlcpy((char *)wifiApConfig->ap.password, PRIV_WIFI_PASS, sizeof(wifiApConfig->ap.password));
		wifiApConfig->ap.ssid_len = strlen(PRIV_WIFI_SSID);
		wifiApConfig->ap.channel = chan;
		wifiApConfig->ap.authmode = WIFI_AUTH_WPA2_PSK;
		wifiApConfig->ap.max_connection = (NUM_PODS * 2);
		MY_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, wifiApConfig));
		free(wifiApConfig);
	}

	// Setup Station
	esp_netif_set_hostname(netifSta, myUniqueId);
	{
		wifi_config_t *wifiStaConfig = (wifi_config_t *)calloc(1, sizeof(wifi_config_t));
		strlcpy((char *)wifiStaConfig->sta.ssid, config.wifiSSID.c_str(), sizeof(wifiStaConfig->sta.ssid));
		strlcpy((char *)wifiStaConfig->sta.password, config.wifiPass.c_str(), sizeof(wifiStaConfig->sta.password));
		wifiStaConfig->sta.channel = chan;
		wifiStaConfig->sta.bssid_set = true;
		memcpy(wifiStaConfig->sta.bssid, bssid, sizeof(wifiStaConfig->sta.bssid));
		wifiStaConfig->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
		MY_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifiStaConfig));
		free(wifiStaConfig);
	}
}

void
wifiSetupSta(esp_netif_t *netifSta)
{
	esp_netif_ip_info_t privIpInfo;
	IP4_ADDR(&privIpInfo.ip, 192, 168, PRIV_WIFI_SUBNET, myPodNum);
	IP4_ADDR(&privIpInfo.gw, 192, 168, PRIV_WIFI_SUBNET, 1);
	IP4_ADDR(&privIpInfo.netmask, 255, 255, 255, 0);

	// Set up in Station Mode - Will be connecting to #1's access point
	MY_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	esp_netif_dhcpc_stop(netifSta);
	esp_netif_set_ip_info(netifSta, &privIpInfo);
	esp_netif_set_hostname(netifSta, myUniqueId);
	{
		wifi_config_t *wifiStaConfig = (wifi_config_t *)calloc(1, sizeof(wifi_config_t));
		strlcpy((char *)wifiStaConfig->sta.ssid, PRIV_WIFI_SSID, sizeof(wifiStaConfig->sta.ssid));
		strlcpy((char *)wifiStaConfig->sta.password, PRIV_WIFI_PASS, sizeof(wifiStaConfig->sta.password));
		wifiStaConfig->sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
		MY_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, wifiStaConfig));
		free(wifiStaConfig);
	}
}

void
getWifiChanInfo (char *chanInfo, size_t len)
{
	if (wifiState == myWifiState::gotIp) {
		wifi_ap_record_t apInfo;
		uint8_t staProtocol;
		esp_wifi_sta_get_ap_info(&apInfo);
		esp_wifi_get_protocol(WIFI_IF_STA, &staProtocol);
		snprintf(chanInfo, len, "%d (%s%s%s%s)", apInfo.primary,
			 (staProtocol & WIFI_PROTOCOL_11B) && apInfo.phy_11b ? "B" : "",
			 (staProtocol & WIFI_PROTOCOL_11G) && apInfo.phy_11g ? "G" : "",
			 (staProtocol & WIFI_PROTOCOL_11N) && apInfo.phy_11n ? "N" : "",
			 (staProtocol & WIFI_PROTOCOL_11AX) && apInfo.phy_11ax ? "AX" : "");
	} else {
		snprintf(chanInfo, len, "n/a");
	}
}

int
getWifiRSSI (void)
{
	int rssi;

	if (wifiState == myWifiState::gotIp) {
		esp_wifi_sta_get_rssi(&rssi);
	} else {
		rssi = 0;
	}
	return rssi;
}

void
getWifiBSSID (byte *bssid)
{
	wifi_ap_record_t apInfo;
	if ((wifiState == myWifiState::gotIp) && (esp_wifi_sta_get_ap_info(&apInfo) == ESP_OK)) {
		memcpy(bssid, apInfo.bssid, 6);
	} else {
		memset(bssid, 0, 6);
	}
}

bool
checkWifiStatus (void)
{
	static unsigned long lastWifiTry = 0;
	enum myWifiState startingWifiState;;
	static enum myWifiState previousWifiState = myWifiState::stopped;;
	static esp_netif_t *netifSta = NULL, *netifAp = NULL;
	static int scanTries = 0;
	static int startedCnt = 0;
	static bool udpIsOn = false;
#ifdef WIFI_TEST_POWER
	static uint8_t wifiPower = WIFI_POWER_8_5dBm;
#endif // WIFI_TEST_POWER
	unsigned int restartTime, reconnFreq;

	// These are multiplied by 10 sec below when waiting in start mode.
	if (myPodNum == 1) {
		restartTime = 30;	// after 300 sec = 5 min
		reconnFreq = 4;		// every 40 sec
	} else {
		restartTime = 9;	// after 90 sec = 1.5 min
		reconnFreq = 3;		// every 30 sec
	}

	startingWifiState = wifiState;

	switch (startingWifiState) {
	case myWifiState::stopped:
#ifdef DEBUG_WIFI
		wifiStoppedCnt++;
#endif // DEBUG_WIFI
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("Connecting to %s\n", myPodNum == 1 ? config.wifiSSID.c_str() : PRIV_WIFI_SSID);
#endif // DEBUG_OVER_SERIAL
		if (udpIsOn) {
			udp.clear();
			udp.stop();
			udpIsOn = false;
		}
		if (otaServer) {
			delete otaServer;
			otaServer = NULL;
		}

		MY_ERROR_CHECK(esp_wifi_stop());
		if (netifSta != NULL) {
			esp_netif_destroy_default_wifi(netifSta);
			netifSta = NULL;
		}
		if (netifAp != NULL) {
			esp_netif_destroy_default_wifi(netifAp);
			netifAp = NULL;
		}
		if (myPodNum == 1) {
			MY_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
			MY_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_AP, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_11AX));
		}
		MY_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_STA, WIFI_BW_HT20));
		MY_ERROR_CHECK(esp_wifi_set_protocol(WIFI_IF_STA, WIFI_PROTOCOL_11B|WIFI_PROTOCOL_11G|WIFI_PROTOCOL_11N|WIFI_PROTOCOL_11AX));
		delay(100);
#ifdef WIFI_TEST_POWER
		if (myPodNum == 1) {
			if (wifiPower == WIFI_POWER_8_5dBm) {
				wifiPower = WIFI_POWER_13dBm;
			} else if (wifiPower == WIFI_POWER_13dBm) {
				wifiPower = WIFI_POWER_21dBm;
			} else {
				wifiPower = WIFI_POWER_8_5dBm;
			}
		} else {
			wifiPower = WIFI_POWER_8_5dBm;
		}
#endif // WIFI_TEST_POWER
		if (myPodNum == 1) {
			MY_ERROR_CHECK(esp_wifi_start());
#ifdef WIFI_TEST_POWER
			MY_ERROR_CHECK(esp_wifi_set_max_tx_power(wifiPower));
#endif // WIFI_TEST_POWER
		}
		wifiState = myWifiState::configuring;
		lastWifiTry = 0; // No delay for fallthru below
		scanTries = 0;
		/* FALLTHRU */
	case myWifiState::configuring:
		if (myPodNum == 1) {
			if (checkTimer(&lastWifiTry, INTERVAL_TEN_SECONDS)) {
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
				Serial.println("Configuring WiFi");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
				uint8_t chan, bssid[6];
				int goodScans = wifiDoScans(&chan, &bssid[0]);
				scanTries++;
				if (scanTries < 4) {
					if (goodScans < 3) {
						break; // Try scans again later
					}
				} else if (scanTries < 7) {
					if (goodScans < 2) {
						break; // Try scans again later
					}
				} else if (goodScans == 0) {
					break; // Try scans again later
				}
				MY_ERROR_CHECK(esp_wifi_stop());
				delay(100);
				netifAp = esp_netif_create_default_wifi_ap();
				netifSta = esp_netif_create_default_wifi_sta();
				wifiSetupApSta(chan, bssid, netifAp, netifSta);
			} else {
				break;
			}
		} else {
			netifSta = esp_netif_create_default_wifi_sta();
			wifiSetupSta(netifSta);
		}
		startedCnt = 0;
		wifiState = myWifiState::started;
		lastWifiTry = 0;  // No delay next time in this loop
		MY_ERROR_CHECK(esp_wifi_start());
#ifdef WIFI_TEST_POWER
		MY_ERROR_CHECK(esp_wifi_set_max_tx_power(wifiPower));
#endif // WIFI_TEST_POWER
#ifdef WIFI_TEST_INACTIVE_TIME
		esp_wifi_set_inactive_time(WIFI_IF_STA, WIFI_TEST_INACTIVE_TIME);
#endif // WIFI_TEST_INACTIVE_TIME
		break;
	case myWifiState::disconnected:
		startedCnt = - 1; // So reconn will fire immediately
		wifiState = myWifiState::started;
		/* FALLTHRU */
	case myWifiState::started:
		// Should normally leave this state because eventHandler calls connect()
		if (checkTimer(&lastWifiTry, INTERVAL_TEN_SECONDS)) {
			startedCnt++;
			if (startedCnt >= restartTime) {
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
				Serial.println("Restarting WiFi");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
				wifiState = myWifiState::stopped;
			} else if ((startedCnt % reconnFreq) == 0) {
#if defined(DEBUG_OVER_SERIAL) && defined(DEBUG_WIFI)
				Serial.println("Reconnecting WiFi");
#endif // DEBUG_OVER_SERIAL && DEBUG_WIFI
				MY_ERROR_CHECK(esp_wifi_disconnect());
				delay(100);
				MY_ERROR_CHECK(esp_wifi_connect());
			}
		}
		break;
	case myWifiState::gotIp:
		if (previousWifiState != myWifiState::gotIp) {
			IPAddress privIP(192, 168, PRIV_WIFI_SUBNET, myPodNum);
			char chanInfo[12];
			wifi_ap_record_t apInfo;
#ifdef DEBUG_WIFI
			wifiGotIpCnt++;
#endif // DEBUG_WIFI
			if (myPodNum == 1) {
				static boolean doneOnce = false;
				if (!doneOnce) {
					doneOnce = true;
					initMqtt();
				}
			}

			if (udpIsOn) {
				udp.clear();
				udp.stop();
				delay(50);
			}
			udp.begin(privIP, PRIV_UDP_PORT);
			udpIsOn = true;
			if (otaServer) {
				delete otaServer;
			}
			otaServer = new WebServer(privIP, 80);
			ElegantOTA.begin(otaServer);    // Start ElegantOTA
			otaServer->begin();

			wifiRestarted = true;
#ifdef DEBUG_OVER_SERIAL
			MY_ERROR_CHECK(esp_wifi_sta_get_ap_info(&apInfo));
			getWifiChanInfo(&chanInfo[0], sizeof(chanInfo));
			if (netifSta) {
				esp_netif_ip_info_t ipInfo;
				esp_netif_get_ip_info(netifSta, &ipInfo);
				Serial.printf("WiFi IP is " IPSTR " (%s)\n", IP2STR(&ipInfo.ip), apInfo.ssid);
			} else {
				Serial.println("WiFi IP unavailable.");
			}
			Serial.printf("WiFi BSSID is " MACSTR "\n", MAC2STR(apInfo.bssid));
			Serial.printf("WiFi RSSI: %hhd\n", apInfo.rssi);
			Serial.printf("WiFi channel: %s\n", chanInfo);
			if (myPodNum == 1) {
				if (netifAp) {
					esp_netif_ip_info_t ipInfo;
					esp_netif_get_ip_info(netifAp, &ipInfo);
					wifi_config_t apConf;
					MY_ERROR_CHECK(esp_wifi_get_config(WIFI_IF_STA, &apConf));
					Serial.printf("softAP IP address: " IPSTR " : " IPSTR " (%s)\n", IP2STR(&ipInfo.ip),
						      IP2STR(&ipInfo.netmask), apConf.ap.ssid);
				} else {
					Serial.println("softAP IP unavailable.");
				}
			}
#endif // DEBUG_OVER_SERIAL
		}
		break;
	}

	previousWifiState = startingWifiState;

	return startingWifiState == myWifiState::gotIp;
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
	static int rssi = 0;

	_display.clearDisplay();
	_display.setTextSize(1);
	_display.setTextColor(WHITE);
	_display.setCursor(0, CURSOR_LINE_1);

	char line1[OLED_CHARACTER_WIDTH];

	// Only update the operating indicator once per half second.
	if (checkTimer(&updateStatusBar, UPDATE_STATUS_BAR_INTERVAL)) {
		// Simply swap between space and asterisk every time we come here to give some indication of activity
		_oledOperatingIndicator = (_oledOperatingIndicator == '*') ? ' ' : '*';
		rssi = getWifiRSSI();
	}

	// There's 20 characters we can play with, width wise.
	{
		char wifiStatus, mqttStatus;

		wifiStatus = mqttStatus = ' ';
		wifiStatus = wifiState == myWifiState::gotIp ? 'W' : ' ';
		mqttStatus = mqttClient.connected() ? 'M' : ' ';
		snprintf(line1, sizeof(line1), "Pod%d %c%c%c         %3d",
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
	if (rssi < 0 && rssi >= -55) { 
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
		activeSys = POD_DATA_IS_FRESH(0) ? 1 : 0;
		for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
			if (POD_DATA_IS_FRESH(podNum)) {
				activePeers++;
			}
		}
		if (myPodNum == 1) {
			wifi_sta_list_t staList;
			esp_wifi_ap_get_sta_list(&staList);
			activeWiFi = staList.num;
		} else {
			activeWiFi = wifiState == myWifiState::gotIp ? 1 : 0;
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
		snprintf(line4, sizeof(line4), "WiFi recon: %hu/%hu/%hu", wifiStoppedCnt, wifiConnectedCnt, wifiGotIpCnt);
		dbgIdx = 3;
	} else if (dbgIdx < 4) {
		int8_t power;
		esp_wifi_get_max_tx_power(&power);
		snprintf(line4, sizeof(line4), "WiFi TX: %0.01fdBm", (float)power / 4.0f);
		dbgIdx = 4;
	} else if (dbgIdx < 5) {
		char chanInfo[12];
		getWifiChanInfo(&chanInfo[0], sizeof(chanInfo));
		snprintf(line4, sizeof(line4), "WiFi CH: %s", chanInfo);
		dbgIdx = 5;
#endif // DEBUG_WIFI
#ifdef DEBUG_MQTT
	} else if ((dbgIdx < 6) && (myPodNum == 1)) {
		snprintf(line4, sizeof(line4), "MQTT CB: %lu/%lu/%lu/%lu", mqttRcvCallbacks, mqttUnkCallbacks, mqttBadCallbacks, mqttPubCallbacks);
		dbgIdx = 6;
	} else if ((dbgIdx < 9) && (myPodNum == 1)) {
		snprintf(line4, sizeof(line4), "MQTT conn: %lu", mqttConnects);
		dbgIdx = 9;
	} else if ((dbgIdx < 10) && (myPodNum == 1)) {
		snprintf(line4, sizeof(line4), "MQTT pub: %lu/%lu", mqttPublishSuccess, mqttPublishFail);
		dbgIdx = 10;
#endif // DEBUG_MQTT
#ifdef DEBUG_UPTIME
	} else if (dbgIdx < 15) {
		snprintf(line4, sizeof(line4), "Uptime: %lu", getUptimeSeconds());
		dbgIdx = 15;
#endif // DEBUG_UPTIME
#ifdef DEBUG_UDP
	} else if (dbgIdx < 20) {
		snprintf(line4, sizeof(line4), "UDP: S=%u/%u  R=%u/%u", udpPacketsSent, udpPacketsSentErrors, udpPacketsReceived, udpPacketsReceivedErrors);
		dbgIdx = 20;
#endif // DEBUG_UDP
	} else if (dbgIdx < 21) {
		snprintf(line4, sizeof(line4), "Bat: %d%%  %0.02fV", myPod->batteryPct, myPod->batteryVolts);
		dbgIdx = 21;
#ifdef DEBUG_WIFI
	} else if (dbgIdx < 22) {
		snprintf(line4, sizeof(line4), "Ext Ant: %s", config.extAntenna ? "On" : "Off");
		dbgIdx = 22;
#endif // DEBUG_WIFI
#ifdef DEBUG_SENSORS
	} else if (dbgIdx < 23) {
		snprintf(line4, sizeof(line4), "%s/%d : %s/%d",
			 myPod->topSensorWet ? "Wet" : "Dry", topSensorChanges,
			 myPod->botSensorWet ? "Wet" : "Dry", botSensorChanges);
		dbgIdx = 23;
#endif // DEBUG_SENSORS
	} else { // Must be last
		snprintf(line4, sizeof(line4), "Version: %s", _version);
		dbgIdx = 0;
	}

	updateOLED(false, line2, line3, line4);
}
#endif // USE_DISPLAY

/*
 * initialize and connect MQTT
 */
void
initMqtt(void)
{
	String server, lwt;

	// Configure MQTT to the address and port specified above
#ifdef USE_SSL
	server = "mqtts://";
#else // USE_SSL
	server = "mqtt://";
#endif // USE_SSL
	server += config.mqttSrvr;
	if (config.mqttPort != 0) {
		server += ":";
		server += config.mqttPort;
	}
	mqttClient.setServer(server.c_str());
	mqttClient.setCredentials(config.mqttUser.c_str(), config.mqttPass.c_str());
	mqttClient.setClientId(myUniqueId);
	mqttClient.setBufferSize(MAX_MQTT_BUFFER_SIZE);
#ifdef USE_SSL
	mqttClient.setCACert(rootCA);
#endif // USE_SSL
	lwt = "{ \"systemStatus\": \"offline\"";
	for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
		lwt += ", \"pod";
		lwt += podNum;
		lwt += "Status\": \"unavailable\"";
	}
	lwt += " }";
	mqttClient.setWill(statusTopic, 1, true, lwt.c_str());
	mqttClient.setCleanSession(true);
	{
		esp_mqtt_client_config_t *mqttCfg = mqttClient.getMqttConfig();
		mqttCfg->network.timeout_ms = 3000; // 3s (default is 10s)
	}

	mqttClient.onConnect(onMqttConnect);
	mqttClient.onPublish(onMqttPublish);
	// And any messages we are subscribed to will be pushed to the mqttCallback function for processing
	mqttClient.onMessage(mqttCallback);

	// Connect to MQTT
	mqttClient.connect();
}

void
onMqttPublish(int packetId)
{
#ifdef DEBUG_MQTT
	mqttPubCallbacks++;
#endif // DEBUG_MQTT
}

/*
 * onMqttConnect
 *
 * callback for connect/reconnects
 */
void
onMqttConnect(bool sessionPresent)
{
	String subscriptionDef;
	int numberOfEntities = sizeof(_mqttSysEntities) / sizeof(struct mqttState);

#ifdef DEBUG_MQTT
	mqttConnects++;
#endif // DEBUG_MQTT
#ifdef DEBUG_OVER_SERIAL
	Serial.println("MQTT connected.");
#endif // DEBUG_OVER_SERIAL

	// Special case for Home Assistant
	subscriptionDef = MQTT_SUB_HOMEASSISTANT;
	mqttClient.subscribe(subscriptionDef.c_str(), MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG_OVER_SERIAL
	Serial.println("Subscribed to " + subscriptionDef);
#endif // DEBUG_OVER_SERIAL

	for (int i = 0; i < numberOfEntities; i++) {
		if (_mqttSysEntities[i].subscribe) {
			subscriptionDef = DEVICE_NAME;
			subscriptionDef += "/";
			subscriptionDef += myUniqueId;
			subscriptionDef += "/";
			subscriptionDef += _mqttSysEntities[i].mqttName;
			subscriptionDef += "/command";
			mqttClient.subscribe(subscriptionDef.c_str(), MQTT_SUBSCRIBE_QOS);
#ifdef DEBUG_OVER_SERIAL
			Serial.println("Subscribed to " + subscriptionDef);
#endif // DEBUG_OVER_SERIAL
		}
	}

	// Once we set up the first time, any reconnects should be persistent sessions
	mqttClient.setCleanSession(false);
	mqttStartupMode = true;
}

mqttState *
lookupSubscription(char *entityName)
{
	int numberOfEntities = sizeof(_mqttSysEntities) / sizeof(struct mqttState);
	for (int i = 0; i < numberOfEntities; i++) {
		if (_mqttSysEntities[i].subscribe &&
		    !strcmp(entityName, _mqttSysEntities[i].mqttName)) {
			return &_mqttSysEntities[i];
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
	case mqttEntityId::entityUptime:
		sprintf(value, "%ld", pods[podNum].uptime);
		break;
	case mqttEntityId::entityVersion:
		sprintf(value, "%s", pods[podNum].version);
		break;
#ifdef DEBUG_WIFI_DAVE_HACK
	case mqttEntityId::entityRSSI:
		sprintf(value, "%d", getWifiRSSI());
		break;
	case mqttEntityId::entityBSSID:
		{
			byte bssid[6];
			getWifiBSSID(&bssid[0]);
			sprintf(value, MACSTR, MAC2STR(bssid));
		}
		break;
	case mqttEntityId::entityTxPower:
		{
			int8_t power;
			esp_wifi_get_max_tx_power(&power);
			sprintf(value, "%0.1f", (float)power / 4.0);
		}
		break;
	case mqttEntityId::entityWifiRecon:
		sprintf(value, "%lu", wifiGotIpCnt);
		break;
#endif // DEBUG_WIFI
	}

	if ((podNum != 0) && !POD_DATA_IS_FRESH(podNum)) {
		strcpy(value, "unavailable");
	}
}

/*
 * addState
 *
 * Query the handled entity in the usual way, and add the cleansed output to the buffer
 */
boolean
addState(publishEntry_t *publish, mqttState *singleEntity, int podNum)
{
	char response[MAX_FORMATTED_DATA_VALUE_LENGTH];
	boolean result;

	// Read the register(s)/data
	readEntity(singleEntity, &response[0], podNum);

	result = addToPayload(publish, response);

	return result;
}

boolean
addPodState(publishEntry_t *publish, int podNum)
{
	char response[MAX_FORMATTED_DATA_VALUE_LENGTH];
	boolean result;
	int numberOfEntities = sizeof(_mqttPodEntities) / sizeof(struct mqttState);

	result = addToPayload(publish, "{ ");
	for (int i = 0; (i < numberOfEntities) && result; i++) {
		char value[16];

		// Read the register(s)/data
		readEntity(&_mqttPodEntities[i], &value[0], podNum);
		snprintf(response, sizeof(response), "%s\"%s\": \"%s\"", i == 0 ? "" : ", ", _mqttPodEntities[i].mqttName, value);

		result = addToPayload(publish, response);
	}
	if (result) {
		result = addToPayload(publish, " }");
	}

	return result;
}

void
sendStatus(unsigned long *lastRun)
{
	char stateAddition[256] = "";
	publishEntry_t *publish = &globalPublishEntry;

	if (!checkTimer(lastRun, STATUS_INTERVAL)) {
		return;
	}

	clearPublishEntry(publish);

	strlcat(publish->topic, statusTopic, sizeof(publish->topic));

	snprintf(stateAddition, sizeof(stateAddition), "{ \"systemStatus\": \"online\"");
	for (int podNum = 1; podNum <= NUM_PODS; podNum++) {
		char moreState[32];
		snprintf(moreState, sizeof(moreState), ", \"pod%dStatus\": \"%s\"", podNum,
			 POD_DATA_IS_FRESH(podNum) ? "online" : "offline");
		strlcat(stateAddition, moreState, sizeof(stateAddition));
	}
#ifdef DEBUG_WIFI
	{
		char moreState[64];
		int8_t power;
		char chanInfo[12];
		getWifiChanInfo(&chanInfo[0], sizeof(chanInfo));
		esp_wifi_get_max_tx_power(&power);
		snprintf(moreState, sizeof(moreState), ", \"wifi\": \"%s %0.01fdBm %hu/%hu/%hu\"", chanInfo, (float)power / 4.0f, wifiStoppedCnt, wifiConnectedCnt, wifiGotIpCnt);
		strlcat(stateAddition, moreState, sizeof(stateAddition));
	}
#endif // DEBUG_WIFI
#ifdef DEBUG_FREEMEM
	{
		char moreState[32];
		snprintf(moreState, sizeof(moreState), ", \"mem\": \"%lu\"", freeMemory());
		strlcat(stateAddition, moreState, sizeof(stateAddition));
	}
#endif // DEBUG_FREEMEM
	strlcat(stateAddition, " }", sizeof(stateAddition));
	if (!addToPayload(publish, stateAddition)) {
		*lastRun = 0;
		return;
	}

	if (!sendMqtt(publish, MQTT_RETAIN)) {
		*lastRun = 0;
	}
}

boolean
addConfig(publishEntry_t *publish, mqttState *singleEntity, int podNum)
{
	char stateAddition[1024] = "";
	char prettyName[MAX_MQTT_NAME_LENGTH + 8], mqttName[MAX_MQTT_NAME_LENGTH + 8];

	if (podNum != 0) {
		snprintf(mqttName, sizeof(mqttName), "Pod%d_%s", podNum, singleEntity->mqttName);
	} else {
		strlcpy(mqttName, singleEntity->mqttName, sizeof(mqttName));
	}

	sprintf(stateAddition, "{");
	if (!addToPayload(publish, stateAddition)) {
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
	if (!addToPayload(publish, stateAddition)) {
		return false;
	}

	snprintf(stateAddition, sizeof(stateAddition),
		 ", \"device\": {"
		 " \"name\": \"%s\", \"model\": \"V-Lift\", \"manufacturer\": \"Sunstream\","
		 " \"identifiers\": [\"%s\"]}",
		 myUniqueId, myUniqueId);
	if (!addToPayload(publish, stateAddition)) {
		return false;
	}

	strlcpy(prettyName, mqttName, sizeof(prettyName));
	while(char *ch = strchr(prettyName, '_')) {
		*ch = ' ';
	}
	snprintf(stateAddition, sizeof(stateAddition), ", \"name\": \"%s\"", prettyName);
	if (!addToPayload(publish, stateAddition)) {
		return false;
	}

	snprintf(stateAddition, sizeof(stateAddition), ", \"unique_id\": \"%s_%s\"", myUniqueId, mqttName);
	if (!addToPayload(publish, stateAddition)) {
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
		if (!addToPayload(publish, stateAddition)) {
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
	case mqttEntityId::entityUptime:
	case mqttEntityId::entityPodBatPct:
	case mqttEntityId::entityPodTopSensor:
	case mqttEntityId::entityPodBotSensor:
		// Use default icon
		strcpy(stateAddition, "");
		break;
	}
	if (strlen(stateAddition) != 0) {
		if (!addToPayload(publish, stateAddition)) {
			return false;
		}
	}

	if (singleEntity->subscribe) {
		sprintf(stateAddition, ", \"qos\": %d", MQTT_SUBSCRIBE_QOS);
		if (!addToPayload(publish, stateAddition)) {
			return false;
		}
	}

	if (podNum == 0) {
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"state_topic\": \"" DEVICE_NAME "/%s/%s/state\"",
			 myUniqueId, mqttName);
	} else {
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"state_topic\": \"" DEVICE_NAME "/%s/Pod%d/state\""
			 ", \"value_template\": \"{{ value_json.%s | default(\\\"\\\") }}\""
			 ", \"json_attributes_topic\": \"" DEVICE_NAME "/%s/Pod%d/state\"",
			 myUniqueId, podNum, singleEntity->mqttName, myUniqueId, podNum);
	}
	if (!addToPayload(publish, stateAddition)) {
		return false;
	}

	if (singleEntity->subscribe) {
		sprintf(stateAddition, ", \"command_topic\": \"" DEVICE_NAME "/%s/%s/command\"",
			myUniqueId, mqttName);
		if (!addToPayload(publish, stateAddition)) {
			return false;
		}
	}

	if (podNum != 0) {
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"availability_template\": \"{{ \\\"online\\\" if value_json.pod%dStatus == \\\"online\\\" else \\\"offline\\\" }}\""
			 ", \"availability_topic\": \"%s\"", podNum, statusTopic);
	} else {
		snprintf(stateAddition, sizeof(stateAddition),
			 ", \"availability_template\": \"{{ value_json.systemStatus | default(\\\"\\\") }}\""
			 ", \"availability_topic\": \"%s\"", statusTopic);
	}
	if (!addToPayload(publish, stateAddition)) {
		return false;
	}

	strcpy(stateAddition, "}");
	if (!addToPayload(publish, stateAddition)) {
		return false;
	}

	return true;
}


boolean
addToPayload(publishEntry_t *publish, const char* addition)
{
	int targetRequestedSize = strlen(publish->payload) + strlen(addition);

	if (targetRequestedSize > (sizeof(publish->payload) - 1)) {
		snprintf(publish->payload, sizeof(publish->payload), "{\r\n    \"mqttError\": \"Length of payload exceeds %d bytes.  Length would be %d bytes.\"\r\n}",
			 (sizeof(publish->payload) - 1), targetRequestedSize);
		return false;
	} else {
		strlcat(publish->payload, addition, sizeof(publish->payload));
		return true;
	}
}

#define MQTT_PUBLISH_BURST 4
void
sendData(unsigned long *lastRun, bool urgent)
{
	static int nextToSend = 0;
	static unsigned long dataInterval = DATA_INTERVAL_NORMAL;
	int curSent = 0;
	int numSysEntities = sizeof(_mqttSysEntities) / sizeof(struct mqttState);
	int totalEntities = numSysEntities + NUM_PODS;

	if (urgent) {
		dataInterval = DATA_INTERVAL_URGENT;
		if (*lastRun == 0) {
			checkTimer(lastRun, dataInterval); // Just reset timer.
		}
	}

	if ((nextToSend >= totalEntities) && checkTimer(lastRun, dataInterval)) {
		nextToSend = 0;  // Only restart if we finished previous run.
		dataInterval = DATA_INTERVAL_NORMAL;
	}

	for (int i = nextToSend; (i < totalEntities) && (curSent < MQTT_PUBLISH_BURST); i++, nextToSend++) {
		int podNum, entityIdx;
		struct mqttState *entity;
		publishEntry_t *publish = &globalPublishEntry;

		if (i < numSysEntities) {
			podNum = 0;
			entityIdx = i;
			entity = &_mqttSysEntities[entityIdx];
		} else {
			podNum = (i - numSysEntities) + 1;
			entity = NULL;
		}

		clearPublishEntry(publish);

		if (!sendDataFromMqttState(publish, entity, false, podNum)) {
			break;
		}

		curSent++;
	}
}

void
sendHaData(unsigned long *lastRun, bool startupMode)
{
	static int nextToSend = 0;
	static unsigned long haDataInterval = HA_DATA_INTERVAL_INITIAL;
	int curSent = 0;
	int numSysEntities = sizeof(_mqttSysEntities) / sizeof(struct mqttState);
	int numPodEntities = sizeof(_mqttPodEntities) / sizeof(struct mqttState);
	int totalEntities = numSysEntities + (numPodEntities * NUM_PODS);

	if (startupMode) {
		// Will send once immediately. Then again after the "initial" interval.  Then "normal"
		nextToSend = 0;
		haDataInterval = HA_DATA_INTERVAL_INITIAL;
		*lastRun = 0;
		checkTimer(lastRun, haDataInterval); // Just reset timer.
	}

	if ((nextToSend >= totalEntities) && checkTimer(lastRun, haDataInterval)) {
		nextToSend = 0;  // Only restart if we finished previous run.
		haDataInterval = HA_DATA_INTERVAL_NORMAL;
	}

	for (int i = nextToSend; (i < totalEntities) && (curSent < MQTT_PUBLISH_BURST); i++, nextToSend++) {
		int podNum, entityIdx;
		struct mqttState *entity;
		publishEntry_t *publish = &globalPublishEntry;

		if (i < numSysEntities) {
			podNum = 0;
			entityIdx = i;
			entity = &_mqttSysEntities[entityIdx];
		} else {
			podNum = ((i - numSysEntities) / numPodEntities) + 1;
			entityIdx = (i - numSysEntities) % numPodEntities;
			entity = &_mqttPodEntities[entityIdx];
		}

		if (!entity->doEntity) {
			continue;
		}

		clearPublishEntry(publish);

		if (!sendDataFromMqttState(publish, entity, true, podNum)) {
			break;
		}

		curSent++;
	}
}

bool
sendDataFromMqttState(publishEntry_t *publish, mqttState *singleEntity, bool doHomeAssistant, int podNum)
{
	boolean result = false;
	boolean retain;

	if (doHomeAssistant) {
		const char *entityType;
		switch (singleEntity->haClass) {
		case homeAssistantClass::haClassBox:
//		case homeAssistantClass::haClassNumber:
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

		if (podNum == 0) {
			snprintf(publish->topic, sizeof(publish->topic), "homeassistant/%s/%s/%s/config", entityType, myUniqueId, singleEntity->mqttName);
		} else {
			snprintf(publish->topic, sizeof(publish->topic), "homeassistant/%s/%s/Pod%d_%s/config", entityType, myUniqueId, podNum, singleEntity->mqttName);
		}
		retain = singleEntity->retain;
		result = addConfig(publish, singleEntity, podNum);
	} else {
		if (podNum == 0) {
			snprintf(publish->topic, sizeof(publish->topic), DEVICE_NAME "/%s/%s/state", myUniqueId, singleEntity->mqttName);
			retain = singleEntity->retain;
			result = addState(publish, singleEntity, podNum);
		} else {
			snprintf(publish->topic, sizeof(publish->topic), DEVICE_NAME "/%s/Pod%d/state", myUniqueId, podNum);
			retain = MQTT_RETAIN;
			result = addPodState(publish, podNum);
		}
	}

	if (result) {
		// And send
		result = sendMqtt(publish, retain ? MQTT_RETAIN : false);
	}
	return result;
}


/*
 * mqttCallback()
 *
 * This function is executed when an MQTT message arrives on a topic that we are subscribed to.
 */
void
mqttCallback(char *topic, char *message, int retain, int qos, bool dup)
{
	char mqttIncomingPayload[64] = ""; // Should be enough to cover command requests
	mqttState *mqttEntity = NULL;
	int length = strlen(message);

#ifdef DEBUG_OVER_SERIAL
	Serial.printf("Topic: %s\n", topic);
#endif // DEBUG_OVER_SERIAL

#ifdef DEBUG_MQTT
	mqttRcvCallbacks++;
#endif // DEBUG_MQTT

	if ((length == 0) || (length >= sizeof(mqttIncomingPayload))) {
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("mqttCallback: bad length: %d\n", length);
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_MQTT
		mqttBadCallbacks++;
#endif // DEBUG_MQTT
		return; // We won't be doing anything
	} else {
		// Get the payload (ensure NULL termination)
		strlcpy(mqttIncomingPayload, (char *)message, length + 1);
	}
#ifdef DEBUG_OVER_SERIAL
	Serial.printf("Payload: %d\n", length);
	Serial.println(mqttIncomingPayload);
#endif // DEBUG_OVER_SERIAL

	// Special case for Home Assistant itself
	if (strcmp(topic, MQTT_SUB_HOMEASSISTANT) == 0) {
		if (strcmp(mqttIncomingPayload, "online") == 0) {
			mqttCallbackRcvd = true;
		} else if (strcmp(mqttIncomingPayload, "offline") == 0) {
			// Nothing to do.
		} else {
#ifdef DEBUG_OVER_SERIAL
			Serial.println("Unknown homeassistant/status: ");
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_MQTT
			mqttBadCallbacks++;
#endif // DEBUG_MQTT
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
#ifdef DEBUG_MQTT
			mqttUnkCallbacks++;
#endif // DEBUG_MQTT
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
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_MQTT
			mqttUnkCallbacks++;
#endif // DEBUG_MQTT
			return; // No further processing possible.
		}

		if (valueProcessingError) {
#ifdef DEBUG_OVER_SERIAL
			Serial.printf("Callback for %s with bad value.\n", mqttEntity->mqttName);
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_MQTT
			mqttBadCallbacks++;
#endif // DEBUG_MQTT
		} else {
			// Now set the value and take appropriate action(s)
			switch (mqttEntity->entityId) {
			case mqttEntityId::entitySystemMode:
				if (!strncmp(singleString, "Up", 3)) {
					pods[0].mode = liftModes::modeUp;
					pods[0].forceMode = false;
					pods[0].lastUpdate = getUptimeSeconds();
				} else if (!strncmp(singleString, "Down", 5)) {
					pods[0].mode = liftModes::modeDown;
					pods[0].forceMode = false;
					pods[0].lastUpdate = getUptimeSeconds();
				} else if (!strncmp(singleString, "Off", 4)) {
					pods[0].mode = liftModes::modeOff;
					pods[0].forceMode = false;
					pods[0].lastUpdate = getUptimeSeconds();
#ifdef DEBUG_MQTT
				} else {
					mqttBadCallbacks++;
#endif // DEBUG_MQTT
				}
				mqttCallbackRcvd = true;
				break;
			default:
#ifdef DEBUG_OVER_SERIAL
				Serial.printf("Trying to write an unhandled entity! %d\n", mqttEntity->entityId);
#endif // DEBUG_OVER_SERIAL
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
bool
sendMqtt(publishEntry_t *publish, bool retain)
{
	bool ret;
	int packetId;

	if (wifiState == myWifiState::gotIp) {
		// Attempt a send
		packetId = mqttClient.publish(publish->topic, MQTT_PUBLISH_QOS, retain, publish->payload, strlen(publish->payload), true);
	} else {
		packetId = -111;
	}

	if (packetId < 0) {
		ret = false;
#ifdef DEBUG_OVER_SERIAL
		Serial.printf("MQTT publish failed to %s\n", publish->topic);
		Serial.println(publish->payload);
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_MQTT
		mqttPublishFail++;
#endif // DEBUG_MQTT
	} else {
		ret = true;
#ifdef DEBUG_OVER_SERIAL
//		Serial.printf("MQTT publish success (%d): %s\n", ret, publish->topic);
//		Serial.println(publish->payload);
#endif // DEBUG_OVER_SERIAL
#ifdef DEBUG_MQTT
		mqttPublishSuccess++;
#endif // DEBUG_MQTT
	}

	return ret;
}

/*
 * publishEntry functions
 */
void
clearPublishEntry(publishEntry_t *publish)
{
	publish->topic[0] = '\0';
	publish->payload[0] = '\0';
}

void
configButtonISR(void)
{
	configButtonPressed = true;
}

#ifdef DEBUG_FREEMEM
uint32_t
freeMemory(void)
{
	return ESP.getFreeHeap();
}
#endif // DEBUG_FREEMEM
