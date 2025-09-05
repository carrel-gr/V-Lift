/*
Name:		Definitions.h
Created:	27 August 2025
Author:		David Carrel

*/

#ifndef _Definitions_h
#define _Definitions_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif


/*************************************************/
/* Start customizations here.                    */
/*************************************************/

// How many pods are being controlled
#define NUM_PODS 2

/*************************************************
 * XIAO ESP32C6 Pin map
 *
 *         vBatt in - A0 - 0      13 - 5V
 *    Top Sensor in - D1 - 1      12 - GND
 * Bottom Sensor in - D2 - 2      11 - 3.3V
 *           Up LED - D3 - 3      10 - D10 - Up button
 *          I2C SDA -    - 4       9 - D9  - Down button
 *          I2C SCL -    - 5       8 - D8  - Up relay
 *         Down LED - D6 - 6       7 - D7  - Down relay
 *************************************************/

#define CONFIG_BUTTON_PIN GPIO_NUM_9 // BOOT/GPIO9
#define BAT_READ_PIN	A0	// ADC Pin on which to read battery voltage
#define TOP_SENSOR_PIN	D1
#define BOT_SENSOR_PIN	D2
#define UP_RELAY_PIN	D8
#define DOWN_RELAY_PIN	D7
#define UP_BUTTON_PIN	D10
#define DOWN_BUTTON_PIN	D9
#define UP_LED_PIN	D3
#define DOWN_LED_PIN	D6

#define USE_BUTTON_INTERRUPTS

// Set your battery MAX/MIN voltage corresponding to 100% and 0%
// For 2 12V AGM batteries in series, nominal values are 24 and 23
// For MakerFocus 10000mAh : max 4.16, min 3.0 (nominal), mult 1.03740648
// DAVE
#define BAT_MAX 24
#define BAT_MIN 23
// A resistor voltage divider is used to bring the voltage into the ADC range
#define BAT_VOLTAGE_DIVIDER 12
// If your ADC is measuring voltage slightly wrong, you can adjust with this.
#define	BAT_MULTIPLIER 1

// Compiling for ESP32 always.  Add any processor specifics here.
#define MP_XIAO_ESP32C6

// Display parameters
#define USE_DISPLAY

// Subnet for private WiFi (192.168.PRIV_SUBNET.x)
#define PRIV_WIFI_SUBNET 237
#define PRIV_WIFI_SSID "V-Lift"
#define PRIV_WIFI_PASS "RZtXCLqHNVJKPopiBh3Z"
#define PRIV_UDP_PORT 9124

// Set this to true to set the "retain" flag when publishing to MQTT
// "retain" is also a flag in mqttState.   This is AND-ed with that.
// This does NOT control whether we tell HA to add "retain" to command topics that it publishes back to us.
#define MQTT_RETAIN true
// Set this to 1 or 0
// Use 0 so if lift is offline, control actions don't arrive later
#define MQTT_SUBSCRIBE_QOS 0
// Define this to enable the HA force_update option for many (not all) sensors.
//#define MQTT_FORCE_UPDATE

// 4096 works well given the RAM requirements.
// You may be able to increase this.
// On boot will request a buffer size of (MAX_MQTT_PAYLOAD_SIZE + MQTT_HEADER_SIZE) for MQTT, and
// MAX_MQTT_PAYLOAD_SIZE for building payloads.  If these fail and your device doesn't boot, you can assume you've set this too high.
#define MAX_MQTT_PAYLOAD_SIZE 4096
#define MIN_MQTT_PAYLOAD_SIZE 512
#define MQTT_HEADER_SIZE 512

#define DEBUG_OVER_SERIAL	// Enable debugging msgs over serial port
//#define DEBUG_FREEMEM		// Enable extra debug for memory usage
//#define DEBUG_WIFI		// Enable extra debug for WiFi
//#define DEBUG_CALLBACKS	// Enable extra debug MQTT callbacks
//#define DEBUG_UPTIME
//#define DEBUG_ZIGBEE
#define DEBUG_UDP

// The device name is used as the MQTT base topic and presence on the network.
#define DEVICE_NAME "VLift"

/*************************************************/
/* Shouldn't need to change anything below this. */
/*************************************************/

#define VERSION_STR_LEN 6

#define uS_TO_S_FACTOR 1000000

#ifdef USE_DISPLAY
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define SCREEN_ADDRESS 0x3C
#define OLED_CHARACTER_WIDTH 21
#endif // USE_DISPLAY

#define MAX_MQTT_NAME_LENGTH 81
#define MAX_FORMATTED_DATA_VALUE_LENGTH 513 // DAVE

// MQTT HA Subscription - Lets us know if HA restarts.
#define MQTT_SUB_HOMEASSISTANT "homeassistant/status"

enum mqttEntityId {
#ifdef DEBUG_FREEMEM
	entityFreemem,
#endif // DEBUG_FREEMEM
#ifdef DEBUG_CALLBACKS
	entityCallbacks,
#endif // DEBUG_CALLBACKS
#ifdef DEBUG_WIFI
	entityRSSI,
	entityBSSID,
	entityTxPower,
	entityWifiRecon,
#endif // DEBUG_WIFI
// DAVE - everything after this needs to be per-pod
#ifdef DEBUG_UPTIME
	entityUptime,
#endif // DEBUG_UPTIME
#if defined(USE_ZIGBEE) && defined(DEBUG_ZIGBEE)
	entityZigSignalStrength,
#endif // USE_ZIGBEE && DEBUG_ZIGBEE
	entityVersion,
	entityBatPct,
	entityBatVlt,
	entityTopSensor,
	entityBotSensor
};

enum mqttUpdateFreq {
	freqTenSec,
	freqOneMin,
	freqFiveMin,
	freqOneHour,
	freqOneDay,
	freqNever		// included in something else (i.e. statusTopic items)
};

enum homeAssistantClass {
	haClassMoisture,
	haClassBattery,
	haClassVoltage,
	haClassDuration,
	haClassInfo,
//	haClassSelect,
	haClassBox,
//	haClassNumber
};

struct mqttState
{
	mqttEntityId entityId;
	char mqttName[MAX_MQTT_NAME_LENGTH];
	bool allPods;
	bool subscribe;
	bool retain;
	homeAssistantClass haClass;
};

// mode is where we want to be
enum liftModes {
	modeUp,
	modeDown,
	modeOff
};

// action is what we are doing right now
enum liftActions {
	actionStop,
	actionRaise,
	actionLower
};

// position is where we are
enum liftPositions {
	positionUp,
	positionAlmostUp,
	positionMiddle,
	positionAlmostDown,
	positionDown
};

struct podState {
	unsigned long		lastUpdate;
	enum liftModes          mode = modeOff;
	enum liftActions	action;
	enum liftPositions	position;
	int                     batteryPct;
	float                   batteryVolts;
	boolean                 topSensor;
	boolean                 botSensor;
	char			version[VERSION_STR_LEN];
};

// Config handling                                                                                                                                                                      
struct Config {
	String wifiSSID;
	String wifiPass;
	String mqttSrvr;
	int mqttPort;
	String mqttUser;
	String mqttPass;
#ifdef MP_XIAO_ESP32C6
	bool extAntenna;
#endif // MP_XIAO_ESP32C6                                                                                                                                                               
	int podNumber;
};

#define PREF_NAME_SSID		"WiFi_SSID"
#define PREF_NAME_PASS		"WiFi_Password"
#define PREF_NAME_MQTT_SRV	"MQTT_Server"
#define PREF_NAME_MQTT_PORT	"MQTT_Port"
#define PREF_NAME_MQTT_USER	"MQTT_Username"
#define PREF_NAME_MQTT_PASS	"MQTT_Password"
#ifdef MP_XIAO_ESP32C6
#define PREF_NAME_EXT_ANT	"Ext_Antenna"
#endif // MP_XIAO_ESP32C6                                                                                                                                                               
#define PREF_NAME_POD_NUM	"Pod_Number"


#endif // ! _Definitions_h
