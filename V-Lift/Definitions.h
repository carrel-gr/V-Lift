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
#define NUM_PODS 6

// How long to wait after detecting up/down before
// stopping the raise/lower action
#define ALMOST_DOWN_DELAY 2000  // 2 seconds
#define ALMOST_UP_DELAY   8000  // 8 seconds

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

// Set your battery MAX/MIN voltage corresponding to 100% and 0%
// For 2 12V AGM SLA batteries in series, nominal values are 25.78 and 23.26
//     and 27.6 when charging
// For MakerFocus 10000mAh : max 4.16, min 3.0 (nominal), mult 1.03740648
#define BAT_MAX 25.78
#define BAT_MIN 23.26
// A resistor voltage divider is used to bring the voltage into the ADC range
// Divider = (top_resistor + bottom_resistor) / bottom_resistor
// top = 470k, bottom = 47k
#define BAT_VOLTAGE_DIVIDER 11.0
// If your ADC is measuring voltage slightly wrong, you can adjust with this.
#define	BAT_MULTIPLIER 1.04

// Compiling for ESP32 always.  Add any processor specifics here.
#define MP_XIAO_ESP32C6

// Display parameters
#define USE_DISPLAY

// Subnet for private WiFi (192.168.PRIV_SUBNET.x)
#define PRIV_WIFI_SUBNET 237
#define PRIV_WIFI_SSID "V-Lift"
#define PRIV_WIFI_PASS "RZtXCLqHNVJKPopiBh3Z"
#define PRIV_UDP_PORT 9124
#define MY_WIFI_CHANNEL 1

// Set this to true to set the "retain" flag when publishing to MQTT
// "retain" is also a flag in mqttState.   This is AND-ed with that.
// This does NOT control whether we tell HA to add "retain" to command topics that it publishes back to us.
#define MQTT_RETAIN true
// Set this to 1 or 0
// Use 0 so if lift is offline, control actions don't arrive later
#define MQTT_SUBSCRIBE_QOS 1
#define MQTT_PUBLISH_QOS 0
// Define this to enable the HA force_update option for many (not all) sensors.
//#define MQTT_FORCE_UPDATE

// You may be able to increase this.
#define MAX_MQTT_BUFFER_SIZE 1024

#define DEBUG_OVER_SERIAL	// Enable debugging msgs over serial port
//#define DEBUG_FREEMEM		// Enable extra debug for memory usage
#define DEBUG_WIFI		// Enable extra debug for WiFi
#define DEBUG_MQTT		// Enable extra debug MQTT
//#define DEBUG_UPTIME
//#define DEBUG_ZIGBEE
//#define DEBUG_UDP
#define DEBUG_SENSORS

// The device name is used as the MQTT base topic and presence on the network.
#define DEVICE_NAME "VLift"

#define USE_SSL
#ifdef USE_SSL
// ISRG Root X1
#define ROOT_CA \
	"-----BEGIN CERTIFICATE-----\n" \
	"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\n" \
	"-----END CERTIFICATE-----\n"
#endif // USE_SSL

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

// Values to pass to digitalWrite() for controlling relays
#define RELAY_ON  HIGH
#define RELAY_OFF LOW

// Values to control sensors - N/O sensor with external pullup resistor and wired to GND
#define SENSOR_PIN_MODE INPUT
#define SENSOR_WET      LOW

// Values to pass for controlling/configuring button LEDs
#define POD_13_BUTTON_MODE      INPUT_PULLUP	// Hardwired N/O buttons to GND
#define POD_13_BUTTON_PRESSED   LOW
#define POD_2456_BUTTON_MODE    INPUT_PULLDOWN	// RF active high buttons
#define POD_2456_BUTTON_PRESSED HIGH
#define BUTTON_LED_ON  HIGH
#define BUTTON_LED_OFF LOW

#define MAX_MQTT_NAME_LENGTH 32
#define MAX_FORMATTED_DATA_VALUE_LENGTH 513 // DAVE

// MQTT HA Subscription - Lets us know if HA restarts.
#define MQTT_SUB_HOMEASSISTANT "homeassistant/status"

enum mqttEntityId {
#ifdef DEBUG_FREEMEM
	entityFreemem,
#endif // DEBUG_FREEMEM
#ifdef DEBUG_WIFI_DAVE_HACK
	entityRSSI,
	entityBSSID,
	entityTxPower,
	entityWifiRecon,
#endif // DEBUG_WIFI
// DAVE - everything after this needs to be per-pod
	entityUptime,
	entityVersion,
	entitySystemMode,
	entityPodMode,
	entityPodAction,
	entityPodPosition,
	entityPodBatPct,
	entityPodBatVlt,
	entityPodTopSensor,
	entityPodBotSensor
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
	haClassSelect,
	haClassBox,
//	haClassNumber
};

struct mqttState
{
	mqttEntityId entityId;
	char mqttName[MAX_MQTT_NAME_LENGTH];
	bool subscribe;
	bool retain;
	bool doEntity;
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
	boolean			forceMode;  // Only used by system (pods[0])
	int                     batteryPct;
	float                   batteryVolts;
	boolean                 topSensorWet;
	boolean                 botSensorWet;
	int32_t			uptime;
	char			version[VERSION_STR_LEN];
};

enum buttonState {
	nothingPressed,
	upPressed,
	downPressed,
	upLongPressed,
	downLongPressed,
	bothPressed
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
