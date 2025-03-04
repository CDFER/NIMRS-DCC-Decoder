#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Preferences.h>

#include <NmraDcc.h>
#include <NeoPixelBus.h>
#include <ESPTelnet.h>

#include "secrets.h"

TaskHandle_t motorTaskHandle;

// #define DEBUG_DCC_MSG

const uint16_t PixelCount = 8;	// this example assumes 4 pixels, making it smaller will cause a failure
const uint8_t PixelPin = 13;	// make sure to set this to the correct pin, ignored for Esp8266
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> Strip(PixelCount, PixelPin);

ESPTelnet telnet;
IPAddress ip;
uint16_t telnetPort = 23;

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 25

#define DCC_PIN 23

#define LED_INDICATOR_PIN 16

#define MOTOR_EN_PIN 25
#define MOTOR_A_PIN 26
#define MOTOR_B_PIN 27

#define SAFE_BOOT_IN_PIN 2
#define SAFE_BOOT_GND_PIN 15

#define VCC_RAIL_SENSE 36
#define VCC_RAIL_FACTOR (33.0 + 10.0) / 10.0

uint8_t dccDirection = 0;
bool dccUpdated = true;

uint8_t dccSpeed = 0;
uint8_t numSpeedSteps = SPEED_STEP_128;

uint8_t motorStart;
uint8_t motorMax;

// Structure for CV Values Table
struct CVPair {
	uint16_t CV;
	uint8_t Value;
};

// CV Addresses we will be using
#define CV_VSTART 2
#define CV_VHIGH 5

// Default CV Values Table
CVPair FactoryDefaultCVs[] = {
	// The CV Below defines the Short DCC Address
	{ CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS },

	// Three Step Speed Table
	{ CV_VSTART, 64 },	//155
	{ CV_VHIGH, 255 },	//180

	// These two CVs define the Long DCC Address
	{ CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_MSB(DEFAULT_DECODER_ADDRESS) },
	{ CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB, CALC_MULTIFUNCTION_EXTENDED_ADDRESS_LSB(DEFAULT_DECODER_ADDRESS) },

	// ONLY uncomment 1 CV_29_CONFIG line below as approprate
	//{CV_29_CONFIG,                                      0}, // Short Address 14 Speed Steps
	//{CV_29_CONFIG,                       CV29_F0_LOCATION}, // Short Address 28/128 Speed Steps
	{ CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION },  // Long  Address 28/128 Speed Steps
};

NmraDcc Dcc;

uint8_t FactoryDefaultCVIndex = 0;

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange(uint16_t CV, uint8_t Value) {
	switch (CV) {
		case CV_VSTART:
			motorStart = Value;
			break;

		case CV_VHIGH:
			motorMax = Value;
			break;
	}
}

void notifyCVResetFactoryDefault() {
	// Make FactoryDefaultCVIndex non-zero and equal to num CV's to be reset
	// to flag to the loop() function that a reset to Factory Defaults needs to be done
	FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address
void notifyDccSpeed(uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps) {
	if (dccDirection != Dir || dccSpeed != Speed - 1 || numSpeedSteps != SpeedSteps) {
		dccUpdated = true;
		dccDirection = Dir;
		dccSpeed = Speed - 1;
		numSpeedSteps = SpeedSteps;
	}
};

// This call-back function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState) {

	if (FuncGrp == FN_0_4) {
		// newLedState = (FuncState & FN_BIT_00) ? 1 : 0;
	}
}

// This call-back function is called whenever we receive a DCC Packet
#ifdef DEBUG_DCC_MSG
void notifyDccMsg(DCC_MSG* Msg) {
	telnet.print("notifyDccMsg: ");
	for (uint8_t i = 0; i < Msg->Size; i++) {
		telnet.print(Msg->Data[i], HEX);
		telnet.write(' ');
	}
	telnet.println();
}
#endif

// This call-back function is called by the NmraDcc library when a DCC ACK needs to be sent
// Calling this function should cause an increased 60ma current drain on the power supply for 6ms to ACK a CV Read
// So we will just turn the motor on for 8ms and then turn it off again.
void notifyCVAck(void) {
#ifdef DEBUG_DCC_ACK
	Serial.println("notifyCVAck");
#endif

	digitalWrite(MOTOR_A_PIN, HIGH);
	digitalWrite(MOTOR_B_PIN, LOW);

	delay(8);

	digitalWrite(MOTOR_A_PIN, LOW);
	digitalWrite(MOTOR_B_PIN, LOW);
}

void motorTask(void* parameter) {
	// telnet.onConnect(onTelnetConnect);
	telnet.begin(telnetPort);

#define PWM_FREQUENCY 120
#define PWM_RESOLUTION 10
#define MAX_PWM 1 << PWM_RESOLUTION

	analogWriteFrequency(PWM_FREQUENCY);
	analogWriteResolution(PWM_RESOLUTION);

	bool lowerPowerMode = false;
	bool batteryChargeMode = true;

	// Setup the Pins for the Motor H-Bridge Driver
	pinMode(MOTOR_A_PIN, OUTPUT);
	pinMode(MOTOR_B_PIN, OUTPUT);
	analogWrite(MOTOR_A_PIN, 0);
	analogWrite(MOTOR_B_PIN, 0);

	pinMode(MOTOR_EN_PIN, OUTPUT);
	digitalWrite(MOTOR_EN_PIN, HIGH);

	pinMode(VCC_RAIL_SENSE, INPUT);

	Dcc.pin(DCC_PIN, 0);

	Dcc.init(MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

	// Uncomment to force CV Reset to Factory Defaults
	notifyCVResetFactoryDefault();

	// Read the current CV values for vStart and vHigh
	motorStart = Dcc.getCV(CV_VSTART);
	motorMax = Dcc.getCV(CV_VHIGH);

	Strip.Begin();
	Strip.Show();

	uint32_t last_telemetry_checkpoint = 0;

	float volt = 0.0;
	float min_volt = 0.0;
	float powerFactor = 1.0;
	uint16_t motorPWM = 0;

	while (true) {

		volt = float(analogReadMilliVolts(VCC_RAIL_SENSE)) * VCC_RAIL_FACTOR;
		if (volt < min_volt) { min_volt = volt; }

		// if (volt > 6000.0) {
		// 	powerFactor = (12000.0 / (volt)) * 0.5 + 0.5;
		// }
		// motorPWM = uint16_t(map(float(dccSpeed) * powerFactor, 0.0, float(numSpeedSteps), 0.0, MAX_PWM));
		// motorPWM = constrain(motorPWM, 0, MAX_PWM);
		motorPWM = dccSpeed * 2 * 4;



		if (millis() - 100 > last_telemetry_checkpoint) {
			if (telnet.isConnected()) {
				telnet.printf("%0.2f, %i\n", (min_volt) / 1000, motorPWM);
			}
			last_telemetry_checkpoint = millis();
			min_volt = 16000.0;
		}

		telnet.loop();

		// You MUST call the Dcc.process() method frequently for correct library operation
		Dcc.process();

		if (dccSpeed > 0) {
			if (lowerPowerMode == false) {
				digitalWrite(MOTOR_EN_PIN, HIGH);
				lowerPowerMode = true;
				setCpuFrequencyMhz(80);
				WiFi.setTxPower(WIFI_POWER_11dBm);
				if (telnet.isConnected()) {
					telnet.println("LOW POWER MODE");
				}
			}

			if (dccDirection == DCC_DIR_FWD) {
				analogWrite(MOTOR_A_PIN, 0);
				analogWrite(MOTOR_B_PIN, motorPWM);
			} else if (dccDirection == DCC_DIR_REV) {
				analogWrite(MOTOR_A_PIN, motorPWM);
				analogWrite(MOTOR_B_PIN, 0);
			}

		} else {
			if (lowerPowerMode == true) {
				digitalWrite(MOTOR_EN_PIN, LOW);
				lowerPowerMode = false;
				setCpuFrequencyMhz(240);
				WiFi.setTxPower(WIFI_POWER_19_5dBm);
				analogWrite(MOTOR_A_PIN, 0);
				analogWrite(MOTOR_B_PIN, 0);
				if (telnet.isConnected()) {
					telnet.println("HIGH POWER MODE");
				}
			}
		}

		// Handle resetting CVs back to Factory Defaults
		if (FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
			FactoryDefaultCVIndex--;  // Decrement first as initially it is the size of the array
			Dcc.setCV(FactoryDefaultCVs[FactoryDefaultCVIndex].CV, FactoryDefaultCVs[FactoryDefaultCVIndex].Value);
		}

		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void otaTask(void* parameter) {
	ArduinoOTA.setPort(3232);
	ArduinoOTA.setHostname("NIMRS");  // Hostname for OTA updates

	// Configure OTA event callbacks
	ArduinoOTA
		.onStart([]() {
			// Stop motors and reset outputs before update
			digitalWrite(MOTOR_EN_PIN, LOW);
			analogWrite(MOTOR_A_PIN, 0);
			analogWrite(MOTOR_B_PIN, 0);

			// Visual indicator (LED) for update start
			digitalWrite(LED_INDICATOR_PIN, HIGH);

			// Determine update type
			String type;
			if (ArduinoOTA.getCommand() == U_FLASH) {
				type = "sketch";
			} else {  // U_SPIFFS
				type = "filesystem";
			}

			Serial.println("Start updating " + type);
		})
		.onEnd([]() {
			Serial.println("\nEnd");
		})
		.onProgress([](unsigned int progress, unsigned int total) {
			// Show update progress and toggle LED
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
			digitalWrite(LED_INDICATOR_PIN, !digitalRead(LED_INDICATOR_PIN));
		})
		.onError([](ota_error_t error) {
			// Handle different error cases
			Serial.printf("Error[%u]: ", error);
			if (error == OTA_AUTH_ERROR) {
				Serial.println("Auth Failed");
			} else if (error == OTA_BEGIN_ERROR) {
				Serial.println("Begin Failed");
			} else if (error == OTA_CONNECT_ERROR) {
				Serial.println("Connect Failed. Firewall Issue?");
			} else if (error == OTA_RECEIVE_ERROR) {
				Serial.println("Receive Failed");
			} else if (error == OTA_END_ERROR) {
				Serial.println("End Failed");
			}
		});

	ArduinoOTA.setTimeout(30000);  // 30-second timeout
	ArduinoOTA.begin();

	// Main OTA handling loop
	while (true) {
		ArduinoOTA.handle();
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

void setup() {
	// this resets all the addressable leds to an off state
	Strip.Begin();
	Strip.Show();

	Serial.begin(115200);

	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Set WiFi RF power output level
	WiFi.setAutoReconnect(true);
	WiFi.setSleep(false);

	pinMode(LED_INDICATOR_PIN, OUTPUT);
	digitalWrite(LED_INDICATOR_PIN, LOW);

	pinMode(SAFE_BOOT_IN_PIN, INPUT_PULLUP);
	pinMode(SAFE_BOOT_GND_PIN, OUTPUT);
	digitalWrite(SAFE_BOOT_GND_PIN, LOW);

	if (digitalRead(SAFE_BOOT_IN_PIN) == HIGH) {
		xTaskCreatePinnedToCore(otaTask, "otaTask", 10000, NULL, 5, NULL, 0);

	} else {
		xTaskCreatePinnedToCore(otaTask, "otaTask", 10000, NULL, 5, NULL, 0);

		xTaskCreatePinnedToCore(motorTask, "motorTask", 40000, NULL, 0, &motorTaskHandle, 1);
	}
}

void loop() {
	vTaskDelay(1000 / portTICK_PERIOD_MS);
}