#include <Arduino.h>
#include <ArduinoOTA.h>
#include <Preferences.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#include <ESPTelnet.h>
#include <NeoPixelBus.h>
#include <NmraDcc.h>

#include "secrets.h"

TaskHandle_t motorTaskHandle;

// #define DEBUG_DCC_MSG
// #define DEBUG_DCC_ACK

const uint16_t PixelCount = 8;	// this example assumes 8 pixels
const uint8_t PixelPin = 13;	// make sure to set this to the correct pin
NeoPixelBus<NeoGrbFeature, NeoWs2812xMethod> Strip(PixelCount, PixelPin);

ESPTelnet telnet;
IPAddress ip;
uint16_t telnetPort = 23;

// This is the default DCC Address
#define DEFAULT_DECODER_ADDRESS 26

#define DCC_PIN 23

#define LED_INDICATOR_PIN 16

#define MOTOR_EN_PIN 25
#define MOTOR_A_PIN 26
#define MOTOR_B_PIN 27

#define SAFE_BOOT_IN_PIN 2
#define SAFE_BOOT_GND_PIN 15

#define VCC_RAIL_SENSE 36
#define VCC_RAIL_FACTOR (33000.0 + 10000.0) / 10000.0  // Check resistor values: (R1+R2)/R2

bool dccUpdated = true;
uint8_t dccDirection = DCC_DIR_FWD;
uint8_t dccSpeed = 0;					 // Speed step 0..N-1 (e.g., 0..127 for 128 steps)
uint8_t numSpeedSteps = SPEED_STEP_128;	 // Default to 128 steps

#define PWM_FREQUENCY 20000
#define PWM_RESOLUTION 10
#define MAX_PWM ((1 << PWM_RESOLUTION) - 1)	 // 1023 for 10-bit resolution

// Use float for CVs mapped to PWM range for better precision in calculations
float cvMinSpeed = 0.0;
float cvMaxSpeed = float(MAX_PWM);	// Default max speed to full PWM range

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
	{ CV_MULTIFUNCTION_PRIMARY_ADDRESS, DEFAULT_DECODER_ADDRESS },

	{ CV_VSTART, 150 },	 // of 255, adjust as needed for your motor
	{ CV_VHIGH, 255 },	 // of 255, adjust as needed for your motor

	{ CV_MULTIFUNCTION_EXTENDED_ADDRESS_MSB,
	  CALC_MULTIFUNCTION_EXTENDED_ADDRESS_MSB(DEFAULT_DECODER_ADDRESS) },
	{ CV_MULTIFUNCTION_EXTENDED_ADDRESS_LSB,
	  CALC_MULTIFUNCTION_EXTENDED_ADDRESS_LSB(DEFAULT_DECODER_ADDRESS) },

	// Ensure CV29 matches decoder capabilities and desired operation
	// Long Address, 28/128 Speed Steps, F0 Location bit (for lights)
	{ CV_29_CONFIG, CV29_EXT_ADDRESSING | CV29_F0_LOCATION },
};

NmraDcc Dcc;

uint8_t FactoryDefaultCVIndex = 0;

// --- Helper function to map a 8bit value (0-255) to PWM range (0-MAX_PWM) float ---
float map8bitToPwm(uint8_t value) {
	// Map linearly from 8bit range (0-255) to PWM range (0-MAX_PWM)
	return map(float(value), 0.0, 255.0, 0.0, float(MAX_PWM));
}

// This call-back function is called when a CV Value changes so we can update CVs we're using
void notifyCVChange(uint16_t CV, uint8_t Value) {
	telnet.printf("Received CV %d change to %d\n", CV, Value);
	switch (CV) {
		case CV_VSTART:
			cvMinSpeed = map8bitToPwm(Value);
			telnet.printf(" -> cvMinSpeed updated to: %.2f\n", cvMinSpeed);
			break;
		case CV_VHIGH:
			cvMaxSpeed = map8bitToPwm(Value);
			// Ensure min speed is not greater than max speed
			if (cvMinSpeed > cvMaxSpeed) {
				cvMinSpeed = cvMaxSpeed - 1.0;
				telnet.printf(" -> cvMaxSpeed updated to: %.2f (Adjusted cvMinSpeed to match)\n", cvMaxSpeed);
			} else {
				telnet.printf(" -> cvMaxSpeed updated to: %.2f\n", cvMaxSpeed);
			}
			break;
		default: telnet.printf(" -> CV %d not supported yet...\n", CV); break;
	}
}

void notifyCVResetFactoryDefault() {
	telnet.println("CV Factory Reset Requested");
	FactoryDefaultCVIndex = sizeof(FactoryDefaultCVs) / sizeof(CVPair);
};

// This call-back function is called whenever we receive a DCC Speed packet for our address
void notifyDccSpeed(
	uint16_t Addr, DCC_ADDR_TYPE AddrType, uint8_t Speed, DCC_DIRECTION Dir, DCC_SPEED_STEPS SpeedSteps) {

	// NmraDcc library gives Speed=0 for Estop, Speed=1 for Stop, Speed=2..N for actual steps
	// We want our dccSpeed variable to be 0 for stop, 1..N-1 for steps
	uint8_t correctedSpeed = 0;
	if (Speed >= 2) {
		correctedSpeed = Speed - 1;	 // Map 2..N -> 1..N-1
	} else {
		correctedSpeed = 0;	 // E-Stop (0) and Stop (1) both map to 0
	}

	// Check if anything changed
	if (dccDirection != Dir || dccSpeed != correctedSpeed || numSpeedSteps != SpeedSteps) {
		telnet.printf(
			"DCC Speed Update: Speed=%d->%d, Dir=%d, Steps=%d\n", Speed, correctedSpeed, Dir, SpeedSteps);
		dccUpdated = true;
		dccDirection = Dir;
		dccSpeed = correctedSpeed;	// Store 0 for stop, 1 to MaxSteps-1 for move
		numSpeedSteps = SpeedSteps;
	}
};

// This call-back function is called whenever we receive a DCC Function packet for our address
void notifyDccFunc(uint16_t Addr, DCC_ADDR_TYPE AddrType, FN_GROUP FuncGrp, uint8_t FuncState) {
	// // Example: Control onboard LED with F0
	// if (FuncGrp == FN_0_4) {
	// 	if (FuncState & FN_BIT_00) {				// Check F0 state
	// 		digitalWrite(LED_INDICATOR_PIN, HIGH);	// Turn on LED if F0 is ON
	// 		telnet.println("F0 ON");
	// 	} else {
	// 		digitalWrite(LED_INDICATOR_PIN, LOW);  // Turn off LED if F0 is OFF
	// 		telnet.println("F0 OFF");
	// 	}
	// 	// Add other function controls here (F1-F4)
	// } else if (FuncGrp == FN_5_8) {
	// 	// Handle F5-F8
	// } else if (FuncGrp == FN_9_12) {
	// 	// Handle F9-F12
	// }
	// // Add more groups as needed
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
void notifyCVAck(void) {
#ifdef DEBUG_DCC_ACK
	telnet.println("notifyCVAck: Pulsing motor");
#endif
	// Ensure motor enable pin is high temporarily if needed, depends on H-bridge logic
	digitalWrite(MOTOR_EN_PIN, HIGH);  // Ensure driver is enabled for ACK pulse

	// Pulse the motor briefly (this assumes H-bridge pins are setup for PWM)
	// Apply a noticeable PWM value
	uint16_t ackPulsePwm = (cvMinSpeed > 50) ? uint16_t(cvMinSpeed) : 100;	// Use min speed or a fixed value
	ledcWrite(0, ackPulsePwm);												// Pulse one direction
	ledcWrite(1, 0);

	delay(8);  // Hold pulse for ~8ms

	// Turn off PWM channels and disable driver (if it wasn't already enabled)
	ledcWrite(0, 0);
	ledcWrite(1, 0);
	// The main loop will re-enable the motor driver based on dccSpeed later
	digitalWrite(MOTOR_EN_PIN, LOW);  // Turn off driver immediately after ACK

#ifdef DEBUG_DCC_ACK
	telnet.println("notifyCVAck: Pulse complete");
#endif
}

void onTelnetConnect(String ip) {
	telnet.println("\n--- NIMRS Decoder ---");
	telnet.printf("Initial CVs -> VStart(PWM): %.2f, VHigh(PWM): %.2f\n", cvMinSpeed, cvMaxSpeed);
}

void motorTask(void* parameter) {
	telnet.onConnect(onTelnetConnect);
	telnet.begin(telnetPort);

	pinMode(MOTOR_EN_PIN, OUTPUT);
	digitalWrite(MOTOR_EN_PIN, LOW);  // Start with motor driver disabled

	ledcSetup(0, PWM_FREQUENCY, PWM_RESOLUTION);
	ledcSetup(1, PWM_FREQUENCY, PWM_RESOLUTION);

	ledcAttachPin(MOTOR_A_PIN, 0);
	ledcAttachPin(MOTOR_B_PIN, 1);

	pinMode(VCC_RAIL_SENSE, INPUT);

	// --- Initialize DCC ---
	Dcc.pin(DCC_PIN, 0);  // DCC signal pin, no inversion
	// Minimal flags, let CV29 control address mode etc.
	// FLAGS_AUTO_FACTORY_DEFAULT allows reset via broadcast. Add others if needed.
	Dcc.init(MAN_ID_DIY, 10, FLAGS_MY_ADDRESS_ONLY | FLAGS_AUTO_FACTORY_DEFAULT, 0);

	// Uncomment to force CV Reset to Factory Defaults on boot
	notifyCVResetFactoryDefault();

	// Read the initial CV values and map them to the PWM range
	cvMinSpeed = map8bitToPwm(Dcc.getCV(CV_VSTART));
	cvMaxSpeed = map8bitToPwm(Dcc.getCV(CV_VHIGH));

	// --- Initialize NeoPixels ---
	Strip.Begin();
	Strip.Show();  // Initialize all pixels to 'off'

	// --- Task Variables ---
	uint32_t last_telemetry_checkpoint = 0;
	uint32_t last_PWM_update = 0;
	float avgVolt = 12000.0;  // Initialize with a reasonable guess
	float minVolt = 16000.0;

	float smoothedPWM = 0.0;  // Current smoothed PWM value
	float targetPWM = 0.0;	  // Target PWM value based on dccSpeed
	float outputPWM = 0.0;
	uint16_t outputIntPWM = 0;

	bool motorEnabled = false;	// Track if MOTOR_EN_PIN is HIGH

	// --- Main Motor Control Loop ---
	while (true) {

		// --- Voltage Sensing ---
		float volt = float(analogReadMilliVolts(VCC_RAIL_SENSE)) * VCC_RAIL_FACTOR;
		minVolt = volt < minVolt ? volt : minVolt;
		avgVolt = (avgVolt * 0.99) + (volt * 0.01);	 // Simple IIR filter (aka Exponential smoothing)

		if (last_PWM_update < millis() - 25) {
			// --- Calculate Target PWM ---
			// Recalculate targetPWM if DCC speed/direction/steps changed
			if (dccUpdated) {
				targetPWM = map(float(dccSpeed), 1.0, float(numSpeedSteps), cvMinSpeed, cvMaxSpeed);
				// targetPWM = constrain(targetPWM, cvMinSpeed, cvMaxSpeed);
				// telnet.printf("DCC Updated: Recalculated Target PWM = %.2f\n", targetPWM); // Debug
				dccUpdated = false;	 // Reset flag after recalculating
			}

			// --- Smooth Motor PWM ---
			// Apply smoothing constantly to ramp up/down
			smoothedPWM = (smoothedPWM * 0.95) + (targetPWM * 0.05);  // Adjust smoothing factor as needed

			// --- Apply Stall Prevention Logic ---
			outputPWM = smoothedPWM;
			// If smoothedPWM is moving towards 0 and is between 0 and cvMinSpeed, snap it to 0.
			// If smoothedPWM is moving towards a non-zero target and is between 0 and cvMinSpeed, snap it UP to cvMinSpeed.
			if (outputPWM > 0 && outputPWM <= cvMinSpeed) {
				if (targetPWM > cvMinSpeed) {  // If the target is to run
					outputPWM = cvMinSpeed;	   // Snap up to minimum running speed
				} else {					   // If the target is 0 (stopping)
					outputPWM = 0;			   // Snap down to 0
				}
			}

			// Final quantized PWM value for the hardware
			outputIntPWM = uint16_t(outputPWM);

			// --- Motor Control Logic ---
			if (outputIntPWM > 0) {
				// Enable Motor Driver if it's not already enabled
				if (!motorEnabled) {
					digitalWrite(MOTOR_EN_PIN, HIGH);
					motorEnabled = true;
					// setCpuFrequencyMhz(80);
					// WiFi.setTxPower(WIFI_POWER_11dBm);
					// if (telnet.isConnected()) {
					// 	telnet.println("MOTOR ENABLED");
					// }
				}

				// Set PWM based on direction
				if (dccDirection == DCC_DIR_FWD) {
					ledcWrite(0, 0);			 // Motor A off
					ledcWrite(1, outputIntPWM);	 // Motor B PWM
				} else {						 // DCC_DIR_REV
					ledcWrite(0, outputIntPWM);	 // Motor A PWM
					ledcWrite(1, 0);			 // Motor B off
				}
			} else {  // outputIntPWM == 0
				// Disable Motor Driver if it's not already disabled
				if (motorEnabled) {
					ledcWrite(0, 0);
					ledcWrite(1, 0);
					digitalWrite(MOTOR_EN_PIN, LOW);
					motorEnabled = false;
					// setCpuFrequencyMhz(240);
					// WiFi.setTxPower(WIFI_POWER_19_5dBm);
					// if (telnet.isConnected()) {
					// 	telnet.println("MOTOR DISABLED");
					// }
				}
			}
			last_PWM_update = millis();
		}

		// --- Send Telemetry Periodically ---
		if (millis() - 100 > last_telemetry_checkpoint) {
			if (telnet.isConnected()) {
				telnet.printf(
					"Vmin:%.1fV Vavg:%.1f DCC:%d Target:%.0f(%.0f%%) Smooth:%.0f(%.0f%%) Out:%d(%.0f%%) "
					"Min:%.0f Max:%.0f MCU Temp:%0.1f°C\n",
					minVolt / 1000.0,
					avgVolt / 1000.0,
					dccSpeed,
					targetPWM,
					(targetPWM / float(MAX_PWM)) * 100.0,
					smoothedPWM,
					(smoothedPWM / float(MAX_PWM)) * 100.0,
					outputIntPWM,
					(float(outputIntPWM) / float(MAX_PWM)) * 100.0,
					cvMinSpeed,
					cvMaxSpeed,
					temperatureRead());
			}
			last_telemetry_checkpoint = millis();
			minVolt = 16000.0;	// Reset min voltage for next interval
		}

		// You MUST call the Dcc.process() method frequently for correct library operation
		Dcc.process();

		// --- Handle CV Reset ---
		// Handle resetting CVs back to Factory Defaults (if requested by notifyCVResetFactoryDefault)
		if (FactoryDefaultCVIndex && Dcc.isSetCVReady()) {
			FactoryDefaultCVIndex--;  // Decrement first as initially it is the size of the array
			uint16_t cvToSet = FactoryDefaultCVs[FactoryDefaultCVIndex].CV;
			uint8_t valToSet = FactoryDefaultCVs[FactoryDefaultCVIndex].Value;
			telnet.printf("Resetting CV %d to %d\n", cvToSet, valToSet);
			Dcc.setCV(cvToSet, valToSet);
			// We don't need to call notifyCVChange here, NmraDcc will do that automatically
			// if the setCV is successful and the value actually changes internally.
		}

		// --- Telnet Loop ---
		telnet.loop();	// Handle Telnet connections and input

		// --- Task Delay ---
		// Yield for other tasks. 1ms is frequent enough for receiving DCC commands.
		vTaskDelay(pdMS_TO_TICKS(1));
	}  // End while(true)
}

// --- otaTask ---
void otaTask(void* parameter) {
	ArduinoOTA.setPort(3232);
	ArduinoOTA.setHostname("NIMRS");  // Hostname for OTA updates

	ArduinoOTA
		.onStart([]() {
			// vTaskSuspend(motorTaskHandle);		// Suspend motor task during OTA (DO NOT ENABLE)
			digitalWrite(MOTOR_EN_PIN, LOW);		// Ensure motor is off
			digitalWrite(LED_INDICATOR_PIN, HIGH);	// LED solid on during update

			String type;
			if (ArduinoOTA.getCommand() == U_FLASH) {
				type = "sketch";
			} else {  // U_SPIFFS
				type = "filesystem";
			}
			Serial.println("Start updating " + type);
			//telnet.println("Starting OTA update...");  // Notify telnet users
			// telnet.stop();							   // Stop telnet server
		})
		.onEnd([]() {
			Serial.println("\nEnd");
			// digitalWrite(LED_INDICATOR_PIN, LOW);  // Turn off LED
			// Restart occurs automatically after successful firmware update
		})
		.onProgress([](unsigned int progress, unsigned int total) {
			// Show update progress and toggle LED
			Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
			digitalWrite(LED_INDICATOR_PIN, !digitalRead(LED_INDICATOR_PIN));
		})
		.onError([](ota_error_t error) {
			Serial.printf("Error[%u]: ", error);
			if (error == OTA_AUTH_ERROR)
				Serial.println("Auth Failed");
			else if (error == OTA_BEGIN_ERROR)
				Serial.println("Begin Failed");
			else if (error == OTA_CONNECT_ERROR)
				Serial.println("Connect Failed. Firewall Issue?");
			else if (error == OTA_RECEIVE_ERROR)
				Serial.println("Receive Failed");
			else if (error == OTA_END_ERROR)
				Serial.println("End Failed");
		});

	ArduinoOTA.setTimeout(30000);  // 30-second timeout
	ArduinoOTA.begin();
	Serial.println("OTA Ready");

	while (true) {
		ArduinoOTA.handle();
		vTaskDelay(pdMS_TO_TICKS(1));
	}
}

// --- setup (Minor adjustments) ---
void setup() {
	Serial.begin(115200);
	Serial.println("\nBooting NIMRS Decoder...");

	// --- Initialize LED ---
	pinMode(LED_INDICATOR_PIN, OUTPUT);
	digitalWrite(LED_INDICATOR_PIN, LOW);  // LED off initially

	// Initialize NeoPixels early to turn them off
	Strip.Begin();
	Strip.Show();

	// --- WiFi Setup ---
	WiFi.mode(WIFI_STA);
	WiFi.begin(ssid, password);
	WiFi.setTxPower(WIFI_POWER_19_5dBm);  // Set desired WiFi power
	WiFi.setAutoReconnect(true);
	WiFi.setSleep(false);  // Disable WiFi power save for responsiveness

	Serial.print("Connecting to WiFi ");
	int wifi_retries = 0;
	while (WiFi.status() != WL_CONNECTED && wifi_retries < 30) {  // ~15 seconds timeout
		Serial.print(".");
		digitalWrite(LED_INDICATOR_PIN, !digitalRead(LED_INDICATOR_PIN));  // Blink LED during connection
		delay(500);
		wifi_retries++;
	}
	digitalWrite(LED_INDICATOR_PIN, LOW);  // LED off after connection attempt

	if (WiFi.status() == WL_CONNECTED) {
		Serial.println(" Connected!");
		Serial.print("IP Address: ");
		ip = WiFi.localIP();  // Store IP address
		Serial.println(ip);
	} else {
		Serial.println(" Connection Failed!");
		// Perhaps enter a fallback mode or just continue without WiFi features fully working
	}

	// --- Task Creation ---
	// Always create OTA task
	xTaskCreatePinnedToCore(
		otaTask,   /* Task function. */
		"otaTask", /* name of task. */
		10000,	   /* Stack size of task */
		NULL,	   /* parameter of the task */
		5,		   /* priority of the task */
		NULL,	   /* Task handle to keep track of created task */
		0);		   /* pin task to core 0 */
	Serial.println("OTA Task Started.");

	// Check safe boot pin *after* basic setup
	pinMode(SAFE_BOOT_IN_PIN, INPUT_PULLUP);
	pinMode(SAFE_BOOT_GND_PIN, OUTPUT);
	digitalWrite(SAFE_BOOT_GND_PIN, LOW);

	if (digitalRead(SAFE_BOOT_IN_PIN) == LOW) {
		Serial.println("Safe Boot Pin LOW - Starting Motor Task.");
		// Create Motor task only if not in safe boot mode
		xTaskCreatePinnedToCore(
			motorTask,		  /* Task function. */
			"motorTask",	  /* name of task. */
			40000,			  /* Stack size of task */
			NULL,			  /* parameter of the task */
			0,				  /* priority of the task (lower than OTA) */
			&motorTaskHandle, /* Task handle to keep track of created task */
			1);				  /* pin task to core 1 */
		Serial.println("Motor Task Started.");
	} else {
		Serial.println("Safe Boot Pin HIGH - Skipping Motor Task.");
		while (true) {
			digitalWrite(LED_INDICATOR_PIN, !digitalRead(LED_INDICATOR_PIN));
			vTaskDelay(pdMS_TO_TICKS(100));
		}
	}
}

// --- loop (Keep empty as using FreeRTOS tasks) ---
void loop() {
	// Empty. Everything is handled in tasks.
	vTaskSuspend(NULL);	 // Suspend the loop() task itself
}
