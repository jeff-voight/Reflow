/*
 Name:		Reflow.ino
 Created:	6/15/2016 4:00:12 PM
 Author:	jeff
*/
//#include <gfxfont.h>
//#include <Adafruit_GFX.h>
#include <max6675.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_BME280.h>
#include <avr/pgmspace.h>

#ifndef _swap_int16_t
#define _swap_int16_t(a, b) { int16_t t = a; a = b; b = t; }
#endif

#define HEATER_TOP_PIN 2
#define HEATER_BOTTOM_PIN 3
#define PROFILE_SELECT_BUTTON 4
#define START_STOP_BUTTON 5
#define CPU_FAN_CTL_PIN 6
#define CIRC_FAN_CTL_PIN 7
#define HEATER_TOP_LED_PIN 8
#define HEATER_BOTTOM_LED_PIN 9
#define BUZZER_PIN 10
#define SCK 11
#define CS 12
#define MISO 13
#define OLED_RESET 4
#define DEBOUNCE_TIME 50
#define CPU_TEMP_SAMPLE_TIME 60 * 1000 // seconds
#define BUZZER_DURATION 3 * 1000
#define nextStageTime profileTemp[profile][stage][0]
#define nextStageTemp  profileTemp[profile][stage][1]
#define x01 16
#define y01 56
#define x11 128
#define y11 16 // Graph area

Adafruit_SSD1306 ssd = Adafruit_SSD1306(OLED_RESET);
Adafruit_BME280 bme;
MAX6675 thermA(SCK, CS, MISO);

const char profileName[2][7] PROGMEM = { {"Leaded"}, {"LeadFr"} };
const char profileStage[6][8] PROGMEM = { {"Off"}, {"Preheat"}, {"Soak"}, {"Heat"}, {"Liquid"}, {"Cool"} };

bool heaterTopOn = false, heaterBottomOn = false,
heaterTopLedOn = false, heaterBottomLedOn = false, buzzerOn = false;
uint8_t cpuTemperature;
uint16_t probeATemperature = 0, probeBTemperature = 0;
uint8_t profile = 0;
uint16_t cookTime;
unsigned long startTime = millis();
uint8_t stage = 0; // 0=off, 1=preheat, 2=soak, 3=heating, 4=liquid, 5=cooldown

uint8_t circFanSpeed = 0, cpuFanSpeed = 0;



uint16_t profileTemp[2][6][2] = {
	{
		{0, 0},     // Off
		{90, 150}, // Preheat
		{180, 150}, // Soak
		{220, 250}, // heating
		{260, 250}, // liquid
		{300, 0}    // Cooldown
	},
	{
		{0,0},
		{90, 150},
		{180, 150},
		{225, 250},
		{255, 250},
		{300, 0}

	}
};
unsigned long buzzerTime = millis();

// the setup function runs once when you press reset or power the board
void setup() {
	bme.begin(0x76);
	cpuTemperature = bme.readTemperature();
	ssd.begin(SSD1306_SWITCHCAPVCC);
	ssd.setTextColor(WHITE);
	ssd.setTextSize(1);
	ssd.clearDisplay();
	ssd.display();
	Serial.begin(115200);
	for (int i = 0; i < 6; i++) {
		Serial.println(profileStage[i]);
	}

	pinMode(HEATER_TOP_PIN, OUTPUT);
	digitalWrite(HEATER_TOP_PIN, HIGH);
	pinMode(HEATER_BOTTOM_PIN, OUTPUT);
	digitalWrite(HEATER_BOTTOM_PIN, HIGH);
	pinMode(PROFILE_SELECT_BUTTON, INPUT_PULLUP);
	pinMode(START_STOP_BUTTON, INPUT_PULLUP);
	pinMode(CPU_FAN_CTL_PIN, OUTPUT);
	pinMode(CIRC_FAN_CTL_PIN, OUTPUT);
	pinMode(HEATER_TOP_LED_PIN, OUTPUT);
	pinMode(HEATER_BOTTOM_LED_PIN, OUTPUT);
	pinMode(BUZZER_PIN, OUTPUT);
	digitalWrite(BUZZER_PIN, 0);
	digitalWrite(START_STOP_BUTTON, HIGH);
	digitalWrite(PROFILE_SELECT_BUTTON, HIGH);
}

// the loop function runs over and over again until power down or reset
void loop() {
	cpuTemperature = bme.readTemperature();
	if (cpuTemperature > 40) {
		cpuFanSpeed = 255;
	} else {
		cpuFanSpeed = 0;
	}
	//cpuFanSpeed = 127;
	//circFanSpeed = 255;
	probeATemperature = thermA.readCelsius();
	// probeB Temperature Reading
	if (stage == 0) {
		cookTime = 0;
		heaterTopOn = false;
		heaterBottomOn = false;
	} else {
		cookTime = (millis() - startTime) / 1000;
		if (cookTime > nextStageTime) {
			incrementStage();
		}

		if (getProbeTemp() < nextStageTemp) {
			applyHeat();
		} else if (getProbeTemp() > nextStageTemp) { // No buffer for over-temp condition
			if (heaterTopOn) {
				heaterTopOn = false;
			} else {
				heaterBottomOn = false;
			}
		}
		if (abs(probeATemperature - probeBTemperature) > 3) {
			circFanSpeed = 127;
		} else {
			circFanSpeed = 0;
		}
	}

	if (buzzerOn) {
		soundAlarm();
	}

	setHeater(HEATER_TOP_PIN, heaterTopOn);
	setHeater(HEATER_BOTTOM_PIN, heaterBottomOn);
	setLed(HEATER_TOP_LED_PIN, heaterTopOn);
	setLed(HEATER_BOTTOM_LED_PIN, heaterBottomOn);
	setFan(CIRC_FAN_CTL_PIN, circFanSpeed);
	setFan(CPU_FAN_CTL_PIN, cpuFanSpeed);
	bool start = debounce(START_STOP_BUTTON);
	if (start) {
		if (stage == 0) {
			stage = 1;
			char stageString[8];
			strcpy_P(stageString, profileStage[stage]);
			startTime = millis();
			Serial.println(stage);
			Serial.println(stageString);
			Serial.println(nextStageTime);
			Serial.println(nextStageTemp);
		}
		heaterTopOn = !heaterTopOn;
	}

	bool profileButton = debounce(PROFILE_SELECT_BUTTON);
	if (profileButton) {
		heaterBottomOn = !heaterBottomOn;
		if (stage == 0) {
			profile = !profile;
		}
	}
	displayGraph();
	delay(200);
}

void soundAlarm() {
	if (millis() > buzzerTime + BUZZER_DURATION) {
		buzzerOn = false;
		analogWrite(BUZZER_PIN, 0);
	} else {
		analogWrite(BUZZER_PIN, 127);
	}
}
/**
	WARNING: This method assumes that we are not at stage 0 because
	stage 0 should never be heating. It also assumes that we are either
	trying to maintain a temperature or we're trying to raise it.
	This method will NOT work for cooling.
	Probably.
*/
void applyHeat() {
	// TODO: replace with PID loop. Detect how far off graph we are before broiling.
	if (heaterBottomOn) { // Heater is already on an we're below temp
		if (abs(nextStageTemp - getProbeTemp()) > 5) { // WAY out of range.
			heaterTopOn = true;
		}
	} else { // Bottom heater isn't on. Turn it on
		heaterBottomOn = true;
		heaterTopOn = false;
	}
	/*if (nextStageTemp > profileTemp[profile][stage - 1][0]) {
		heaterTopOn = true;
		heaterBottomOn = true;
	} else {
		heaterBottomOn = true;
	}
	*/

}
void incrementStage() {
	stage++;
	if (stage == 5) {
		heaterTopOn = false;
		heaterBottomOn = false;
		buzzerOn = true;
		buzzerTime = millis();
		circFanSpeed = 127;
	}
	if (stage > 5) { // TODO: Verify enough time has passed in liquid stage
		stage = 0;
		Serial.println(F("Finished!*********"));
		circFanSpeed = 0;
	} else {
		Serial.println(stage);
		char stageString[8];
		strcpy_P(stageString, profileStage[stage]);
		Serial.println(stageString);
		Serial.println(nextStageTime);
		Serial.println(nextStageTemp);
	}


}

/**
	Enables or disables the specified heater. Relay pin LOW = ON.
*/
void setHeater(uint8_t _heater, bool _state) {
	if (_state) {
		//Serial.print(F("Turning heater ")); Serial.print(_heater); Serial.println(F(" on."));
		digitalWrite(_heater, LOW);
	} else {
		digitalWrite(_heater, HIGH);
	}
}

/**
	Enables or disables the LED indicator
*/
void setLed(uint8_t _led, bool _state) {
	if (_state) {
		digitalWrite(_led, HIGH);
	} else {
		digitalWrite(_led, LOW);
	}
}

/**
	Enables and adjusts the selected fan speed
*/
void setFan(uint8_t _fan, uint8_t _speed) {
	analogWrite(_fan, _speed);
}

/**
	Debounces pin by testing for LOW, waiting for a delay and testing
	again. If both tests are LOW, the button was pushed.
*/
bool debounce(uint8_t _pin) {
	bool returnValue = digitalRead(_pin) == LOW;
	if (returnValue) {
		delay(DEBOUNCE_TIME);
		returnValue = digitalRead(_pin) == returnValue;
	}
	return returnValue;
}

void displayGraph() {
	ssd.clearDisplay();
	char profileN[7];
	strcpy_P(profileN, profileName[profile]);
	char profileS[8];
	strcpy_P(profileS, profileStage[stage]);
	ssd.setCursor(0, 0);
	ssd.print(profileN);
	ssd.setCursor(38, 0);
	ssd.print(getProbeTemp());
	ssd.setCursor(60, 0);
	ssd.print(F("CPU"));
	ssd.setCursor(82, 0);
	ssd.print(cpuTemperature);
	ssd.setCursor(105, 0);
	ssd.print(cookTime);
	ssd.setCursor(0, 8);
	ssd.print(F("Target"));
	ssd.setCursor(38, 8);
	ssd.print(nextStageTemp);
	ssd.setCursor(60, 8);
	ssd.print(profileS);
	ssd.setCursor(0, 16);
	ssd.print(F("250"));
	ssd.setCursor(0, 24);
	ssd.print(F("200"));
	ssd.setCursor(0, 32);
	ssd.print(F("150"));
	ssd.setCursor(0, 40);
	ssd.print(F("100"));
	ssd.setCursor(5, 48);
	ssd.print(F("50"));
	ssd.setCursor(10, 56);
	ssd.print(F("0"));
	ssd.setCursor(33, 56);
	ssd.print(F("75"));
	ssd.setCursor(52, 56);
	ssd.print(F("150"));
	ssd.setCursor(80, 56);
	ssd.print(F("225"));
	ssd.setCursor(107, 56);
	ssd.print(F("300"));
	buildGraph();
	ssd.display();

}

void buildGraph() {
	// draw Frame
	ssd.drawFastHLine(x01, y01, x11 - x01, WHITE);
	ssd.drawFastVLine(x01, y11, y01 - y11, WHITE);

	// draw Cutoff Points
	for (int i = 0; i < 5; i++) {
		uint16_t timeThreshold = profileTemp[profile][i][0];
		uint16_t tempThreshold = profileTemp[profile][i][1];
		uint8_t y = getTempY(tempThreshold);
		uint8_t x = getTimeX(timeThreshold);
		drawDashedLine(x01, y, x, y);
		drawDashedLine(x, y01, x, y);
	}

	// draw Profile
	for (int i = 1; i < 6; i++) {
		uint8_t startX = getTimeX(profileTemp[profile][i - 1][0]);
		uint8_t stopX = getTimeX(profileTemp[profile][i][0]);
		uint8_t startY = getTempY(profileTemp[profile][i - 1][1]);
		uint8_t stopY = getTempY(profileTemp[profile][i][1]);
		ssd.drawLine(startX, startY, stopX, stopY, WHITE);
	}
	// Draw progress
	uint8_t currentX = getTimeX(cookTime);
	uint8_t currentY = getTempY(getProbeTemp());
	ssd.drawLine(currentX, currentY - 2, currentX, currentY + 2, WHITE);
	ssd.drawLine(currentX - 2, currentY, currentX + 2, currentY, WHITE);
}

uint8_t getTempY(uint16_t tempThreshold) {
	float percentTemp = tempThreshold / 250.0;
	uint8_t y = 56 - ((y01 - y11)*percentTemp);
	return y;
}

uint8_t getTimeX(uint16_t timeThreshold) {
	float percentTime = timeThreshold / 300.0;
	uint8_t x = 16 + (x11 - x01)*percentTime;
	return x;
}

void drawDashedLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1) {
	uint8_t color = WHITE;
	int8_t steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		_swap_int16_t(x0, y0);
		_swap_int16_t(x1, y1);
	}

	if (x0 > x1) {
		_swap_int16_t(x0, x1);
		_swap_int16_t(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	for (; x0 <= x1; x0++) {
		if ((x0 + y0) % 2) {
			color = WHITE;
		} else {
			color = BLACK;
		}
		if (steep) {
			ssd.drawPixel(y0, x0, color);
		} else {
			ssd.drawPixel(x0, y0, color);
		}
		err -= dy;
		if (err < 0) {
			y0 += ystep;
			err += dx;
		}
	}
}


uint16_t getProbeTemp() {
	return probeATemperature;// (probeATemperature + probeBTemperature) / 2;
}