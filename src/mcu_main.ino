/* SmartFireSensor R1 */
// Debugging
//#define __DEBUG__
//#define __TEST__
#ifdef __DEBUG__
#define debug Serial
#endif
#ifdef __TEST__
static bool f_val, abn_val, p_val;
#endif

// State define
#define INVERT_ON_STATE(n) (!(n))
typedef enum {OFF, ON} sw_state_t;

// ESP boot state
static bool esp_boot = OFF;

// Timer
#define ALARM_DURATION_TIME     3 /*******************/
#define ABNORMAL_TRANSFER_DELAY 1 /* based in minute */
#define NORMAL_TRANSFER_DELAY  10 /*******************/
HardwareTimer* periodTimer;
HardwareTimer* alarmTimer;
static volatile bool delay_start = OFF, delay_end = OFF;
static volatile bool alarm_start = OFF, alarm_end = OFF;
static volatile uint8_t delay_time;
static bool alarm_on = OFF;

// SoftwareSerial
#include <SoftwareSerial.h>
SoftwareSerial WifiSerial(10, 9); // RX, TX

// SMOKE Sensor
#include <Wire.h>
#include "MAX30105.h"
//volatile int smoke;
volatile bool smoke;

// JSON Encode & Decode
#include <ArduinoJson.h>
#define DATA_DOC_CAPACITY JSON_OBJECT_SIZE(6) + JSON_ARRAY_SIZE(2)
#define JSON_DATA_BUFF_MAX_SIZE 200
static uint8_t integrated[JSON_DATA_BUFF_MAX_SIZE];

// Uart serial data
#define SERIAL_BUFF_MAX_SIZE 200
static uint8_t serialBuff[SERIAL_BUFF_MAX_SIZE];
static uint8_t* s_data = nullptr; // Data start point
static volatile bool received = OFF;
//static volatile bool requested = false;
//static uint8_t req_data;

// Data start & end signal
typedef enum {
	STX = 0xBE, // Start data
	ETX = 0xED  // End data
} se_sign_t;

// MCU to ESP Serial OP Code
typedef enum {
	TEST   = 0x00, // Used to test
	ACK    = 0x01, // Positive ack
	REQ    = 0x02, // Request data
	ERR    = 0x03, // Error
	NAK    = 0x0E, // Negative ack
	SENSOR = 0x10, // Sensor data 
	IMAGE  = 0x20, // Image data
} op_code_t;

// MCU to ESP Serial Data Type 
typedef enum {
	NONE        = 0x00, // None state
	AP          = 0x01, // AP Mode
	STA         = 0x02, // Station Mode
	AP_STA      = 0x03, // AP & Station Mode
	MODE_MASK   = 0xFC, // Not value, used to split a mode from other data
	BOOT        = 0xF0, // Boot OK
	CAMERA_FAIL = 0xFF // Camera init fail
} cntr_sig_t;
static uint8_t mcu_mode = NONE;
static uint8_t esp_mode = NONE;
static uint8_t ack_mode = NONE;
#define BOOT_REQ_DELAY(n) ((n)*(60))
static uint16_t retry_delay = 3;

MAX30105 particleSensor;

// LED & Switch
#define GLED A2 // PA3
#define RLED D6 // PB1
#define MODESW D2 // PA12
#define TESTSW D3 // PB0
static volatile bool last=INVERT_ON_STATE(OFF), curr=INVERT_ON_STATE(OFF), ModeFlag = OFF;
static volatile bool test_on = OFF;
//boolean last=LOW, curr=LOW, ModeFlag=false;

// Alarm
#define EMERGENCY_ALARM D10

// Fire detect
bool fire_detected = false;

// GAS Sensor
#define GAS_PIN A5 // PA6
//volatile int gas;
volatile bool gas;
// #define GAS_EN D12 // PB4, unused

// NTC Thermistor : NTC-103F397F (SAMKYOUNG Ceramic)
#define THERMISTOR_PIN A1 // PA1

// PIR Sensor
volatile bool pir[2];
typedef enum {L, R} pir_dir_t;
static volatile bool pir_detected = OFF;
#define PIR_PIN_L A3 // PA4
#define PIR_PIN_R A4 // PA5

// Dust Sensor
#define no_dust 0.35  // Initialize Voltage value 0.35
// Dust Sensor Interface
#define DUST_PIN A0 // PA0
#define DUST_EN A6 // PA7

#if 0 // not used
// Variables will change:
int buttonState;             // the current reading from the input pin
int lastButtonState = LOW;   // the previous reading from the input pin

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers
#endif
//#define debug SerialUSB //Uncomment this line if you're using a SAMD21
#define toESP Serial
//#define toESP WifiSerial

// Function pre-define
size_t parseData();
void sendToESP(const char op_code, const uint8_t* send_data, size_t data_len);
void sendToESP(const char op_code, const char send_data);
size_t dataToJson();
bool mcuSetMode(const uint8_t ack_mode);
void mode_conv_init();
void ap_init(); 
void sta_init(); 
void checkModeSw();
boolean debounce(boolean last);
void period_timer();
//void alarm_timer();
bool checkAbnormal();
bool checkPir(); 
int checkSmoke();
int checkGas();
float readTemp();
float readDust();
void setup_gpio();
void setup_timer();
void alarm_sign(bool state);
void onTestSw();
void operation_test(uint8_t value);


void onTestSw() {
	test_on = ON;
}

void setup_gpio() {
	// PIR Sensor
	pinMode(PIR_PIN_L, INPUT);
	pinMode(PIR_PIN_R, INPUT);

	// LED
	pinMode(GLED, OUTPUT);
	digitalWrite(GLED, INVERT_ON_STATE(OFF));
	pinMode(RLED, OUTPUT);
	digitalWrite(RLED, INVERT_ON_STATE(OFF));

	// AP <-> Station Mode Switch 
	pinMode(MODESW, INPUT);

	// TEST Switch
	pinMode(TESTSW, INPUT);
	attachInterrupt(digitalPinToInterrupt(TESTSW), onTestSw, FALLING);
}

void setup_timer() {
	TIM_TypeDef* ins1 = TIM1;
	periodTimer = new HardwareTimer(ins1);
	periodTimer->setOverflow(1, HERTZ_FORMAT); // 1HZ
	periodTimer->attachInterrupt(period_timer);
#if 0 //Wait to remove
	TIM_TypeDef* ins2 = TIM2;
	alarmTimer = new HardwareTimer(ins2);
	alarmTimer->setOverflow(1, HERTZ_FORMAT); // 1HZ
	alarmTimer->attachInterrupt(alarm_timer);
#endif
}

void setup()
{
	toESP.begin(115200);
#ifdef __DEBUG__
//	debug.begin(9600);
	debug.println(F("Smart Fire Sensor R1"));
#endif

	// Initialize sensor
	if (particleSensor.begin() == false)
	{
#ifdef __DEBUG__
		debug.println(F("MAX30105 was not found. Please check wiring/power. "));
#endif
		while (1);
	}

	particleSensor.setup(); //Configure sensor. Use 6.4mA for LED drive

	// PIN setup
	setup_gpio();

	// Timer setup
	setup_timer();
}

void loop()
{
	if (received == true) {
		size_t parse_len = parseData();
		//if (requested) { /* Data processing */ requested = false; } // Reserved
		received = false;
	}

	if (!esp_boot) {
		if (!(--retry_delay)) {
			sendToESP(REQ, BOOT);
			retry_delay = BOOT_REQ_DELAY(1);
		}
		delay(500);
		goto WAIT;
	}

#ifdef __DEBUG__
//	debug.printf("mcu: %u / esp: %u / req: %u\n", mcu_mode, esp_mode, ack_mode);
#endif

	checkModeSw();
	if (ModeFlag) {
		ack_mode = NONE;
		mode_conv_init();
		uint8_t req_mode = (mcu_mode|STA)^AP;
		sendToESP(REQ, req_mode);

		digitalWrite(GLED, INVERT_ON_STATE(ON));  
		delay(250);
		digitalWrite(GLED, INVERT_ON_STATE(OFF));
		delay(250);
		ModeFlag = OFF;
	}

	// Display current mode state
	digitalWrite(GLED, !(mcu_mode & AP));

	if (mcu_mode != ack_mode) {	if(!(mcuSetMode(ack_mode))) { goto WAIT; } }
	
	if (mcu_mode == STA) {
		bool abnormal = checkAbnormal();
		alarm_sign(abnormal);
		digitalWrite(EMERGENCY_ALARM, INVERT_ON_STATE(test_on||abnormal));
		bool pir_on = checkPir();
#ifdef __TEST__
		pir_on = p_val;
		abnormal = abn_val;
		fire_detected = f_val;
#endif
		uint8_t curr_delay = (abnormal)? ABNORMAL_TRANSFER_DELAY : NORMAL_TRANSFER_DELAY;

		if (delay_time != curr_delay) {
			delay_end = ON;
			delay_time = curr_delay;
		}
		
		if (pir_detected || ((abnormal || pir_on) && delay_end)) {
#ifdef __DEBUG__
			debug.println("send on!!!!");
#endif
			periodTimer->pause();
			delay_end = OFF;

#if 0 // wait to remove
			// Current circumstance may be emergency
			if (abnormal && !alarm_on) {
				// Alarm On
				digitalWrite(EMERGENCY_ALARM, INVERT_ON_STATE(ON));
#ifdef __DEBUG__
				debug.println("alarm on!!!!");
#endif
				alarm_on = ON;
				alarm_start = ON;
				alarmTimer->resume();
			}
#endif

			uint8_t _op_code = SENSOR;
			if (fire_detected) { _op_code |= IMAGE; }

			size_t json_len = dataToJson();
			sendToESP(_op_code, integrated, json_len);
			digitalWrite(GLED, INVERT_ON_STATE(ON)); // Transfer complete sign

			delay_start = ON;
			periodTimer->resume();
		}

		// Hardware Diagnosis
		if (test_on) {
#if 0 //Wait to remove
			alarmTimer->pause();
			// Alarm On
			digitalWrite(EMERGENCY_ALARM, INVERT_ON_STATE(ON));
#ifdef __DEBUG__
			debug.println("alarm on!!!!");
#endif
			alarm_on = ON;
			alarm_start = ON;
			alarmTimer->resume();
#endif
			size_t json_len = dataToJson();
			sendToESP((SENSOR|IMAGE), integrated, json_len);

			test_on = OFF;
		}

#if 0 //Wait to remove
		if (alarm_on && alarm_end) {
			alarmTimer->pause();
			alarm_end = OFF;
			// Alarm Off
			digitalWrite(EMERGENCY_ALARM, INVERT_ON_STATE(OFF));
			alarm_on = OFF;
		}
#endif
		// Station mode end
	} else { /* NONE & AP mode */ }

WAIT:
	delay(500);
}

#ifdef __TEST__
void operation_test(uint8_t value) {
	abn_val = value & 0x1;
	p_val = value & 0x2;
	f_val = value & 0x4;
}
#endif

void alarm_sign(bool state) {
	static bool toggle = INVERT_ON_STATE(OFF);
	toggle = (state)? !toggle : INVERT_ON_STATE(OFF);
	digitalWrite(RLED, toggle);   // Red LED Toggle
}

void mode_conv_init() {
	// Alarm Off
	alarm_sign(OFF);
	digitalWrite(EMERGENCY_ALARM, INVERT_ON_STATE(OFF));
	periodTimer->pause();
	delay_end = ON;

//	alarmTimer->pause();
//	alarm_on = OFF;
//	alarm_end = OFF;
}

void ap_init() {
}

void sta_init() {
}

// MCU mode setup
bool mcuSetMode(const uint8_t ack_mode) {
	if (ack_mode & AP) { ap_init(); } // AP & AP_STA
	else if (ack_mode == STA) { sta_init(); } // STA
	else { return false; } // NONE

	mcu_mode = ack_mode;
	sendToESP(ACK, mcu_mode);
	esp_mode = mcu_mode;

	return true;
}

// Received request for sensor data from MQTT server
void serialEvent() {
	size_t cnt = 0;

	memset(serialBuff, 0, SERIAL_BUFF_MAX_SIZE);
	while (toESP.available() > 0) {
		char get_data = toESP.read();
		serialBuff[cnt++] = get_data;
		if (get_data == ETX || cnt >= SERIAL_BUFF_MAX_SIZE) {
			received = true;
			return;
		}
	}
}

size_t parseData() {
	size_t cnt = 0;

	uint8_t* p_data = serialBuff;
	// Start sign search
	while (*(p_data++) != STX)
		if ((cnt++) >= SERIAL_BUFF_MAX_SIZE) { return 0; }

	uint8_t _op_code = *(p_data++);
	size_t _data_len = *(p_data++) << 8 | *(p_data++);
	if (p_data[_data_len] != ETX) { return 0; }

	s_data = p_data;

	if (_op_code & 0x0F) { // Command
		switch (_op_code) {
			case ACK: {
						  uint8_t ack_data = *s_data;
						  if (!(ack_data & MODE_MASK)) { ack_mode = ack_data; } // Mode(AP/Station) ACK
						  else if (ack_data == BOOT) {
							  digitalWrite(RLED, INVERT_ON_STATE(OFF));
							  esp_boot = ON;
							  ack_mode = STA;
						  }
						  else { /* Data ACK */ }
						  break;
					  }
			case REQ: {
						  uint8_t req_data = *s_data;
						  if (req_data == BOOT) { sendToESP(ACK, BOOT); }
						  //requested = true;
						  break;
					  }
			case ERR: {
						  uint8_t err_data = *s_data;
						  if (err_data == CAMERA_FAIL) { digitalWrite(RLED, INVERT_ON_STATE(ON)); }
						  break;
					  }
			case NAK: break;
		}
	} else if (_op_code & 0xF0) { /* Data type */ }
#ifdef __TEST__
	else { operation_test(*s_data); } // TEST
#endif

	return _data_len;
}

// Send data stream in serial to ESP32
void sendToESP(const char op_code, const uint8_t* send_data, size_t data_len) {
	char len_high = data_len >> 8;
	char len_low = data_len & 0xFF;

	toESP.write(STX);
	toESP.write(op_code);
	toESP.write(len_high);
	toESP.write(len_low);
	toESP.write(send_data, data_len);
	toESP.write(ETX);
}

// Send data in serial to ESP32
void sendToESP(const char op_code, const char send_data) {
	toESP.write(STX);
	toESP.write(op_code);
	toESP.write((uint8_t)0);
	toESP.write((uint8_t)1);
	toESP.write(send_data);
	toESP.write(ETX);
}

// Send sensor data to ESP32
size_t dataToJson() {
	StaticJsonDocument<DATA_DOC_CAPACITY> Data;
	Data["gas"] = gas; //checkGas() >= 100;
	Data["smoke"] = smoke; //checkSmoke() >= 10000;
	Data["fire"] = fire_detected; //Data["gas"] && Data["smoke"];
	JsonArray Motion = Data.createNestedArray("motion");
	Motion.add(pir[L]);
	Motion.add(pir[R]);
	Data["dust"] = readDust();
	Data["temp"] = readTemp();

	memset(integrated, 0, JSON_DATA_BUFF_MAX_SIZE);
	return serializeJson(Data, integrated);
}

void checkModeSw()
{
	curr=debounce(last);
	if (last == INVERT_ON_STATE(OFF) && curr == INVERT_ON_STATE(ON)) {
		ModeFlag = ON;
	}
	last=curr;
}

// Switch Debounce
boolean debounce(boolean last)
{
	boolean curr=digitalRead(MODESW);

	if (last!=curr) {
		delay(500);
		curr=digitalRead(MODESW);
	}

	return curr;
}

// Period timer isr
void period_timer()
{
	static uint8_t secCnt, minCnt;

	if (delay_start) {
		secCnt = 0;
		minCnt = 0;
		delay_start = OFF;
	}

	if (++secCnt == 60) {
		if (++minCnt >= delay_time) {
			delay_end = ON;
			minCnt = 0;
		}
		secCnt = 0;
	}
}

#if 0 //Wait to remove
// Alarm timer isr
void alarm_timer()
{
	static uint8_t secCnt, minCnt;

	if (alarm_start) {
		secCnt = 0;
		minCnt = 0;
		alarm_start = OFF;
	}

	if (++secCnt == 60) {
		if (++minCnt >= ALARM_DURATION_TIME) {
			alarm_end = ON;
			minCnt = 0;
		}
		secCnt = 0;
	}
}
#endif

bool checkAbnormal()
{
#if 1 // Read true or false
	gas = checkGas() >= 100;
	smoke = checkSmoke() >= 10000; 
	fire_detected = gas & smoke;

	return (gas || smoke);
#else // Read value
	gas = checkGas();
	smoke = checkSmoke(); 
	fire_detected = gas >= 100 && smoke >= 10000;

	return (gas >= 100 || smoke >= 10000)? true : false;
#endif
}

// Check PIR1, PIR2
bool checkPir() 
{
	bool pir_left = digitalRead(PIR_PIN_L);
	bool pir_right = digitalRead(PIR_PIN_R);

	bool pir_on = pir_left || pir_right;
	bool pir_trans = (pir_left != pir[L]) || (pir_right != pir[R]);
	pir_detected = (pir_on && pir_trans)? ON : OFF;

	pir[L] = pir_left;
	pir[R] = pir_right;
#ifdef __DEBUG__
	debug.print("pir1[");
	debug.print(pir[L]);
	debug.print("] pir2[");
	debug.print(pir[R]);
	debug.print("]");
	debug.println();
#endif

	return pir_on;
}

int checkSmoke()
{
	int smoke_red = particleSensor.getRed();
	int smoke_infrared = particleSensor.getIR();
	int smoke_green = particleSensor.getGreen();
	int smoke_avg = (smoke_red + smoke_infrared + smoke_green) / 3;
#if 0 //def __DEBUG__
	debug.printf("R[%d] IR[%d] G[%d] SMOKE[%d]\n",
                  smoke_red, smoke_infrared, smoke_green, smoke_avg);
#endif

	return smoke_avg;
}

int checkGas()
{
	int gas_volume = analogRead(GAS_PIN);
#if 0 //def __DEBUG__
	debug.print("GAS[");
	debug.print(gas_volume);  
	debug.print("]");
	debug.println();
#endif

	return gas_volume;
}

float readTemp()
{
	int Vo = analogRead(THERMISTOR_PIN);  
	float R1 = 10000;
	float R2 = R1 * (1023.0 / (float)Vo - 1.0);  
	float logR2 = log(R2);  
	float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;
	float T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));  
	float Tc = T - 273.15;  // valid value
#if 0 //def __DEBUG__
	debug.print("Temperature: ");
	debug.print(Tc);  
	debug.println(" C");
	debug.println();
#endif

	return Tc;
}
#if 0 // not used
// DUST Sensor
float get_voltage(float value)
{
	// 아날로그 값을 전압 값으로 바꿈
	float V= value * 5.0 / 1024; 
	return V;
}

float get_dust_density(float voltage)
{
	// 데이터 시트에 있는 미세 먼지 농도(ug) 공식 기준
	float dust=(voltage-no_dust) / 0.005;
	return dust;
}
#endif
float readDust()
{
	digitalWrite(DUST_EN, INVERT_ON_STATE(ON));
	delayMicroseconds(280);
	float Vo_value = analogRead(DUST_PIN); // Read Dust Sensor
	float sensor_voltage = Vo_value*(5.0/1024); // Read Sensor Voltage
	delayMicroseconds(9680);
	digitalWrite(DUST_EN, INVERT_ON_STATE(ON));
	delayMicroseconds(9680);

	// 미세 먼지 밀도
	float dust_density = (0.17*sensor_voltage-0.1)*1000;  // converter voltage to dust ub/m3
#if 0 //def __DEBUG__
	debug.print("DUST(ug/m3) ");
//	debug.print(Vo_value);  
//	debug.print(" ");
//	debug.print(sensor_voltage);  
//	debug.print(" ");
	debug.print(dust_density);  // valid value
	debug.println();
#endif

	return dust_density;
}
