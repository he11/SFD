// Standard header
#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

// Define
#define CAMERA_MODEL_AI_THINKER
#define WIFI_MAX_SIZE 30  // max 30, received ap list
#define AUTH_MAX_SIZE 20  // WiFi auth info max 20
#define WRITE_EEPROM 
#define SERIAL_BUFF_MAX_SIZE 200
//#define HTML_PAGE_MAX_SIZE 2000 // test
//#define TOTAL_WIFI_STRING_LEN 200 // test
// Button pin
#define MODE_SWITCH 13
#define toMCU Serial
#define debug Serial
//#define __TEST_SW__

// User custom & sdk header
#include "esp_camera.h"	 // Camera Module 
#include "camera_pins.h" // ---------------
//#include "esp_wifi.h" //test
#include "mqtt.h"

#if 1
// Function pre-define
void callback(char* topic, byte* payload, unsigned int length);
void setup_uart();
void setup_camera();
bool setup_wifi(wifi_mode_t mode);
void setup_mqtt();
void setup_gpio();
void setup_timer();
String wifi_list_page();
void handle_root();
void handle_credential();
bool save_credential();
#endif

// Status value
enum {
	OFF = 0,
	ON  = 1
};

// Data start & end signal
enum SE_SIGN {
	STX = 0xBE, // Start data
	ETX = 0xED  // End data
};

// MCU to ESP Serial OP Code
enum OP_CODE {
	DATA = 0x00, // Data ack
	ACK  = 0x01, // Positive ack
	REQ  = 0x02, // Request data
	NAK  = 0xFE  // Negative ack
};

// MCU to ESP Serial Data Type 
enum DATA_TYPE {
	NONE      = 0x00, // None state
	AP        = 0x01, // AP Mode
	STA       = 0x02, // Station Mode
	AP_STA    = 0x03, // AP & Station Mode
	SENSOR    = 0xF0, // Sensor data
	MODE_MASK = 0xFC  // Not value, used to split a mode from other data	
};
uint8_t esp_mode = NONE;
uint8_t mcu_mode = NONE;
uint8_t req_mode = STA;

// Uart Data buffer
uint8_t serialBuff[SERIAL_BUFF_MAX_SIZE];
uint8_t* s_data = nullptr; // Data start point
bool is_data = OFF;

// Intterrupt variable
volatile uint8_t timer_cnt;
hw_timer_t* timer = NULL;
portMUX_TYPE timer_mux = portMUX_INITIALIZER_UNLOCKED;
portMUX_TYPE ext_itrt_mux = portMUX_INITIALIZER_UNLOCKED;
// WiFi scan info
bool wifi_scan = OFF;
String wifi_list[WIFI_MAX_SIZE];
//char* wifi_list[WIFI_MAX_SIZE];
// WiFi setting info
bool wifi_conn = OFF;
// AP mode access id and password
const char* dev_ssid PROGMEM = "Telco_Smart";
const char* dev_pw PROGMEM   = "12345678";
IPAddress apIP(192, 168, 5, 1);
IPAddress apMask(255, 255, 255, 0);
// Station mode id and password
char ap_ssid[AUTH_MAX_SIZE];
char ap_pw[AUTH_MAX_SIZE];
// Web server
bool web_bind = OFF;
//String htmlPage;
//String strList;
WebServer server(80);
// MQTT Server
const char* mqtt_id PROGMEM      = "************";
const char* mqtt_pw PROGMEM      = "************";
const char* mqtt_server PROGMEM  = "192.168.0.5"; // "192.168.43.54"; 
const uint16_t mqtt_port PROGMEM = 1883;
const char* tps1 PROGMEM         = "auth";
const char* tps2 PROGMEM         = "config";
const char* tps3 PROGMEM         = "photo";
const char* tps4 PROGMEM         = "video";
const char* tps5 PROGMEM         = "sensor";
const char* sub_tps[] PROGMEM    = {tps1, tps2, tps3, tps4, tps5}; 
const char* tp_photo PROGMEM     = "picture";
const char* tp_video PROGMEM     = "stream";
const char* tp_sensor PROGMEM    = "data";
const uint8_t sub_tp_cnt = sizeof(sub_tps)/sizeof(const char*);
WiFiClient espClient;
MQTT client(espClient, callback);

#ifdef __DEBUG__
const char* fmt1 PROGMEM = "2BPP(RGB565)";
const char* fmt2 PROGMEM = "2BPP(YUV422)";
const char* fmt3 PROGMEM = "1BPP";
const char* fmt4 PROGMEM = "JPEG";
const char* fmt5 PROGMEM = "3BPP";
const char* fmt6 PROGMEM = "RAW";
const char* fmt7 PROGMEM = "3BP2P(RGB444)";
const char* fmt8 PROGMEM = "3BP2P(RGB555)";
const char* img_fmt[] PROGMEM = {fmt1, fmt2, fmt3, fmt4, \
                                     fmt5, fmt6, fmt7, fmt8};

void printRaw(const uint8_t* buf, size_t len) {
	int cnt = 0;

	for (int i=0; i<len; i++) {
		debug.printf("%u", buf[i]);
		if ((cnt%10)==0 && cnt)
			debug.println();
		cnt++;
	}

	debug.println();
}
#endif

// Search networks in around
void scan_wifi() {
	memset(wifi_list, 0, WIFI_MAX_SIZE);
	// WiFi.scanNetworks will return the number of networks found
	int16_t netCount = WiFi.scanNetworks();
	if (!netCount) { return; }
#if 0
	uint16_t test;
	esp_wifi_scan_get_ap_num(&test);
	if (!test) { return; }
	//debug.printf("ap num: %hu\n", test);
	debug.println();

	wifi_ap_record_t* _scanResult = (wifi_ap_record_t*)new wifi_ap_record_t[test];
	if (!_scanResult) { return; }

	esp_err_t err = esp_wifi_scan_get_ap_records(&(test), _scanResult);
	if (err) { goto SCAN_END; }

	memset(wifi_list, 0, WIFI_MAX_SIZE);
	for (int i=0; i<test; i++) {
		wifi_list[i] = reinterpret_cast<const char*>((_scanResult + i)->ssid);
		debug.printf("%s\n", wifi_list[i]);
	}
#else
	for(int i=0; i<WIFI_MAX_SIZE; i++)
	{
		wifi_list[i] = WiFi.SSID(i);
	}
#endif
#if 0
SCAN_END:
	delete[] _scanResult;
#endif
}

#if 0
	//xEventGroupCreate()// -> serial interrupt available?
#endif

// Try to reconnect to the MQTT server 
void reconn() {
	// Loop until we're reconnected
	while (!client.connected()) {
		debug.print("Attempting MQTT connection...");
	// TODO: edit
		// Create a random client ID
	/*	TODO: String ---> char * change  */
		String clientId = "test-";
		clientId += String(random(0xffff), HEX);
		// Attempt to connect
		if (client.connect(clientId.c_str())) {//, mqtt_id, mqtt_pw)) {
	//
			debug.println(" connected");
			// Once connected, publish an announcement...
			//client.publish(topic_auth, "This is an auth message for the reconnection.");
			// ... and resubscribe
			client.Subscribe(sub_tps, sub_tp_cnt);
		} else {
			debug.print("failed, rc=");
			debug.print(client.state());
			debug.println(" try again in 3 seconds");
			// Wait about 3 seconds before retrying
			delay(2500);
			return;
		}
	}
}

// Timer interrupt service rootin
void IRAM_ATTR on_timer() {
	portENTER_CRITICAL_ISR(&timer_mux); // lock start
	timer_cnt++;
	portEXIT_CRITICAL_ISR(&timer_mux); // lock end
}

#ifdef __TEST_SW__ // used to test
volatile bool test_sw = OFF;
void IRAM_ATTR ap_set_callback() {
	portENTER_CRITICAL_ISR(&ext_itrt_mux);
	test_sw = ON;
	portEXIT_CRITICAL_ISR(&ext_itrt_mux);
}
#endif

#if 0 // uart interrupt
//char abnormal[];
char a;
void serialEvent() {
	if (toMCU.available() > 0) {
	//	 = toMCU.read();
		a = toMCU.read();
	}
}
#endif


// Ap mode initialize
void ap_init() {
	wifi_scan = ON;
	portENTER_CRITICAL(&timer_mux); // lock start
	timer_cnt = 0;
	portEXIT_CRITICAL(&timer_mux); // lock end
	timerRestart(timer);
}

// Station mode initialize
void sta_init() {
	timerStop(timer);
}

// Send data stream in serial to MCU
void sendToMCU(const char op_code, const uint8_t* send_data, size_t data_len) {
	uint8_t len_high = data_len >> 8;
	uint8_t len_low = data_len & 0xFF;

	toMCU.write(STX);
	toMCU.write(op_code);
	toMCU.write(len_high);
	toMCU.write(len_low);
	toMCU.write(send_data, data_len);
	toMCU.write(ETX);
}

// Send data in serial to MCU
void sendToMCU(const char op_code, const char send_data) {
	toMCU.write(STX);
	toMCU.write(op_code);
	toMCU.write(0);
	toMCU.write(1);
	toMCU.write(send_data);
	toMCU.write(ETX);
}

// Parse serial data
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

	if (!_op_code) { is_data = ON; } // DATA
	else if (_op_code == REQ) { // REQ
		if (!(*s_data & MODE_MASK)) { req_mode = *s_data; }
	} else if (_op_code == ACK) { // ACK
		if (!(*s_data & MODE_MASK)) { req_mode = *s_data; } // Mode(AP/Station) ACK
		else { /* Data ACK */ }
	} else { /* NAK */ }

	return _data_len;
}

size_t serialEvent() {
	size_t cnt = 0;

	memset(serialBuff, 0, SERIAL_BUFF_MAX_SIZE);
	while (toMCU.available() > 0) {
		char get_data = toMCU.read();
		serialBuff[cnt++] = get_data;
		if (get_data == ETX || cnt >= SERIAL_BUFF_MAX_SIZE) {
			break;
		}
	}

	return parseData();
}

// ESP mode setup
bool espSetMode(const uint8_t req_mode) {
	if (req_mode & AP) { // AP & AP_STA
		if (!setup_wifi(WIFI_MODE_APSTA)) { return false; }
		ap_init(); // AP mode setup
	} else if (req_mode == STA) { // STA
		if (!setup_wifi(WIFI_MODE_STA)) { return false; } 
		sta_init(); // Station mode setup
	} else { return false; } // NONE

	esp_mode = req_mode;
	sendToMCU(ACK, esp_mode);
	mcu_mode = esp_mode;

	return true;
}

void setup() {
	// put your setup code here, to run once:
	setup_uart();
	// Setup peripheral device
	setup_camera();
	setup_gpio();
	setup_timer();
	setup_mqtt();
//	setup_wifi(WIFI_MODE_STA);
	// randomSeed(analogRead(14)); what is analog pin?
}

void loop() {
	// put your main code here, to run repeatedly:
#ifdef __TEST_SW__
	if (test_sw == ON) {
		portENTER_CRITICAL_ISR(&ext_itrt_mux);
		test_sw = OFF;
		portEXIT_CRITICAL_ISR(&ext_itrt_mux);
	}
#endif

	size_t pub_len = serialEvent();
	if (is_data) {
		sendToMCU(ACK, SENSOR);
		mqtt_err_t err = send_sensor(s_data, pub_len);
		is_data = OFF;
	}

#ifdef __DEBUG__
//	debug.printf("esp: %u / mcu: %u / req: %u\n", esp_mode, mcu_mode, req_mode);
#endif

	// ESP Mode change occur
	if (esp_mode != req_mode) {	if(!(espSetMode(req_mode))) { goto FAIL; } }

	if (!(esp_mode & AP)) {
		// Station mode Work loop START
		if (WiFi.status() != WL_CONNECTED) {
			wifi_conn = OFF;
			debug.print(".");
			goto FAIL;
		} else if (!wifi_conn) {
			wifi_conn = ON;
			debug.println(" WiFi connected");
#ifdef __DEBUG__
			debug.print("Device IP: ");
			debug.println(WiFi.localIP());
			WiFi.printDiag(debug);
#endif
		}
#ifdef __DEBUG__
//		debug.println("loop start (in Station mode)");
#endif
		if (!client.connected()) { reconn(); }
		// To process incoming messages to send publish data and to make a refresh of the connection
		client.loop(); 
		// Station mode work loop END
	} else {
#ifdef __DEBUG__
		client.freeMemSize(__func__, __LINE__);
#endif
		/*
			TODO: Worklist
				1. AP Setting(smart config?)
												*/
		// AP mode work loop START
#ifdef __DEBUG__
//		debug.println("loop start (in AP mode)");
//		debug.printf("scan: %u / timer: %u\n", wifi_scan, timer_cnt);
#endif
		if (wifi_scan ^ (timer_cnt & 0x1)) { // about 1 minute period
			scan_wifi();
			wifi_scan = !wifi_scan;
		}

		// Client connection handle
		server.handleClient();
		// AP mode work loop END
	}

FAIL:
	delay(500);
}

// Send sensor data
mqtt_err_t send_sensor(const uint8_t* pub_data, size_t pub_len) {
	mqtt_err_t err = client.sendJson((const char*)pgm_read_ptr(&tp_sensor), pub_data, pub_len);
	if (err)
		debug.printf("Publishing data failed, err code: 0x%d\n", err);

	return !(err)? PUB_OK : PUB_FAIL;
}

// Send image data
int send_photo() {
	camera_fb_t * fb = NULL;

	fb = esp_camera_fb_get();
	if (!fb) {
		debug.println("Camera capture failed");
		return ESP_FAIL;
	}

	mqtt_err_t err = client.sendBinary((const char*)pgm_read_ptr(&tp_photo), fb->buf, fb->len);
	if (err)
		debug.printf("Publishing data failed, err code: 0x%d\n", err);
#ifdef __DEBUG__
//	const char* _data_fmt = (const char*)pgm_read_ptr(img_fmt + fb->format);
//	debug.printf("ADDR:[enc buf: %X], SIZE:[%s: %lu enc: %lu enc buf: %d]\n", \
//			client.getEncData(), _data_fmt, fb->len, client.getEncLen(), client.getBuffLen());
#endif
	esp_camera_fb_return(fb);

	return !(err)? PUB_OK : PUB_FAIL;
}

// Send stream data
mqtt_err_t send_video() {
	camera_fb_t * fb = NULL;
	uint8_t err_cnt = 0;

	portENTER_CRITICAL(&timer_mux); // lock start
	timer_cnt = 0;
	portEXIT_CRITICAL(&timer_mux); // lock end
	// timer interrupt start
	timerRestart(timer);

	while (timer_cnt < 3) { // about 3 minute
		fb = esp_camera_fb_get();
		if (!fb) {
			debug.println("Getting camera data failed");
			return ESP_FAIL;
		}

		mqtt_err_t err = client.sendBinary((const char*)pgm_read_ptr(&tp_video), fb->buf, fb->len);
		if (err) {
			err_cnt++;
			debug.printf("Publishing data failed, err code: 0x%d\n", err);
		}
#ifdef __DEBUG__
//		const char* _data_fmt = (const char*)pgm_read_ptr(img_fmt + fb->format);
//		debug.printf("ADDR:[enc buf: %X], SIZE:[%s: %lu enc: %lu enc buf: %d]\n", \
//				client.getEncData(), _data_fmt, fb->len, client.getEncLen(), client.getBuffLen());
#endif
		esp_camera_fb_return(fb);
		fb = NULL;
	}

	timerStop(timer);

	return !(err_cnt)? PUB_OK : PUB_FAIL;
}

// Receive messages for subscribed topic from the MQTT Server
void callback(char* topic, byte* payload, unsigned int length) {
	if (!memcmp((const char*)topic, "photo", 5)) { send_photo(); }
	else if (!memcmp((const char*)topic, "video", 5)) { send_video(); }
	else if (!memcmp((const char*)topic, "sensor", 6)) { sendToMCU(REQ, SENSOR); }
	else if (!memcmp((const char*)topic, "config", 6)) { /*TODO: Configuration?? */ }
}


// Setup uart serial
void setup_uart() {
	toMCU.begin(115200);
	// TODO: Uart interrupt settup
}

// Setup camera module	-> check setting --> image size
void setup_camera() {
	camera_config_t config;
	config.ledc_channel = LEDC_CHANNEL_0;
	config.ledc_timer = LEDC_TIMER_0;
	config.pin_d0 = Y2_GPIO_NUM;
	config.pin_d1 = Y3_GPIO_NUM;
	config.pin_d2 = Y4_GPIO_NUM;
	config.pin_d3 = Y5_GPIO_NUM;
	config.pin_d4 = Y6_GPIO_NUM;
	config.pin_d5 = Y7_GPIO_NUM;
	config.pin_d6 = Y8_GPIO_NUM;
	config.pin_d7 = Y9_GPIO_NUM;
	config.pin_xclk = XCLK_GPIO_NUM;
	config.pin_pclk = PCLK_GPIO_NUM;
	config.pin_vsync = VSYNC_GPIO_NUM;
	config.pin_href = HREF_GPIO_NUM;
	config.pin_sscb_sda = SIOD_GPIO_NUM;
	config.pin_sscb_scl = SIOC_GPIO_NUM;
	config.pin_pwdn = PWDN_GPIO_NUM;
	config.pin_reset = RESET_GPIO_NUM;
	config.xclk_freq_hz = 20000000;
	// test
	config.pixel_format = PIXFORMAT_JPEG; // 2BPP(RGB555)/2BPP(YUV422)/18PP/JPEG/3BPP/RAW/3BP2P(RGB444)/3BP2P(RGB555)
	//init with high specs to pre-allocate larger buffers
	if (psramFound()) {
		config.frame_size = FRAMESIZE_UXGA; // QQVGA/QQVGA2/QCIF/HQVGA/QVGA/CIF/VGA/SVGA/XGA/SXGA/UXGA/QXGA
		config.jpeg_quality = 1; // 0~63 lower means higher quality
		config.fb_count = 2; // Number of frame buffers to be allocated. If more than one, then each frame will be acquired. (double speed)
	} else {
		config.frame_size = FRAMESIZE_SVGA;
		config.jpeg_quality = 12;
		config.fb_count = 1;
	}

	// camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
		debug.printf("Camera init failed with error 0x%x", err);
		return;
#if 0 // Reboot???
		ESP.restart();
#endif
	}
}
#if 0
void detail_config() {

	sensor_t* s = esp_camera_sensor_get();
	//initial sensors are flipped vertically and colors are a bit saturated
	if (s->id.PID == OV3660_PID) {
		s->set_vflip(s, 1);//flip it back
		s->set_brightness(s, 1);//up the blightness just a bit
		s->set_saturation(s, -2);//lower the saturation
	}
	//drop down frame size for higher initial frame rate
	s->set_framesize(s, FRAMESIZE_QVGA);

}
#endif
// Setup wifi module
bool setup_wifi(wifi_mode_t mode) {
	bool res;

	wifi_mode_t _cur_mode = WiFi.getMode();
	if (_cur_mode == mode) { return true; }

	res = WiFi.mode(mode);
	if (!res) {
		debug.println("WiFi mode setup failed");
		return res;
	}
	
	delay(300);

	if (mode == WIFI_MODE_STA) {
#ifdef WRITE_EEPROM 
		/*
			TODO: Worklist
				1. EEPROM read(ID, Password for AP access)
															*/
#endif
		WiFi.begin(ap_ssid, ap_pw);
		debug.print("Attempting WiFi connection");
	} else if (mode == WIFI_MODE_APSTA || mode == WIFI_MODE_AP) {
#if 1
		// WiFi.disconnect(true, true); // mode test
//		WiFi.softAPConfig(apIP, apIP, apMask);
//		delay(500);
		/*
			TODO: ap_ssid & ap_password getting
					(smartconfig apply??)
												*/
#endif
		WiFi.softAP((const char*)pgm_read_ptr(&dev_ssid), (const char*)pgm_read_ptr(&dev_pw));
		IPAddress dev_ip = WiFi.softAPIP();
		if (!web_bind) {
#if 0 // string reserved
			strList.reserve(TOTAL_WIFI_STRING_LEN);
			htmlPage.reserve(HTML_PAGE_MAX_SIZE);
#endif
			server.on("/", handle_root);
			server.on("/credential", handle_credential);
			// server.onNotFound(handleNotFound);
			server.begin(); // TODO: how to close???
			web_bind = ON;
		}
#ifdef __DEBUG__
		debug.println();
		debug.print("Web server IP: "); // temporary
		debug.println(dev_ip);
//		WiFi.printDiag(debug);
#endif
	} else { return res; } // Wifi off

	delay(500);

	return res;
}

// Setup mqtt server
void setup_mqtt() {
	client.setServer((const char*)pgm_read_ptr(&mqtt_server), (uint16_t)pgm_read_word(&mqtt_port));
}

void setup_gpio() {
#ifdef __TEST_SW__ // GPIO Interrupt, used to test
	esp_err_t err;
	pinMode(MODE_SWITCH, INPUT_PULLUP);

	err = gpio_isr_handler_add((gpio_num_t)MODE_SWITCH, (gpio_isr_t)ap_set_callback, (void*)MODE_SWITCH);
	if (err) {
		debug.printf("GPIO ISR handler add failed with error 0x%x\n", err);
		return;
	}

	err = gpio_set_intr_type((gpio_num_t)MODE_SWITCH, (gpio_int_type_t)GPIO_INTR_NEGEDGE);
	if (err) {
		debug.printf ( "GPIO interrupt type set failed 0x%x\n", err);
		return;
	}
#endif
}

void setup_timer() {
	timer = timerBegin(1, 80, true);
	timerStop(timer);
	timerAttachInterrupt(timer, &on_timer, true);
	timerAlarmWrite(timer, 60000000, true);
	timerAlarmEnable(timer);
}

void handle_root() {
	server.send(200, "text/html", wifi_list_page());
}

void handle_credential() {
	String get_ssid = server.arg("apName");
	String get_pw = server.arg("apPw");
	String msg;

	get_ssid.toCharArray(ap_ssid, get_ssid.length()+1);
	get_pw.toCharArray(ap_pw, get_pw.length()+1);
#ifdef __DEBUG__
	debug.printf("- AP_SSID: %s\n", ap_ssid);
	debug.printf("- AP_PASSWORD: %s\n", ap_pw);
#endif

	if (save_credential()) { msg = "OK, Saved!\r\n <br>"; }
	else { msg = "Fail, Not Saved!\r\n <br>"; }

	server.send(200, "text/html", msg);
	server.client().stop(); // ---- ????

	delay(300);
}

String wifi_list_page() {
	String strList ="<ul>"; 

	for(int i =0; i< WIFI_MAX_SIZE; i++)
	{ 
		if(wifi_list[i] != "") 
		{ 
			strList += "<li><a name='" + wifi_list[i] + "' onclick='select(this.name)'>" + wifi_list[i] + "</a> </li>"; 
		} 
	} 
	strList += "</ul>"; 

	String apName = ""; 
	String apPw = ""; 
	String htmlPage = 
		String("<!DOCTYPE html>") + 
		"<html> <head>" + 
		"<meta http-equiv=\"Content-Type\" Content=\"text/html; charset=utf-8\">\r\n" + 
		"<meta name='viewport' content='width=device-width, initial-scale=1, minimum-scale=1, maximum-scale=1, user-scalable=no'>\r\n" + 
		"<title>WIFI 목록</title>\r\n"+ 
		"<style>\r\n"+ 
		".a:hover{cursor:pointer;}\r\n"+ 
		"</style></head>\r\n"+ 
		"<body>\r\n"+ 
		"<h1>WiFi List</h1>\r\n"+ 
		"인증가능한 WiFi를 선택하세요. <br>\r\n"+ 
		"<div id='wifi_list'>"+ strList + "</div>\r\n" 
		"<div id='selected'></div>\r\n" 
		"<div id='_pw'>\r\n"+ 
		"<form method='post' action='/credential'>\r\n"+ 
		"<input type='text' id='ap' name='apName'> 을 선택하셨습니다. 아래에 비밀번호를 입력하세요. <br>\r\n"+ 
		"<input type='password' id='pw' name='apPw'> <input type='submit' value='connect'>\r\n"+ 
		"</form>\r\n"+ 
		"</div>\r\n"+ 
		"</body>\r\n"+ 
		"<script>\r\n"+ 
		"document.getElementById('_pw').style.visibility = 'hidden';\r\n"+ 
		"function select(apName){\r\n"+ 
		"document.getElementById('ap').value = apName;\r\n"+ 
		"document.getElementById('_pw').style.visibility = 'visible';\r\n"+ 
		"document.getElementById('pw').value = ''}\r\n"+ 
		"function connection(){ \r\n"+ 
		"var pw = document.getElementById('pw').value;\r\n"+ "} \r\n"+ 
		"</script> \r\n"+ 
		"</html> \r\n";

	return htmlPage;
}

bool save_credential() {
#ifdef WRITE_EEPROM 
	/*
	   TODO: Save credentials in EEPROM
										*/
#endif
	return true;
}
