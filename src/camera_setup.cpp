/*
 * camera_setup.cpp - Implement setup function for use a camera
 * by Jinseong Jeon
 * Created date - 2020.09.27
 */

#include <Arduino.h>
#include "camera_setup.h"

// Setup camera module
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
	config.pixel_format = PIXFORMAT_JPEG;
	//init with high specs to pre-allocate larger buffers
	if (psramFound()) {
		config.frame_size = FRAMESIZE_XGA; //FRAMESIZE_UXGA;
		config.jpeg_quality = 10; //1;
		config.fb_count = 2; // Number of frame buffers to be allocated. If more than one, then each frame will be acquired. (double speed)
	} else {
		config.frame_size = FRAMESIZE_SVGA;
		config.jpeg_quality = 12;
		config.fb_count = 1;
	}
	// camera init
	esp_err_t err = esp_camera_init(&config);
	if (err != ESP_OK) {
#ifdef _D0_
		debug.printf("Camera init failed with error 0x%x", err);
#endif
#if 1 // Reboot
		ESP.restart();
#else
		return;
#endif
	}
}

#ifdef __TEST__
void adjust_img(int quality, uint8_t frame) {
	sensor_t* conf = esp_camera_sensor_get();
	framesize_t fType;
	
	switch (frame) {
		case 0: fType = FRAMESIZE_QQVGA; break;
		case 1: fType = FRAMESIZE_QQVGA2; break;
		case 2: fType = FRAMESIZE_QCIF; break;
		case 3: fType = FRAMESIZE_HQVGA; break;
		case 4: fType = FRAMESIZE_QVGA; break;
		case 5: fType = FRAMESIZE_CIF; break;
		case 6: fType = FRAMESIZE_VGA; break;
		case 7: fType = FRAMESIZE_SVGA; break;
		case 8: fType = FRAMESIZE_XGA; break;
		case 9: fType = FRAMESIZE_SXGA; break;
		case 10: fType = FRAMESIZE_UXGA; break;
		case 11: fType = FRAMESIZE_QXGA; break;
		default: fType = FRAMESIZE_INVALID;
	}
	conf->set_framesize(conf, fType); // QQVGA/QQVGA2/QCIF/HQVGA/QVGA/CIF/VGA/SVGA/XGA/SXGA/UXGA/QXGA
	conf->set_quality(conf, quality); // 0~63 lower means higher quality
//	conf->set_pixformat(conf, PIXFORMAT_JPEG); // 2BPP(RGB555)/2BPP(YUV422)/18PP/JPEG/3BPP/RAW/3BP2P(RGB444)/3BP2P(RGB555)
}
#endif
