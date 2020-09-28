/*
 * camera_setup.h - Define camera pins for use a camera
 * by Jinseong Jeon
 * Created date - 2020.09.27
 */

#ifndef _CAMERA_SETUP_H_
#define _CAMERA_SETUP_H_

#include "esp_camera.h"
#include "debugging.h"

#define CAMERA_MODEL_AI_THINKER

#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

extern void setup_camera();
#ifdef __TEST__
extern void adjust_img(int quality, uint8_t frame);
#endif
#endif // END _CAMERA_SETUP_H_
