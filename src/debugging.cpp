/*
 * debugging.cpp - Implement debug functions
 * by Jinseong Jeon
 * Created date - 2020.09.27
 */

#include <Arduino.h>
#include "debugging.h"

#ifdef _D1_
void freeMemSize(const char* funcName, int32_t line) {
	debug.println(F("-----------------------------------------"));
	debug.printf("Func: %s / Line: %d\n", funcName, line);
	debug.printf("Free heap size: %d\n", ESP.getFreeHeap());
	debug.printf("Free max heap size: %d\n", ESP.getMaxAllocHeap());
	debug.printf("Free psram size: %d\n", ESP.getFreePsram());
	debug.printf("Free max psram size: %d\n", ESP.getMaxAllocPsram());
	debug.printf("Current stack size: %d\n", uxTaskGetStackHighWaterMark(NULL));
	debug.println(F("-----------------------------------------"));
}

// Print to hex value
void print2hex(const uint8_t* buf, size_t len) {
	char hex[3];
	uint8_t cnt = 0;

	for (int i=0; i<len; i++) {
		sprintf(hex, "%02X", buf[i]);
		debug.printf(" %s", hex);
		if (cnt%10 == 0 && cnt)
			debug.println();
		cnt++;
	}

	debug.println();
}

void printLineDiv(const char* buf, size_t len) {
	int cnt = 0;

	for (int i=0; i<len; i++) {
		debug.printf("%c", buf[i]);
		if (cnt%10 == 0 && cnt)
			debug.println();
		cnt++;
	}

	debug.println();
}

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

void send_err(uint8_t err_code) {
	Serial.write(0xBE);
	Serial.write(0x03); // ERR
	Serial.write((uint8_t)0);
	Serial.write((uint8_t)1);
	Serial.write(err_code);
	Serial.write(0xED);
}
