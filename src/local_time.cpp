/*
 * local_time.cpp - Implement functions of the class for get local time
 * by Jinseong Jeon
 * Created date - 2020.09.23
 */

#include <Arduino.h>
#include "local_time.h"

LOCALTIME::LOCALTIME(int YY, int MM, int DD, int hh, int mm, int ss)
: _currTime({ss, mm, hh, DD, MM, YY}),
//_currTime(tm_ss:ss, tm_mm:mm, tm_hh:hh, tm_DD:DD, tm_MM:MM, tm_YY:YY)
_ntpServer({"pool.ntp.org"}),
_timeZone(TIME_ZONE_OFFSET(TZ_KOREA)),
_dstOffset(DAYLIGHT_SAVE_TIME_OFFSET(DST_KOREA)),
_reSync(false),
_preSync(false),
_preSyncTime(0)
{}

LOCALTIME::~LOCALTIME() {
}

void LOCALTIME::setNtpServer(const char** ntpServer) {
	for (int i=0; i<3; i++) {
		_ntpServer[i] = reinterpret_cast<const char*>(pgm_read_ptr(ntpServer+i));
	}
}

bool LOCALTIME::onSetTime(time_t period) {
	if (!_preSync) { return true; }

	time_t now;
	time_t diff = time(&now) - _preSyncTime;
	if (diff >= period) { return true; }

	return false;
}

bool LOCALTIME::timeSync() {
	time_t now, curr;
	bool execSw = false;

	if (_preSync) { _saveCurrTime(); }

	configTime(_timeZone, _dstOffset, _ntpServer[0], _ntpServer[1], _ntpServer[2]);

	while ((curr = time(&now)) < 5) {
		if (execSw  ^ (curr & 0x1)) { // Execute per sec
			configTime(_timeZone, _dstOffset, _ntpServer[0], _ntpServer[1], _ntpServer[2]);
			execSw = !execSw;
		}
	}

	_preSyncTime = time(&now);
	_preSync = true;

	return (curr != 5)? getLocalTime(&_tmInfo) : false;
}

void LOCALTIME::setCalcTime(int timeOffset) {
	_tmInfo.tm_sec = _currTime.tm_ss;
	_tmInfo.tm_min = _currTime.tm_mm;
	_tmInfo.tm_hour = _currTime.tm_hh;
	_tmInfo.tm_mday = _currTime.tm_DD;
	_tmInfo.tm_mon = _currTime.tm_MM - 1;
	_tmInfo.tm_year = _currTime.tm_YY - 1900;
	time_t preTime = mktime(&_tmInfo);
	struct timeval curr = { .tv_sec = preTime + ((~_reSync + 1) & timeOffset) };
	settimeofday(&curr, NULL);
	time_t now;
	_preSyncTime = time(&now);
}

const char* LOCALTIME::getCurrTime(char* buf, size_t len) {
	if (len < 20) { return nullptr; }
	getLocalTime(&_tmInfo);
	_currTime.tm_ss = _tmInfo.tm_sec;
	_currTime.tm_mm = _tmInfo.tm_min;
	_currTime.tm_hh = _tmInfo.tm_hour;
	_currTime.tm_DD = _tmInfo.tm_mday;
	_currTime.tm_MM = _tmInfo.tm_mon + 1;
	_currTime.tm_YY = _tmInfo.tm_year + 1900;
	sprintf(buf, "%d-%02d-%02dT%02d:%02d:%02d", _currTime.tm_YY, \
                                                _currTime.tm_MM, \
                                                _currTime.tm_DD, \
                                                _currTime.tm_hh, \
                                                _currTime.tm_mm, \
                                                _currTime.tm_ss);
	return const_cast<const char*>(buf);
}

void LOCALTIME::_saveCurrTime() {
	_reSync = true;
	getLocalTime(&_tmInfo);
	_currTime.tm_ss = _tmInfo.tm_sec;
	_currTime.tm_mm = _tmInfo.tm_min;
	_currTime.tm_hh = _tmInfo.tm_hour;
	_currTime.tm_DD = _tmInfo.tm_mday;
	_currTime.tm_MM = _tmInfo.tm_mon + 1;
	_currTime.tm_YY = _tmInfo.tm_year + 1900;

	struct timeval tmInit = { .tv_sec = 0 };
	settimeofday(&tmInit, NULL);
}
