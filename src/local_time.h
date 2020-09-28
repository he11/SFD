/*
 * local_time.h - Define a class for get local time
 * by Jinseong Jeon
 * Created date - 2020.09.23
 */

#ifndef _LOCAL_TIME_H_
#define _LOCAL_TIME_H_

#include <sys/time.h>

#define TIME_ZONE_OFFSET(n)              ((n)*(3600)) // unit: sec
#define DAYLIGHT_SAVE_TIME_OFFSET(n)     ((n)*(3600)) // unit: sec

// Time Zone
typedef enum {
	TZ_KOREA = 9
} tz_t;

// Daylight Save Time
typedef enum {
	DST_KOREA = 0
} dst_t;

// Store Current Time
typedef struct {
	int tm_ss;
	int tm_mm;
	int tm_hh;
	int tm_DD;
	int tm_MM;
	int tm_YY;
} tm_info_t;


class LOCALTIME {
private:
	const char* _ntpServer[3];
	long _timeZone;
	int _dstOffset;
	struct tm _tmInfo;
	tm_info_t _currTime;
	bool _reSync;
	bool _preSync;
	time_t _preSyncTime;

public:
	LOCALTIME(int YY, int MM, int DD, int hh, int mm, int ss);
	~LOCALTIME();

	void setNtpServer(const char** ntpServer);
	bool onSetTime(time_t period);
	bool timeSync();
	void setCalcTime(int timeOffset);
	const char* getCurrTime(char* buf, size_t len);

protected:
	void _saveCurrTime();
};
#endif // END _LOCAL_TIME_H_
