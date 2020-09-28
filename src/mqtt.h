/*
 * MQTT.h - Define MQTT interface class based on the "PubSubClient" class
 * by Jinseong Jeon
 * Created date - 2020.09.04
 */

#ifndef _MQTT_H_
#define _MQTT_H_

#include <PubSubClient.h>
#include "debugging.h"

#define __PSRAM_EN__
#define MQTT_MAX_PACKET_SIZE             16384

#define __BASE64_ENC__
#define __PACKET_FRAG__                  15000 // Frag size must be less about 10% than MQTT_MAX_PACKET_SIZE 

typedef int32_t mqtt_err_t;

/* Definitions for error constants. */
#define MQTT_OK                          0 /*!< mqtt_err_t value indicating success (no error) */
#define MQTT_FAIL                       -1 /*!< mqtt_err_t value indicating failure */

#define PUB_OK                           0 /*!< Success to publish MQTT message */
#define PUB_FAIL                        -1 /*!< Failure to publish MQTT message */

#define MQTT_ERR_MEM_ALLOC_FAIL          0x100 /*!< Memory allocation failure */
#define MQTT_ERR_AUTH_FAIL               0x200 /*!< Authentication failure to access a MQTT server */
#define MQTT_ERR_ENC_FAIL                0x300 /*!< Data encoding failure */

#define MAX_AUTH_STR_LEN                 30

//#define JSON_PUB_MSG_CAPACITY            JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(6) + JSON_ARRAY_SIZE(2)
#define JSON_PUB_MSG_CAPACITY            JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(6) + JSON_ARRAY_SIZE(2)
#define JSON_DATA_CAPACITY               JSON_OBJECT_SIZE(6) + JSON_ARRAY_SIZE(2)
// #define EXPECTED_DATA_BUFF_SIZE(n)    (((((n) << 2) / 3) + x) & ~3) // about 33%
#define EXPECTED_DATA_BUFF_SIZE(n)       ((n) * 1.4) // about 40%

#ifndef __PACKET_FRAG__
 #define TOTAL_FRAGMENT_CNT(x)           1
#else
 #define TOTAL_FRAGMENT_CNT(x)           (((x) / (__PACKET_FRAG__)) + (((x) % (__PACKET_FRAG__))? 1:0))
#endif

#ifdef __PSRAM_EN__
 #define EXTEND_MEM(p_bf, l)             heap_caps_realloc(p_bf, l, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)
#else
 #define EXTEND_MEM(p_bf, l)             realloc(p_bf, l)
#endif


class MQTT : public PubSubClient {
private:
	uint8_t _maxAuthLen;
	unsigned char* _clientId;
//	unsigned char* _clientPw;
	const uint8_t* _pubData;
	size_t _pubLen;
	unsigned char* _encBuff;
	size_t _buffLen;
	size_t _encLen;
	time_t _preTime;

public:
	MQTT(Client& client, MQTT_CALLBACK_SIGNATURE);
	~MQTT();

//	void setAuthInfo(xxxx);
	mqtt_err_t Subscribe(const char** topics, uint8_t topicCnt);
	void deleteBuffer();
	/*
	 * Json Message Component
	 * {
	 * //  "uuid": 8-4-4-4-12(string),	# RFC4122 version.4 Random values
	 *   "uuid": 8-4-4-4-12(string),	# RFC4122 version.1 TimeNode base
	 *   "date": string,				# Publication date
	 * //  "mac_addr": string,			# Mac address of device
	 *	 "data": {						# Sensor Data
	 *	  "dust" : float
	 *	  "fire" : bool
	 *	  "gas" : bool
	 *	  "motion" : list[bool, bool]
	 *	  "smoke" : bool
	 *	  "temperature" : float
	 *	}
	 * }
	 * Max capacity = JSON_OBJECT_SIZE(3) + JSON_OBJECT_SIZE(6) + JSON_ARRAY_SIZE(2)
	 * //Max capacity = JSON_OBJECT_SIZE(4) + JSON_OBJECT_SIZE(6) + JSON_ARRAY_SIZE(2)
	 */
	mqtt_err_t sendJson(const char* topic, const uint8_t* rawData, size_t rawLen, const char* current);
	mqtt_err_t sendBinary(const char* topic, const uint8_t* rawData, size_t rawLen);

#ifdef _D1_
	const char* _generateUUID(uint8_t version, char* uuid, size_t bufLen);
	size_t getBuffLen();
	size_t getEncLen();
	const char* getEncData();
#endif
protected:
	mqtt_err_t _base64Enc(const uint8_t* pubData, size_t pubLen);
	unsigned char* _variableBuff(unsigned char* oldBuff, size_t* oldLen, size_t newLen);
//	const char* _generateUUID(uint8_t version, char* uuid, size_t bufLen);
	const char* _getMacId(char* macId);
};
#endif // END _MQTT_H_
