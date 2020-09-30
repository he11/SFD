/*
 * MQTT.cpp - Implement functions of the class for MQTT communication
 * by Jinseong Jeon
 * Created date - 2020.09.04
 */

#include <ArduinoJson.h>
#include "mbedtls/base64.h"
#include "mqtt.h"

MQTT::MQTT(Client& client, MQTT_CALLBACK_SIGNATURE)
: PubSubClient(client),
_maxAuthLen(MAX_AUTH_STR_LEN),
_clientId(nullptr),
//_clientPw(nullptr),
_pubData(nullptr),
_pubLen(0),
_encBuff(nullptr),
_buffLen(0),
_encLen(0),
_fragBuff(nullptr),
_fragLen(0),
_jsonBuff(nullptr),
_jsonLen(0),
_preTime(0)
{PubSubClient::setCallback(callback);
PubSubClient::setBufferSize(MQTT_MAX_PACKET_SIZE);}

MQTT::~MQTT() {
	deleteBuffer();
}

#ifdef _D1_
size_t MQTT::getBuffLen() {
	return _buffLen;
}

size_t MQTT::getEncLen() {
	return _encLen;
}

const char* MQTT::getEncData() {
	return reinterpret_cast<const char*>(_encBuff);
}
#endif

mqtt_err_t MQTT::Subscribe(const char** topics, uint8_t topicCnt) {
	for (int i=0; i<topicCnt; i++) {
		bool res = PubSubClient::subscribe(reinterpret_cast<const char*>(pgm_read_ptr(topics+i)), 0);
		if (!res)
			return MQTT_FAIL;
	}

	return MQTT_OK;
}

mqtt_err_t MQTT::sendImage(const char* topic, const uint8_t* rawData, size_t rawLen, const char* current) {
	mqtt_err_t err = 0;

	_pubData = rawData;
	_pubLen = rawLen;
#if defined(__BASE64_ENC__) || defined(__JSON__)
	err = _base64Enc(_pubData, _pubLen);
	if (err) { return err; }
#endif
	uint16_t fragCnt = TOTAL_FRAGMENT_CNT(_pubLen);
	uint16_t cntToSend = fragCnt;
	int32_t preLen = static_cast<int32_t>(_pubLen);
	const uint8_t* prePos = _pubData;
	size_t lenToSend = _pubLen;
#ifdef __JSON__
	char getUUID[37];
	const char* uuid = _generateUUID(1, getUUID, 37);
	size_t dataLen = _pubLen;
#ifdef __PACKET_FRAG__
	_fragBuff = _variableBuff(_fragBuff, &_fragLen, __PACKET_FRAG__+1);
	if (!_fragBuff) { return MQTT_ERR_MEM_ALLOC_FAIL; }
#endif
#endif

	do {
#ifdef __PACKET_FRAG__
		lenToSend = (preLen > __PACKET_FRAG__)? __PACKET_FRAG__ : preLen;
#endif
		_pubLen = lenToSend;
		_pubData = prePos;
#ifdef __JSON__
		StaticJsonDocument<JSON_IMAGE_PACK_CAPACITY> pubMsg;
		pubMsg["uuid"] = uuid;
		pubMsg["date"] = current;
		pubMsg["total_len"] = dataLen;
		JsonObject frag = pubMsg.createNestedObject("frag");
		frag["cnt"] = fragCnt;
		frag["pos"] = fragCnt - cntToSend;
#ifdef __PACKET_FRAG__
		memcpy(_fragBuff, prePos, lenToSend);
		_fragBuff[lenToSend] = '\0';
		_pubData = reinterpret_cast<const uint8_t*>(_fragBuff);
#endif
		pubMsg["data"] = reinterpret_cast<const char*>(_pubData);
		size_t msgLen = measureJson(pubMsg) + 1; // +1: for null
		_jsonBuff = _variableBuff(_jsonBuff, &_jsonLen, msgLen);
		if (!_jsonBuff) { return MQTT_ERR_MEM_ALLOC_FAIL; }
		serializeJson(pubMsg, _jsonBuff, msgLen);
		_pubLen = msgLen - 1;
		_pubData = reinterpret_cast<const uint8_t*>(_jsonBuff);
#endif
		err += PubSubClient::publish(topic, _pubData, _pubLen, false)? 0 : 1;
#ifdef _D1_
		freeMemSize(__func__, __LINE__);
#ifdef __JSON__
		debug.printf("msgLen: %d, fragBuffLen: %d, jsonBuffLen: %d, fragBuff: %x, jsonBuff: %x\n", \
                      msgLen, _fragLen, _jsonLen, _fragBuff, _jsonBuff);
#endif
		debug.printf("Total cnt: %hu, Left cnt: %hu, Left len: %d, Cur frag len: %lu\n", \
                      fragCnt, cntToSend, preLen, _pubLen);
#endif
		prePos += lenToSend;
		preLen -= lenToSend;
	} while(--cntToSend);

	return (!err)? PUB_OK : PUB_FAIL ;
}

mqtt_err_t MQTT::sendData(const char* topic, const uint8_t* rawData, size_t rawLen, const char* current) {
	StaticJsonDocument<JSON_DATA_PACK_CAPACITY> pubMsg;
	StaticJsonDocument<JSON_DATA_CAPACITY> Data;
	char uuid[37];
	pubMsg["uuid"] = _generateUUID(1, uuid, 37);
	pubMsg["date"] = current;
	uint8_t getData[rawLen];
	memcpy(getData, rawData, rawLen);
	deserializeJson(Data, getData);
	pubMsg["data"] = Data;

	size_t msgLen = measureJson(pubMsg) + 1; // +1: for null
	uint8_t msgBuff[msgLen];
	serializeJson(pubMsg, msgBuff, msgLen);
	_pubLen = msgLen;
	_pubData = msgBuff;

	return PubSubClient::publish(topic, _pubData, _pubLen, false)? PUB_OK : PUB_FAIL;
}

#if defined(__BASE64_ENC__) || defined(__JSON__)
mqtt_err_t MQTT::_base64Enc(const uint8_t* pubData, size_t pubLen) {
	size_t expLen = EXPECTED_DATA_BUFF_SIZE(pubLen);

	_encBuff = _variableBuff(_encBuff, &_buffLen, expLen);
	if (!_encBuff) { return MQTT_ERR_MEM_ALLOC_FAIL; }

	memset(_encBuff, 0, _buffLen);
	int err = mbedtls_base64_encode(_encBuff, _buffLen, &_encLen, \
              reinterpret_cast<const unsigned char*>(pubData), pubLen);
	if (err) { return (MQTT_ERR_ENC_FAIL | (~(err) + 1)); }

	_encBuff[_encLen] = '\0';
	_pubLen = _encLen;
	_pubData = reinterpret_cast<const uint8_t*>(_encBuff);

	return MQTT_OK;
}
#endif

const char* MQTT::_generateUUID(uint8_t version, char* uuid, size_t bufLen) {
	uint8_t res = 0;

	if (version == 1) {
		time_t now;
		uint64_t timeStamp = (time(&now) * 10000) + 122192928000000000; // since 15 October 1582
		uint32_t tsLow = timeStamp & 0xffffffff;
		uint16_t tsMid = (timeStamp>>32) & 0xffff;
		uint16_t tsHigh = (timeStamp>>48) & 0x0fff;
		uint16_t clockSeq = random(0xffff);
		if (now < _preTime) { clockSeq++; }
		clockSeq &= 0x3fff;
		char macId[13];
		res = sprintf(uuid, "%08x-%04x-%04x-%04x-%s", \
                             tsLow, tsMid, tsHigh | 0x1000,
                             clockSeq | 0x8000, _getMacId(macId));
		_preTime = now;
#ifdef _D1_
		debug.printf("ts: %016llx / low: %08x / mid: %04hx / high: %04hx / clk: %04x\n",
                      timeStamp, tsLow, tsMid, tsHigh, clockSeq);
		debug.printf("uuid: %s\n", uuid);
#endif
	}
	if (version == 4) {
		res = sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x", \
                             random(0xffff), random(0xffff), 
                             random(0xffff),
                             (random(0xffff) & 0x0fff) | 0x4000,
                             (random(0xffff) & 0x3fff) | 0x8000,
                             random(0xffff), random(0xffff), random(0xffff));
	}

	return (res != 36)? nullptr : reinterpret_cast<const char*>(uuid);
}

const char* MQTT::_getMacId(char* macId) {
	uint8_t mac[6];
	esp_err_t err = esp_efuse_mac_get_default(mac);
	if (err) { return nullptr; }

	for (int i=0; i<6; i++) {
		sprintf(macId + (i<<1), "%02x", mac[i]);
	}

	return const_cast<const char*>(macId);
}

unsigned char* MQTT::_variableBuff(unsigned char* oldBuff, size_t* oldLen, size_t newLen) {
	unsigned char* newBuffer = nullptr;

	if (!oldBuff || newLen > *oldLen) {
		newBuffer = reinterpret_cast<unsigned char*>(EXTEND_MEM(oldBuff, newLen));
		if (!newBuffer) { return nullptr; }
		*oldLen = newLen;
	} else { newBuffer = oldBuff; }

	return newBuffer;
}

void MQTT::deleteBuffer() {
#if defined(__BASE64_ENC__) || defined(__JSON__)
	if (_encBuff) {
		free(_encBuff);
		_encBuff = nullptr;
		_buffLen = 0;
	}
	if (_fragBuff) {
		free(_fragBuff);
		_fragBuff = nullptr;
		_fragLen = 0;
	}
	if(_jsonBuff) {
		free(_jsonBuff);
		_jsonBuff = nullptr;
		_jsonLen = 0;
	}
#endif
}
