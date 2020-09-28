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
_encLen(0)
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

mqtt_err_t MQTT::sendBinary(const char* topic, const uint8_t* rawData, size_t rawLen) {
	mqtt_err_t err = 0;

	_pubData = rawData;
	_pubLen = rawLen;
#ifdef __BASE64_ENC__
	err = _base64Enc(_pubData, _pubLen);
	if (err) { return err; }
#endif
	uint16_t fragCnt = TOTAL_FRAGMENT_CNT(_pubLen);
	uint16_t cntToSend = fragCnt;
	int32_t lenToSend = static_cast<int32_t>(_pubLen);

	do {
#ifdef __PACKET_FRAG__
		_pubLen = (lenToSend > __PACKET_FRAG__)? __PACKET_FRAG__ : lenToSend;
#endif
		err += PubSubClient::publish(topic, _pubData, _pubLen, false)? 0 : 1;
#ifdef _D1_
		debug.printf("Total cnt: %hu, Left cnt: %hu, Left len: %d, Cur frag len: %lu\n", fragCnt, cntToSend, lenToSend, _pubLen);
#endif
		_pubData += _pubLen;
		lenToSend -= _pubLen;
	} while(--cntToSend);

	return (!err)? PUB_OK : PUB_FAIL ;
}

mqtt_err_t MQTT::sendJson(const char* topic, const uint8_t* rawData, size_t rawLen, const char* current) {
	StaticJsonDocument<JSON_PUB_MSG_CAPACITY> pubMsg;
	StaticJsonDocument<JSON_DATA_CAPACITY> Data;
	char uuid[37];
	pubMsg["uuid"] = _generateUUID(uuid, 37);
	pubMsg["date"] = current;
	char macId[13];
	pubMsg["mac_id"] = _getMacId(macId);
	deserializeJson(Data, rawData, rawLen);
	pubMsg["data"] = Data.as<JsonObject>();

	size_t msgLen = measureJson(pubMsg) + 1; // +1: for null
	uint8_t msgBuff[msgLen];
	serializeJson(pubMsg, msgBuff, msgLen);
	_pubLen = msgLen;
	_pubData = msgBuff;

	return PubSubClient::publish(topic, _pubData, _pubLen, false)? PUB_OK : PUB_FAIL;
}

#ifdef __BASE64_ENC__
mqtt_err_t MQTT::_base64Enc(const uint8_t* pubData, size_t pubLen) {
	size_t expLen = EXPECTED_DATA_BUFF_SIZE(pubLen);

	_encBuff = _variableBuff(_encBuff, &_buffLen, expLen);
	if (!_encBuff) { return MQTT_ERR_MEM_ALLOC_FAIL; }

	memset(_encBuff, 0, _buffLen);
	int err = mbedtls_base64_encode(_encBuff, _buffLen, &_encLen, \
              reinterpret_cast<const unsigned char*>(pubData), pubLen);
	if (err) { return (MQTT_ERR_ENC_FAIL | (~(err) + 1)); }

	_pubLen = _encLen;
	_pubData = reinterpret_cast<const uint8_t*>(_encBuff);

	return MQTT_OK;
}
#endif

const char* MQTT::_generateUUID(char* uuid, size_t bufLen) {
	sprintf(uuid, "%04x%04x-%04x-%04x-%04x-%04x%04x%04x", \
                            random(0xffff), random(0xffff), 
                            random(0xffff),
                            (random(0xffff) & 0x0fff) | 0x4000,
                            (random(0xffff) & 0x3fff) | 0x8000,
                            random(0xffff), random(0xffff), random(0xffff));

	return reinterpret_cast<const char*>(uuid);
}

const char* MQTT::_getMacId(char* macId) {
	uint8_t mac[6];
	esp_err_t err = esp_efuse_mac_get_default(mac);
	if (err) { return nullptr; }

	for (int i=0; i<6; i++) {
		sprintf(macId + (i<<1), "%02X", mac[i]);
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
#ifdef __BASE64_ENC__
	if (_encBuff) {
		free(_encBuff);
		_encBuff = nullptr;
		_buffLen = 0;
	}
#endif
}
