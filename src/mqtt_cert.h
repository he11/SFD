/*
 * mqtt_cert.h - Certificate data for secure MQTT over tls
 * by Jinseong Jeon
 * Created date - 2020.09.27
 */

#ifndef _MQTT_CERTIFICATE_H_
#define _MQTT_CERTIFICATE_H_

extern const char* ca_cert PROGMEM = ;

// You can use x.509 client certificates if you want
//extern const char* client_cert PROGMEM = ;
//extern const char* client_private_key PROGMEM = ;
#endif // END _MQTT_CERTIFICATE_H_
