/*
 * debugging.h - Define debug functions
 * by Jinseong Jeon
 * Created date - 2020.09.27
 */

#ifndef _DEBUGGING_H_
#define _DEBUGGING_H_

#define __TEST__
//#define __DEBUG__ 0

#ifdef __DEBUG__
#define debug Serial
#define _D0_
#if __DEBUG__ == 1
#define _D1_

extern void freeMemSize(const char* funcName, int32_t line);
extern void print2hex(const uint8_t* buf, size_t len);
extern void printLineDiv(const char* buf, size_t len);
extern void printRaw(const uint8_t* buf, size_t len);
#endif
#endif
extern void send_err(uint8_t err_code);
#endif // END _DEBUGGING_H_
