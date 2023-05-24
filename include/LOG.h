/*
 * BaseBoard
 * 
 */

#ifndef _LOG_H_
#define _LOH_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include "stdio.h"
#include "SEGGER_RTT.h"

//#define LOG_MSG_MAX_SIZE 1024

#define MSG_LENGTH 1024

char LOG_MSG[MSG_LENGTH];

#if defined(UartPrintf)
#ifdef __GNUC__
  /* With GCC, small printf (option LD Linker->Libraries->Small printf
     set to 'Yes') calls __io_putchar() */
  #define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
  #define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

#define LOG_CLEAR()
#define LOG     printf
#define LOGI    printf
#define LOGW    printf
#define LOGE    printf
//#define LOGos   Addto_osPrintf

#elif defined(LOG_DEBUG)

#define LOG_PROTO(type,color,format,...)            \
        SEGGER_RTT_printf(0,"%s%s" format"%s", \
                          color,                    \
                          type,                     \
                          ##__VA_ARGS__,            \
                          RTT_CTRL_RESET)

/* 清屏*/
#define LOG_CLEAR() SEGGER_RTT_WriteString(0, "  " RTT_CTRL_CLEAR)

/* 无颜色日志输出 */
#define LOG(format,...) LOG_PROTO("","",format,##__VA_ARGS__)

/* 有颜色格式日志输出 */
#define LOGI(format,...) LOG_PROTO("I: ", RTT_CTRL_TEXT_BRIGHT_GREEN , format, ##__VA_ARGS__)
#define LOGW(format,...) LOG_PROTO("W: ", RTT_CTRL_TEXT_BRIGHT_YELLOW, format, ##__VA_ARGS__)
#define LOGE(format,...) LOG_PROTO("E: ", RTT_CTRL_TEXT_BRIGHT_RED   , format, ##__VA_ARGS__)

#endif

//#define printf LOG
extern void Addto_osPrintf(char *format, ...);


#ifdef __cplusplus
}
#endif
#endif // !_LOG_H_