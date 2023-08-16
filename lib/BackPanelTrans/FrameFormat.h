#ifndef FRAME_FORMAT_h
#define FRAME_FORMAT_h

/**
 * 自定义协议的数据帧格式
 *  _______________________________________________________________________
 *  |     起始     |    内容长度    |    消息类型    |    内容    |    CRC    |
 *  |  0x23 0x32  |     2Byte     |     1Byte     |     -     |   2Byte   |
 *  |_____________|_______________|_______________|___________|___________|
 */
/*数据帧的格式*/ 
#define FRAME_START_1 0x23 
#define FRAME_START_2 0x32
#define FRAME_BASELEN   7

#define Recall_BaseMsgType 0x80
/*消息类型*/
typedef enum 
{
	ResetPowerBoard_MsgType	    = 0,    //BaseBoard --> PowerBoard
    StartPowerBoard_MsgType     = 1,    //BaseBoard --> PowerBoard
	SendCMD_MsgType		        = 2,    //BaseBoard --> PowerBoard
	TransToken_MsgType		    = 3,
    TransData_MsgType		    = 4,
	ReadStatus_MsgType		    = 5,
} FrameMsgType_t;


/**
 * 消息类型为 0x00 ： BaseBoard  发送给  PowerBoard， 用于复位PowerBoard板   ###单独由串口2来实现###
 *  _________________________________
 *  |     消息类型     |     板ID     | 
 *  |      1Byte     |     1Byte    |
 *  |________________|______________|
 * 
 * PowerBoard回应消息
 * 消息类型为 0x80 ： PowerBoard  回应消息给 BaseBoard
 *  ________________________________________________
 *  |     消息类型     |     板ID     |     结果     |
 *  |      1Byte     |     1Byte    |    1Byte    |
 *  |________________|______________|_____________|
 */
#define BaseBoard_ID    0x00
#define PowerBoard_1_ID 0x01
#define PowerBoard_2_ID 0x02
#define PowerBoard_3_ID 0x03
#define PowerBoard_4_ID 0x04

/**
 * 消息类型为 0x01 ： BaseBoard  发送给  PowerBoard, 用于启动powerboard板   ###单独由串口2来实现###
 *  _________________________________
 *  |     消息类型     |     板ID     | 
 *  |      1Byte     |     1Byte    |  
 *  |________________|______________|
 * 
 * PowerBoard回应消息
 * 消息类型为 0x81 ： PowerBoard  回应消息给 BaseBoard
 *  ________________________________________________
 *  |     消息类型     |     板ID     |     结果     |
 *  |      1Byte     |     1Byte    |    1Byte    |
 *  |________________|______________|_____________|
 */




/**
 * 消息类型为 0x02 ： BaseBoard  发送 控制指令 给  PowerBoard   ###单独由串口2来实现###
 *  ____________________________________________________________________________________________________
 *  |     消息类型     |     板ID     |     控制类型 + 控制的数值 1    		       控制类型 + 控制的数值 9     |
 *  |      1Byte     |     1Byte    |      1Byte  +   2Byte       ........     1Byte  +   2Byte        |
 *  |________________|______________|__________________________________________________________________|
 * 
 * 板ID ： BaseBoard = 0x00 ， PowerBoard_1 = 0x01 ， PowerBoard_2 = 0x02 ， PowerBoard_3 = 0x03 ， PowerBoard_4 = 0x04
 * 通道ID ： 每个PowerBoard板上都有9路负载，第一路负载的通道ID为0x01，以此类推，第九路负载的通道ID为0x09
 * 控制类型 ： 目前支持的控制类型有2种：PWM Control 和 Phase Control
 * 控制的数值： 根据控制类型的不同，数值类型也不一样
 * 
 * PowerBoard回应消息
 * 消息类型为 0x82 ： PowerBoard  回应消息给 BaseBoard
 *  ________________________________________________
 *  |     消息类型     |     板ID     |     结果     |
 *  |      1Byte     |     1Byte    |    1Byte    |
 *  |________________|______________|_____________|
 */
#define PWM_Control_Type    0x01
#define Phase_Control_Type  0x02


/**
 * 消息类型为 0x03 ： BaseBoard板 和 4块PowerBoard板之间相互传递     ###单独由串口3来实现###
 * 此消息类型可以跟其他消息类型合并使用，比如跟 “TansData_MsgType” 合并使用，可以在传输数据的同时，将令牌传递给下一个从站
 *  _______________________________________________________________________________
 *  |     消息类型     |                           令牌ID                           | 
 *  |________________|____________________________________________________________|
 *  |                |    当前板子的ID    ｜    下一个板子的ID    ｜    令牌有效时间    ｜  
 *  |      1Byte     |       1Byte       |        1Byte       |        2Byte      |
 *  |________________|___________________|____________________|___________________|
 * 令牌ID ： 当前板ID + 下一个板ID + 令牌有效时间
 */
extern uint8_t      _token_currentBoardID;
extern uint8_t      _token_nextBoard_ID; 
extern uint16_t     _token_validTime;


/**
 * 消息类型为 0x04 ： BaseBoard板 发送 获取数据的指令 给 PowerBoard     ###单独由串口3来实现###
 *  _________________________________
 *  |     消息类型     |     板ID     |
 *  |      1Byte     |    1Byte     |
 *  |________________|______________|
 * 如果与令牌指令合并使用 板ID默认设置为 0xFF，如果单独使用，就是具体的PowerBoard板的ID
 * 
 * PowerBoard回应消息
 * 消息类型为 0x84 ： PowerBoard  传递数据给  BaseBoard， 同时将令牌发送给其他PowerBoard
 *  __________________________________________________________________________________________________
 *  |     消息类型     |     板ID     |     数据类型    |     数据     ｜     数据类型     |     数据      ｜
 *  |      1Byte     |    1Byte     |     1Byte      |    9Byte     |      1Byte      |   4*9Byte    |
 *  |________________|______________|________________|______________|_________________|______________|
 */
//#define LoadState_DataType       0x01
//#define Inow_DataType            0x02
//#define Irms_DataType            0x03
//#define Itrms_DataType           0x04



/**
 * 传递令牌和传输数据两个消息类型合并使用的示例：
 * _________________________________________________________________________________________________________________________________________________________________________________________________
 * |                |     消息类型     |    当前板子的ID    ｜    下一个板子的ID    ｜    令牌有效时间    ｜     消息类型     |     板ID     |     数据类型    |     数据     ｜     数据类型     |     数据      ｜
 * |   BaseBoard    |      0x03       |       0x00       |          0x01       |     0x10 0x00     |       0x04      |     0xFF    |                |              |                |               |
 * |  PowerBoard 1  |      0x03       |       0x01       |          0x02       |     0x10 0x00     |       0x84      |     0xFF    |       0x01     |    9Byte     |      0x02      |    4*9Byte    |
 * |  PowerBoard 2  |      0x03       |       0x02       |          0x03       |     0x10 0x00     |       0x84      |     0xFF    |       0x01     |    9Byte     |      0x02      |    4*9oard    |
 * |  PowerBoard 3  |      0x03       |       0x03       |          0x04       |     0x10 0x00     |       0x84      |     0xFF    |       0x01     |    9Byte     |      0x02      |    4*9Byte    |
 * |  PowerBoard 4  |      0x03       |       0x04       |          0x05       |     0x10 0x00     |       0x84      |     0xFF    |       0x01     |    9Byte     |      0x02      |    4*9Byte    |
 * |________________|_________________|__________________|_____________________|___________________|_________________|_____________|________________|______________|________________|_______________|
 */


#endif /* FRAME_FORMAT_h */