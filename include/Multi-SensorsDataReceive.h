#ifndef LASER_ODOM_UART_H
#define LASER_ODOM_UART_H

#include <condition_variable>
#include <thread>
#include <mutex>

#include <stdio.h>      /*标准输入输出定义*/
#include <stdlib.h>     /*标准函数库定义*/
#include <unistd.h>     /*Unix 标准函数定义*/
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>      /*文件控制定义*/
#include <termios.h>    /*PPSIX 终端控制定义*/
#include <errno.h>      /*错误号定义*/
#include <string>

using namespace std;
typedef int8_t         _s8;
typedef uint8_t        _u8;

typedef int16_t        _s16;
typedef uint16_t       _u16;

typedef int32_t        _s32;
typedef uint32_t       _u32;

typedef int64_t        _s64;
typedef uint64_t       _u64;

typedef struct _LidarPointStructDef{
    _u16 Distance;
    _u8 Confidence;
}__attribute__((packed)) LidarPointStructDef;

typedef struct _lds_response_measurement_node_t {
    _u16    sync_header;
    _u32    unknow;
    _u32    counter;
    _u16    Temperature; //当前温度值 
    _u16    CurrSpeed; //当前转速值 
    _u16    GivenSpeed;//设定的转速值 
    LidarPointStructDef data[360];  
     _u8    Checksum;//校验位 
    _u16    sync_tail;
}__attribute__((packed)) lds_response_measurement_node_t;

typedef struct _lds_response_measurement_node_odom_t {
    _u8    header;
    _u8    checksum1;
    _u8    checksum2;
    _u8    messagetype;
    _u8    length ;
    _u8    seq;
    _u8    ack;
    _s16   X;
    _s16   Y;
    _s32   Theta;
    _u8     tail;
    timeval timestamp;
}__attribute__((packed)) lds_response_measurement_node_odom_t;

//[ x, y, theta] data structure
typedef struct ODOM_DATA{
    timeval timestamp;
    int frameIndex;
    float X;
    float Y;
    float Theta;
}Odom_Data;

typedef struct LASER_DATA{
    timeval timestamp;
    int frameIndex;
    float Distance[360];
    float Angle[360];
    float TmpX[360];
    float TmpY[360];
}Laser_Data;

typedef struct LASER_ODOM_DATA{
    timeval timestamp;
    float Distance[360];
    float Angle[360];
    float TmpX[360];
    float TmpY[360];
    float OldX[360];
    float OldY[360];
    float X;
    float Y;
    float Theta;
}Laser_Odom_Data;

class LaserOdomUart {
public:
    LaserOdomUart();
    ~LaserOdomUart();

    //uart open port and init protocol, portNum, baudrate, databits, stopbits, parity, streamcontrol
    bool uartInit(const char*, const char*, int, int, int, int, const char, const char);

    //close uartFd
    void uartClose(void);

    void datareceive_laser(void);
    void datareceive_odom(void);

    int waitNode(lds_response_measurement_node_t *);
    int waitNode_odom(lds_response_measurement_node_odom_t *);
    int waitfordata(int, _u32, int *);
    int recvdata(unsigned char * , int);
    int waitfordata_odom(int , int *);
    int recvdata_odom(unsigned char * , int);
    bool _flag_certain;

    Laser_Odom_Data getCurrentData(void);

protected:
    Laser_Data _LaserData;
    Odom_Data _OdomData;
    Laser_Odom_Data _LaserOdomData;
    
    int _uartFd_laser;
    int _uartFd_odom;
    float _constDistance;
    float _constDiffX;
    float _constDiffY;
    float _constDiffTheta;
    int _angle;

    float _DeltaPose[3], _LastPose[3], _TmpPose[3], _recordPose[3];
    struct timeval _record;
    struct timeval _start_laser;
    struct timeval _end_laser;

    
    lds_response_measurement_node_t       _cached_scan_node_buf[60];
    lds_response_measurement_node_odom_t       _cached_scan_node_buf_odom[6];
    
    std::mutex _LaserDataMutex;
    std::mutex _OdomDataMutex;
    std::mutex _LaserOdomDataMutex;
    std::mutex _uartShutDownMutex;
    std::mutex _receivedateMutex;
    std::mutex _ThreadSyncMutex;
    std::mutex _receivedateodomMutex;
    
    bool _uartShutDownFlag;
};
#endif
