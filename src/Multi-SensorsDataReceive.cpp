#include "Multi-SensorsDataReceive.h"
#include "sys/time.h"
#include <cmath>
#include <assert.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <iostream>
#include <string.h>
#include <sys/ioctl.h>

#include <condition_variable>
#include <thread>
#include <mutex>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#define pi 3.1415926

LaserOdomUart::LaserOdomUart():_uartShutDownFlag(false), _uartFd_laser(-1), _uartFd_odom(-1) {
    cerr<<"The Version of this is 1.2"<<endl;
    _OdomData.X = 0;
    _OdomData.Y = 0;
    _OdomData.Theta = 0;
    _OdomData. frameIndex= -1;
    _OdomData. timestamp.tv_sec= 0;
    _OdomData. timestamp.tv_usec= 0;
    
    _LaserOdomData.X = 0;
    _LaserOdomData.Y = 0;
    _LaserOdomData.Theta = 0;
     _LaserOdomData.timestamp.tv_sec = 0;
    _LaserOdomData.timestamp.tv_usec = 0;
    
    _LaserData.timestamp.tv_sec = 0;
    _LaserData.timestamp.tv_usec = 0;
    
    _record.tv_sec = 0;
    _record.tv_usec = 0;
    _angle = 0;
    _flag_certain = false;

    for(int i=0; i<360; i++) {
        _LaserData.Distance[i] = 0;
        _LaserData.Angle[i] = 0;
	_LaserData.TmpX[i] = 0;
	_LaserData.TmpY[i] = 0;
	_LaserOdomData.Angle[i] = 0;
	_LaserOdomData.Distance[i] = 0;
	_LaserOdomData.TmpX[i] = 0;
	_LaserOdomData.TmpY[i] = 0;
	_LaserOdomData.OldX[i] = 0;
	_LaserOdomData.OldY[i] = 0;
    }
    
    _constDiffTheta = 0*pi/180;
    _constDistance = 0.122;
    _constDiffX = _constDistance*cos(_constDiffTheta);
    _constDiffY = _constDistance*sin(_constDiffTheta);

    for(int i=0; i<3; i++) {
      _DeltaPose[i] = 0;
      _LastPose[i] = 0;
      _TmpPose[i] = 0;
      _recordPose[i] = 0;
    }
    

    //Initlize the serials 
    if(!uartInit("/dev/ttyS1", "/dev/ttyS0", 115200, 115200, 8, 1, 'n', 'n')) {
        cerr<<"Open uarts failed!"<<endl;
    }
    
    std::thread t1(&LaserOdomUart::datareceive_laser, this);//创建一个分支线程，回调到datareceive_laser函数里
    std::thread t2(&LaserOdomUart::datareceive_odom, this);//创建一个分支线程，回调到datareceive_odom函数里
    t1.detach();  // the function is that separating the main with this sub-thread
    t2.detach();  // the function is that separating the main with this sub-thread
//     t1.join();    //the function is that going on the main after this sub-thread to be finshed    
}

LaserOdomUart::~LaserOdomUart() {
    LaserOdomUart::uartClose();
}

bool LaserOdomUart::uartInit(const char* comPort0, const char* comPort1, int baudRate0, int baudRate1, int dataBits, int stopBits, const char parity,const char streamControl) {
    _uartFd_laser = open(comPort0, O_RDWR|O_NOCTTY|O_NONBLOCK);//O_NDELAY);
    _uartFd_odom = open(comPort1, O_RDWR|O_NOCTTY|O_NONBLOCK);//O_NDELAY);

    if(_uartFd_laser == -1) {
        cerr<<"open port "<<comPort0<<" failed"<<endl;
        return false;
    }
    if(_uartFd_odom == -1) {
        cerr<<"open port "<<comPort1<<" failed"<<endl;
        return false;
    }
    
    int fd_laser = _uartFd_laser;
    int fd_odom = _uartFd_odom;

    int   speed_arr[] = { B230400, B115200, B19200, B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = { 230400, 115200,  19200,  9600,  4800,  2400,  1200,  300};
    struct termios options, options1;

    //存储目前的序列埠设定
    if  ( tcgetattr( fd_laser,&options) != 0 ) {
        cerr<<"error get uart attrib"<<endl;
        return false;
    }

    if  ( tcgetattr( fd_odom,&options1) != 0) {
        cerr<<"error get uart attrib"<<endl;
        return false;
    }
    
    //设置串口输入波特率和输出波特率
    for ( int i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++) {
        if  (baudRate0 == name_arr[i]) {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
	}
        if  (baudRate1 == name_arr[i]) {
	     cfsetispeed(&options1, speed_arr[i]);
             cfsetospeed(&options1, speed_arr[i]); 
	} 
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    options1.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;
    options1.c_cflag |= CREAD;

    //设置数据流控制
    switch(streamControl)
    {
    case 'n' ://不使用流控制
    case 'N':
        options.c_cflag &= ~CRTSCTS;	
        options1.c_cflag &= ~CRTSCTS;
        break;

    case  'h'://使用硬件流控制
    case 'H':
        options.c_cflag |= CRTSCTS;
        options1.c_cflag |= CRTSCTS;
        break;
    case 's' ://使用软件流控制
    case 'S':
        options.c_cflag |= IXON | IXOFF | IXANY;
        options1.c_cflag |= IXON | IXOFF | IXANY;
	break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    options1.c_cflag &= ~CSIZE;
    switch (dataBits)
    {
    case 5    :
        options.c_cflag |= CS5;
        options1.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        options1.c_cflag |= CS6;
	break;
    case 7    :
        options.c_cflag |= CS7;
	options1.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        options1.c_cflag |= CS8;
	break;
    default:
//         fprintf(stderr,"Unsupported data size\n");
        return false;
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        options1.c_cflag &= ~PARENB;
        options1.c_iflag &= ~INPCK;
	break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        options1.c_cflag |= (PARODD | PARENB);
        options1.c_iflag |= INPCK;
	break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        options1.c_cflag |= PARENB;
        options1.c_cflag &= ~PARODD;
        options1.c_iflag |= INPCK;
	break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        options1.c_cflag &= ~PARENB;
        options1.c_cflag &= ~CSTOPB;
	break;
    default:
//         fprintf(stderr,"Unsupported parity\n");
        return false;
    }
    // 设置停止位
    switch (stopBits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
        options1.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
        options1.c_cflag |= CSTOPB; break;
    default:
//         fprintf(stderr,"Unsupported stop bits\n");
        return false;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//original data input
    options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); //iflag not lflag
    
    options1.c_oflag &= ~OPOST;
    options1.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);//original data input
    options1.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON); //iflag not lflag

    //设置等待时间和最小接收字符
//    options.c_cc[VTIME] = 0; /* 读取一个字符等待1*(1/10)s */
//    options.c_cc[VMIN] = 0; /* 读取字符的最少个数为1 */ //13*2 byte

    //如果发生数据溢出，接收数据，但是不再读取, 刷新收到的数据但是不读
    tcflush(fd_laser,TCIFLUSH);
    tcflush(fd_odom,TCIFLUSH);
    if ((fcntl(fd_laser, F_SETFL, FNDELAY)) != 0 || fcntl(fd_odom, F_SETFL, FNDELAY) != 0) {
        uartClose();
        return false;
    }
    
    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd_laser,TCSANOW,&options) != 0 ||tcsetattr(fd_odom,TCSANOW,&options1) != 0) {
       perror("com set error!\n");
       return false;
    }
    return true;
}

void LaserOdomUart::uartClose(void) {  
    std::unique_lock<std::mutex> lck (_uartShutDownMutex);
    _uartShutDownFlag = true;
    if(_uartFd_laser != -1) {
	close(_uartFd_laser);
    }
    if(_uartFd_odom != -1) {
	close(_uartFd_odom);
    }
}

void LaserOdomUart::datareceive_laser(void) {
    lds_response_measurement_node_t node;

    float tmp_x = 0, tmp_y = 0, real_x = 0, real_y = 0;

    _u8 buff[15]={0xaa, 0xaa, 0x10, 0x09, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x01, 0x00, 0x1c, 0x55, 0x55};
    write(_uartFd_laser,buff,15);
//     tcflush(_uartFd_laser,TCIFLUSH); //clear out the buffer!
    while(1) {
	if(_uartShutDownFlag == true)   break;   //when receive the signal of uart shutting down, exiting the loop!

         gettimeofday(&_start_laser, NULL);
	  int result = waitNode(&node);
	 gettimeofday(&_end_laser, NULL);
      	  std::unique_lock<std::mutex> lk(_OdomDataMutex);
	  _TmpPose[2] = (_OdomData.Theta + _constDiffTheta)*pi/180.0;
	  _TmpPose[0] = cos(_constDiffTheta)*_OdomData.X - sin(_constDiffTheta)*_OdomData.Y + _constDiffX;
	  _TmpPose[1] =sin(_constDiffTheta)*_OdomData.X + cos(_constDiffTheta)*_OdomData.Y + _constDiffY;
	  lk.unlock();
	  
// 	  int timeuse1 = 1000000 * (_record.tv_sec - _start_laser.tv_sec ) + _record.tv_usec -_start_laser.tv_usec;
// 	  cerr<<"The first _diffTime is that  "<<timeuse1<<endl;
	  
	   _LastPose[2] = (_recordPose[2] + _constDiffTheta)*pi/180.0;
	   _LastPose[0] = cos(_constDiffTheta)*_recordPose[0] - sin(_constDiffTheta)*_recordPose[1] + _constDiffX;
	   _LastPose[1] =sin(_constDiffTheta)*_recordPose[0] + cos(_constDiffTheta)*_recordPose[1] + _constDiffY;
	  
	 _DeltaPose[0] = _TmpPose[0] - _LastPose[0]; 
	 _DeltaPose[1] = _TmpPose[1] - _LastPose[1];
	 _DeltaPose[2] = _TmpPose[2] - _LastPose[2];
//          cerr<<"Real diff is   "<< "                  "<<_DeltaPose[2]<<endl;
	 if(_DeltaPose[2]<-pi) {
	      _DeltaPose[2] += 2*pi;      
	 }
	 else if(_DeltaPose[2]>pi) {
	      _DeltaPose[2] -= 2*pi;
	 }

	 int timeuse = 1000000 * ( _end_laser.tv_sec - _start_laser.tv_sec ) + _end_laser.tv_usec -_start_laser.tv_usec;
// 	 cerr<<"Timeuse is   "<<timeuse<<endl;
	 if(timeuse < 160000 && timeuse > 100000) {
	    std::unique_lock<std::mutex> lk2(_LaserOdomDataMutex);
	    for(int i=0; i<360; i++) {
// 		  cerr<<"Tmp Point is   "<<i<<"    "<<node.data[359-i].Distance<<endl;
		  _LaserOdomData.TmpX[i] = node.data[359-i].Distance *cos(i*pi/180)/1000.0; 
		  _LaserOdomData.TmpY[i] = node.data[359-i].Distance *sin(i*pi/180)/1000.0; 
		  real_x = cos(i*_DeltaPose[2]/360)*_LaserOdomData.TmpX[i]  - sin(i*_DeltaPose[2]/360)*_LaserOdomData.TmpY[i]  + i*_DeltaPose[0]/360;
		  real_y = sin(i*_DeltaPose[2]/360)*_LaserOdomData.TmpX[i] + cos(i*_DeltaPose[2]/360)*_LaserOdomData.TmpY[i]  + i*_DeltaPose[1]/360;
// 		  cerr<<"Real Point is   "<<i<<"    X: "<<real_x<<"      Y: "<<real_y<<endl;		
		  _LaserOdomData.Angle[i] = atan2(real_y, real_x);
		  if(_LaserOdomData.Angle[i] < 0) _LaserOdomData.Angle[i] += 2*pi;
// 		  cerr<<"Real Point angle is   "<< i << "                  "<<_LaserOdomData.Angle[i]*180/pi<<endl;	
		  _LaserOdomData.Distance[i] = sqrt(pow(real_x, 2)+pow(real_y, 2));
	   }
	   for(int i=0; i<360; i++) {
		_LaserOdomData.TmpX[i] = 0;
		_LaserOdomData.TmpY[i] = 0;
	  }
	  for(int i=0; i<360; i++) {
	       _angle = _LaserOdomData.Angle[i]*180/pi;
	       _LaserOdomData.TmpX[_angle] =cos(_LaserOdomData.Angle[i])*_LaserOdomData.Distance[i];
	       _LaserOdomData.TmpY[_angle] =sin(_LaserOdomData.Angle[i])*_LaserOdomData.Distance[i];
// 	       _LaserOdomData.TmpX[_angle] =cos(_constDiffTheta)*_LaserOdomData.TmpX[_angle] - sin(_constDiffTheta)*_LaserOdomData.TmpY[_angle] - _constDiffX;
// 	       _LaserOdomData.TmpY[_angle] =sin(_constDiffTheta)*_LaserOdomData.TmpX[_angle] + cos(_constDiffTheta)*_LaserOdomData.TmpY[_angle] - _constDiffY;
	  }
	   
	    _LaserOdomData.X = _recordPose[0];
	    _LaserOdomData.Y = _recordPose[1];
	    _LaserOdomData.Theta = _recordPose[2];
	    _flag_certain = true;
	    lk2.unlock();
	}
    }
}

void LaserOdomUart::datareceive_odom(void) {
//     lds_response_measurement_node_odom_t local_buf_odom[2];
//     memset(local_buf_odom, 0, sizeof(local_buf_odom));
    lds_response_measurement_node_odom_t node_odom;
    tcflush(_uartFd_odom,TCIFLUSH); //clear out the buffer!
    while(1) {
	 if(_uartShutDownFlag == true) break;
	      waitNode_odom(&node_odom);
	      std::unique_lock<std::mutex> lk(_OdomDataMutex);
              //ascend the odom data!
	      _OdomData.timestamp = node_odom.timestamp;
	      _OdomData.X = node_odom.X/1000.0;
	      _OdomData.Y = node_odom.Y/1000.0;
	      _OdomData.Theta = node_odom.Theta/1000.0;
// 	      cerr <<_OdomData.X<<"        "<< _OdomData.Y <<"        "<<_OdomData.Theta  <<endl;
	      lk.unlock();
    }
}

int LaserOdomUart::waitNode(lds_response_measurement_node_t *node) {
       int  recvPos = 0;
       int count_ctrl = 0;
       _u8  recvBuffer[sizeof(lds_response_measurement_node_t)];
       _u8 *nodeBuffer = (_u8*)node;
       int remainSize = 0;
       _u32 time_out = 160;
       int recvSize, result = 0;
       
	std::unique_lock<std::mutex> lk0(_OdomDataMutex);
	_recordPose[0] = _OdomData.X;
	_recordPose[1] = _OdomData.Y;
	_recordPose[2] = _OdomData.Theta;
	_record = _OdomData.timestamp;
// 	cerr <<_OdomData.X<<"        " << _OdomData.Y <<"        "<<_OdomData.Theta  <<endl;
	lk0.unlock(); 
       while(1) {
            if(_uartShutDownFlag == true) break;
           remainSize = sizeof(lds_response_measurement_node_t) - recvPos;
	    result = waitfordata(remainSize, time_out, &recvSize);
	   if(result == -2) {
	     cerr<<"Something terrible happened!"<<endl;
	     return -2;
	   }
	   else if(result == -1) {
	      return -1;	     
	   }
           if (recvSize > remainSize) recvSize = remainSize;
           recvdata(recvBuffer, recvSize);

           for (int pos = 0; pos < recvSize; ++pos) {
                _u8 currentByte = recvBuffer[pos];
                switch (recvPos) {
		  case 0: {
		    if (currentByte == 0xAA) {}
		    else continue;
		  }
		  break;
                  case 1: {
                       if(currentByte ==0xAA)  {}
                       else {
                           recvPos = 0;
                           continue;
                       }
                  }
                  break;
		  case 6: {
                       if(currentByte == 0x3e)  {
		      }
                       else {
                           recvPos = 0;
                           continue;
                       }
                  }
		  break;
               }
               if(currentByte == 0xa5) {
		      if(recvBuffer[pos-1] == 0xa5) {
		  	   nodeBuffer[recvPos++] = currentByte;
		      }
	       }
	       else {
		    nodeBuffer[recvPos++] = currentByte;
		}
           }
           if (recvPos == sizeof(lds_response_measurement_node_t))   return 0;
     }
}

int LaserOdomUart::waitNode_odom(lds_response_measurement_node_odom_t *node_odom) {
       int  recvPos = 0;
       _u8  recvBuffer[sizeof(lds_response_measurement_node_odom_t) - sizeof(timeval)];
       _u8 *nodeBuffer_odom = (_u8*)node_odom;

       while(1) {
           if(_uartShutDownFlag == true) break;
           int remainSize = sizeof(lds_response_measurement_node_odom_t)  - sizeof(timeval) - recvPos;
           int recvSize;
           waitfordata_odom(remainSize,  &recvSize);
           if (recvSize > remainSize) recvSize = remainSize;
           recvdata_odom(recvBuffer, recvSize);

           for (int pos = 0; pos < recvSize; ++pos) {
               _u8 currentByte = recvBuffer[pos];
               switch (recvPos) {
		  case 0: {
			  if ( currentByte == 0x40) {}
			  else  continue;
		  }
		  break;
		  case 3: {
			  if ( currentByte == 0x43) {}
			  else {
			      recvPos = 0;
			      continue;
			  }
		  }
		  break;
		  case 15: {
			  if (currentByte == 0x2A) { } 
			  else {
			      recvPos = 0;
			      continue;
			  }
		  }
		  break;
	       }
               nodeBuffer_odom[recvPos++] = currentByte;
           }
           if (recvPos == sizeof(lds_response_measurement_node_odom_t) - sizeof(timeval))  {
	       gettimeofday(&node_odom->timestamp, NULL);
	       return 0; 
	   }
        }
}

int LaserOdomUart::waitfordata(int data_count, _u32 time_out, int * returned_size) {
    int length = 0;
    struct timeval timeout_val;
    if (returned_size==NULL) returned_size=(int *)&length;
    *returned_size = 0;

    int max_fd;
    fd_set input_set;
    FD_ZERO(&input_set);
    FD_SET(_uartFd_laser, &input_set);
    max_fd = _uartFd_laser + 1;
    
    /* Initialize the timeout structure */
    timeout_val.tv_sec = time_out / 1000;
    timeout_val.tv_usec = (time_out % 1000) * 1000;
    
    if ( ioctl(_uartFd_laser, FIONREAD, returned_size) == -1) return -1;
    if (*returned_size >= data_count) return 0;

    while(1) {
        if(_uartShutDownFlag == true) break;
        /* Do the select */
        int n = select(max_fd, &input_set, NULL, NULL, &timeout_val);
        if (n < 0)  return -2;
        else if (n == 0) {
// 	  cerr<<"just for test"<<endl;
	  return -1;
	}
	else {
	    assert (FD_ISSET(_uartFd_laser, &input_set));
	    if ( ioctl(_uartFd_laser, FIONREAD, returned_size) == -1) return -2;
	    if (*returned_size >= data_count) {
	      return 0;
	    }
	    else usleep(800);
	}	  
    }
    return -1;
}

int LaserOdomUart::waitfordata_odom(int data_count, int * returned_size) {
    int length = 0;
    if (returned_size==NULL) returned_size=(int *)&length;
    *returned_size = 0;

    fd_set input_set;
    FD_ZERO(&input_set);

    FD_SET(_uartFd_odom, &input_set);
    if ( ioctl(_uartFd_odom, FIONREAD, returned_size) == -1) return -1;
    if (*returned_size >= data_count) return 0;
    while(1) {
         assert (FD_ISSET(_uartFd_odom, &input_set));
         if ( ioctl(_uartFd_odom, FIONREAD, returned_size) == -1) return -1;
         if (*returned_size >= data_count) return 0;
         else usleep(300);
    }
    return -1;
}

int LaserOdomUart::recvdata(unsigned char * data, int size) {
    read(_uartFd_laser, data, size);
}

int LaserOdomUart::recvdata_odom(unsigned char * data, int size) {
    read(_uartFd_odom, data, size);
}

Laser_Odom_Data LaserOdomUart::getCurrentData(void) {
     std::unique_lock<std::mutex> lck(_LaserOdomDataMutex);
     _flag_certain = false;
     return _LaserOdomData;
}


