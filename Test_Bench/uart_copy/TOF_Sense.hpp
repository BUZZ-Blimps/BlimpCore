#ifndef _TOF_SENSE_HPP_
#define _TOF_SENSE_HPP_

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdint.h>
#include <wiringPi.h>
#include <wiringSerial.h>

#define TOF_FRAME_HEADER 0x57//定义TOFSense系列和TOFSense-F系列的帧头
#define TOF_FUNCTION_MARK 0x00//定义TOFSense系列和TOFSense-F系列的功能码


class TOF_Sense{
  public:
    uint8_t id;//TOF模块的id
    uint32_t system_time;//TOF模块上电后经过的时间，单位：ms
    uint32_t dis;//TOF模块输出的距离，单位：m
    uint8_t dis_status;//TOF模块输出的距离状态指示
    uint16_t signal_strength;//TOF模块输出的信号强度
    uint8_t range_precision;//TOF模块输出的重复测距精度参考值，TOFSense-F系列有效，单位：cm
    int fd;

    void TOF_read();
    int uart_setup();
  private:
    uint8_t count_i = 0, count_j = 0; // Loop count variable 循环计数变量
    uint8_t check_sum = 0;            // Checksum 校验和
    uint8_t rx_buf[16];               // Serial port receiving array 串口接收数组
    uint8_t TOF_peek = 0;             // Temporary storage of data 临时存放数据

};


#endif