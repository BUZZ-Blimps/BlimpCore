/*****************************************************************************
 * | File      	:   TOF_Sense.c
 * | Author      :   Waveshare team
 * | Function    :   TOF drive function
 * | Info        :
 *----------------
 * |	This version:   V1.0
 * | Date        :   2024-09-11
 * | Info        :   Basic version
 *
 ******************************************************************************/
#include "TOF_Sense.hpp"

/******************************************************************************
function:	Actively acquire TOF data and decode it 主动获取TOF数据，并进行解码
parameter:
Info:
******************************************************************************/
int TOF_Sense::uart_setup(){
    if ((this->fd = serialOpen ("/dev/ttyS0", 921600)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }

    if (wiringPiSetup () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        return 1 ;
    }
    return 0;
}

void TOF_Sense::TOF_read()
{    
    TOF_peek = serialGetchar(this->fd);      // Read a byte 读取一个字节

    if (TOF_peek == TOF_FRAME_HEADER) // If it is a frame header, restart the loop count 如果是帧头,则重新开始循环计数
    {
        count_i = 0;
        rx_buf[count_i] = TOF_peek; // Store the read data into a tuple for later decoding 将读取到的数据存入元组中，用于后面解码使用
    }
    else
    {
        rx_buf[count_i] = TOF_peek; // Store the read data into a tuple for later decoding 将读取到的数据存入元组中，用于后面解码使用
    }
    count_i++;//Loop count +1 循环计数+1

    if (count_i > 15)//If the number of received data is greater than 15, the count variable can be cleared and a decoding can be performed. 接收数量大于15,则可以将计数变量清零并进行一次解码
    {
        count_i = 0;
        for (count_j = 0; count_j < 15; count_j++)
        {
            check_sum += rx_buf[count_j]; // Calculate the checksum and take the lowest byte 计算检验和并取最低一个字节
        }
        //Determine whether the decoding is correct 判断解码是否正确
        if ((rx_buf[0] == TOF_FRAME_HEADER) && (rx_buf[1] == TOF_FUNCTION_MARK) && (check_sum == rx_buf[15])) // 如果接收数组第一和第二个元素分别等于TOF_FRAME_HEADER和TOF_FUNCTION_MARK，且算出的校验和的低字节等于协议中的校验和，说明解码正确
        {
            this->id = rx_buf[3];                                                                                                                                                  // ID of the TOF module TOF 模块的 ID
            this->system_time = (unsigned long)(((unsigned long)rx_buf[7]) << 24 | ((unsigned long)rx_buf[6]) << 16 | ((unsigned long)rx_buf[5]) << 8 | (unsigned long)rx_buf[4]); // The time after the TOF module is powered on TOF模块上电后经过的时间
            this->dis = ((float)(((long)(((unsigned long)rx_buf[10] << 24) | ((unsigned long)rx_buf[9] << 16) | ((unsigned long)rx_buf[8] << 8))) / 256));                         // The distance output by the TOF module TOF模块输出的距离
            this->dis_status = rx_buf[11];                                                                                                                                         // Distance status indication output by TOF module TOF模块输出的距离状态指示
            this->signal_strength = (unsigned int)(((unsigned int)rx_buf[13] << 8) | (unsigned int)rx_buf[12]);                                                                    // The signal strength output by the TOF module TOF模块输出的信号强度
            this->range_precision = rx_buf[14];                                                                                                                                    // The repeatability accuracy reference value output by the TOF module is invalid for Type C, Type D and Mini. TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的

            //Print data through the terminal 通过终端打印数据
            // printf("TOF id is:%d\r\n", this->id);
            // printf("TOF system time is:%d ms\r\n", this->system_time);
            // printf("TOF distance is:%d mm\r\n", this->dis);
            // printf("TOF status is:%d\r\n", this->dis_status);
            // printf("TOF signal strength is:%d\r\n", this->signal_strength);
            // printf("TOF range precision is:%d\r\n\n", this->range_precision);
            serialFlush(this->fd);//Clear the serial port buffer 清空串口缓存
        }
        else
        {
            printf("Verification failed.\r\n");
        }
    }
    check_sum = 0; //Clear Checksum 清空校验和
}

// uint8_t tx_buf[8] = {0x57,0x10,0xff,0xff,0x00,0xff,0xff,0x63};//Query the command with ID 0 查询ID为0的命令


/******************************************************************************
function:	Get TOF data by querying and decoding 通过查询的方式获取TOF数据，并进行解码
parameter:
Info:
******************************************************************************/
// void TOF_Inquire_Decoding(uint8_t id)
// {
//     tx_buf[4] = id;//Add the ID you want to query to the command 将需要查询的ID添加到命令中
//     tx_buf[7] = id + 0x63;//Update Checksum 更新校验和

//     UART_flushInput();//Clear the serial port buffer 清空串口缓存
//     UART_Write_nByte(tx_buf,8);//Start query 开始查询
//     DEV_Delay_ms(10);//Waiting for the sensor to return data 等待传感器返回数据
//     UART_Read_nByte(rx_buf,16);//Reading sensor data 读取传感器数据
//     for (count_j = 0; count_j < 15; count_j++)
//     {
//         check_sum += rx_buf[count_j]; // Calculate the checksum and take the lowest byte 计算检验和并取最低一个字节
//     }
//     //Determine whether the decoding is correct 判断解码是否正确
//     if ((rx_buf[0] == TOF_FRAME_HEADER) && (rx_buf[1] == TOF_FUNCTION_MARK) && (check_sum == rx_buf[15])) // 如果接收数组第一和第二个元素分别等于TOF_FRAME_HEADER和TOF_FUNCTION_MARK，且算出的校验和的低字节等于协议中的校验和，说明解码正确
//     {
//         TOF_0.id = rx_buf[3];                                                                                                                                                  // ID of the TOF module TOF 模块的 ID
//         TOF_0.system_time = (unsigned long)(((unsigned long)rx_buf[7]) << 24 | ((unsigned long)rx_buf[6]) << 16 | ((unsigned long)rx_buf[5]) << 8 | (unsigned long)rx_buf[4]); // The time after the TOF module is powered on TOF模块上电后经过的时间
//         TOF_0.dis = ((float)(((long)(((unsigned long)rx_buf[10] << 24) | ((unsigned long)rx_buf[9] << 16) | ((unsigned long)rx_buf[8] << 8))) / 256));                         // The distance output by the TOF module TOF模块输出的距离
//         TOF_0.dis_status = rx_buf[11];                                                                                                                                         // Distance status indication output by TOF module TOF模块输出的距离状态指示
//         TOF_0.signal_strength = (unsigned int)(((unsigned int)rx_buf[13] << 8) | (unsigned int)rx_buf[12]);                                                                    // The signal strength output by the TOF module TOF模块输出的信号强度
//         TOF_0.range_precision = rx_buf[14];                                                                                                                                    // The repeatability accuracy reference value output by the TOF module is invalid for Type C, Type D and Mini. TOF模块输出的重复测距精度参考值，对于C型,D型和Mini型是无效的

//         //Print data through the terminal 通过终端打印数据
//         printf("TOF id is:%d\r\n", TOF_0.id);
//         printf("TOF system time is:%d ms\r\n", TOF_0.system_time);
//         printf("TOF distance is:%d mm\r\n", TOF_0.dis);
//         printf("TOF status is:%d\r\n", TOF_0.dis_status);
//         if (TOF_0.dis_status == 0)
//             printf("Measurement distance is invalid.\r\n");
//         else
//             printf("Measurement distance is valid.\r\n");
//         printf("TOF signal strength is:%d\r\n", TOF_0.signal_strength);
//         printf("TOF range precision is:%d\r\n\n", TOF_0.range_precision);
//     }
//     else
//     {
//         printf("Verification failed.\r\n");
//     }
//     check_sum = 0; // Clear Checksum 清空校验和
// }
