#include "TOF_Sense.hpp"

int main()
{
    TOF_Sense lidar;
    lidar.uart_setup();

    while(1){
        lidar.TOF_read();
        printf("TOF id is:%d\r\n", lidar.id);
        printf("TOF system time is:%d ms\r\n", lidar.system_time);
        printf("TOF distance is:%d mm\r\n", lidar.dis);
        printf("TOF status is:%d\r\n", lidar.dis_status);
        printf("TOF signal strength is:%d\r\n", lidar.signal_strength);
        printf("TOF range precision is:%d\r\n\n", lidar.range_precision);
    }
    return 0;
}
