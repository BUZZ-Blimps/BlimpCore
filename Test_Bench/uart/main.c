// #include <stdio.h>      //printf()
// #include <stdlib.h>     //exit()
// #include <signal.h>
// #include <stdio.h>
// #include <string.h>
#include "TOF_Sense.h"

int main(int argc, char **argv)
{
    int fd ;
    int count = 0;
    unsigned int nextTime ;
    int TOF_peek = 0;  


    if ((fd = serialOpen ("/dev/ttyS0", 921600)) < 0)
    {
        fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
        return 1 ;
    }

    if (wiringPiSetup () == -1)
    {
        fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
        return 1 ;
    }

    unsigned long loop_time = millis();
    int sample_count = 0;

    unsigned long sys_time = 0;

    while (1)
    {
        unsigned long sample_time = millis();

        TOF_Active_Decoding(fd); //Actively acquire TOF data and decode it 主动获取TOF数据，并进行

        // TOF_peek = serialGetchar(fd);      // Read a byte 读取一个字节
        // printf("%d tof_peek: %d\n", count, TOF_peek);

        // delay(20);

        // count++;

        if (TOF_0.system_time != sys_time) {
            sys_time = TOF_0.system_time;
            sample_count++;
        }

        if (sample_time - loop_time >= 1000) {
            printf("%d Hz\n", sample_count);
            sample_count = 0;
            loop_time = sample_time;
        }
    }
    return 0; 
}
