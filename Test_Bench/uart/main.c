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

    while (1)
    {   
        TOF_Active_Decoding(fd); //Actively acquire TOF data and decode it 主动获取TOF数据，并进行
        // TOF_peek = serialGetchar(fd);      // Read a byte 读取一个字节
        // printf("%d tof_peek: %d\n", count, TOF_peek);
        delay(20);
        // count++;
    }
    return 0; 
}
