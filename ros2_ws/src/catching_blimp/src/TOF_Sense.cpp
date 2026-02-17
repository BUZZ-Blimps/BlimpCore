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

int TOF_Sense::uart_setup(){
  if ((this->fd = serialOpen ("/dev/ttyS0", 921600)) < 0)
  {
    fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    return 1 ;
  }

  if (wiringPiSetup () == -1)
  {
    fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    return 1;
  }
  return 0;
}

void TOF_Sense::TOF_read()
{
  while (serialDataAvail (this->fd)) {
    TOF_peek = serialGetchar(this->fd);

    if (TOF_peek == TOF_FRAME_HEADER)
    {
      count_i = 0;
      rx_buf[count_i] = TOF_peek;
    }
    else
    {
      rx_buf[count_i] = TOF_peek;
    }
    count_i++;

    if (count_i > 15)
    {
      count_i = 0;
      for (count_j = 0; count_j < 15; count_j++)
      {
        check_sum += rx_buf[count_j];
      }
      if ((rx_buf[0] == TOF_FRAME_HEADER) && (rx_buf[1] == TOF_FUNCTION_MARK) && (check_sum == rx_buf[15]))
      {
        this->id = rx_buf[3];
        this->system_time = (unsigned long)(((unsigned long)rx_buf[7]) << 24 | ((unsigned long)rx_buf[6]) << 16 | ((unsigned long)rx_buf[5]) << 8 | (unsigned long)rx_buf[4]);
        this->dis = ((float)(((long)(((unsigned long)rx_buf[10] << 24) | ((unsigned long)rx_buf[9] << 16) | ((unsigned long)rx_buf[8] << 8))) / 256));
        this->dis_status = rx_buf[11];
        this->signal_strength = (unsigned int)(((unsigned int)rx_buf[13] << 8) | (unsigned int)rx_buf[12]);
        this->range_precision = rx_buf[14];
        serialFlush(this->fd);
      }
      else
      {
        printf("Verification failed.\r\n");
      }
    }
    check_sum = 0;
  }
}
