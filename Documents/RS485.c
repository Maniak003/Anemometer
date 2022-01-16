#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define SPEED B4800

u_int8_t receive_buf[200], send_buf[200];
int hpio, nbyte, len_data;
struct termios ti;

unsigned int CRC16(unsigned char *buf, int len) {  
  unsigned int crc = 0xFFFF;
  for (int pos = 0; pos < len; pos++) {
  crc ^= (unsigned int)buf[pos];    // XOR byte into least sig. byte of crc

  for (int i = 8; i != 0; i--) {    // Loop over each bit
    if ((crc & 0x0001) != 0) {      // If the LSB is set
      crc >>= 1;                    // Shift right and XOR 0xA001
      crc ^= 0xA001;
    }
    else                            // Else LSB is not set
      crc >>= 1;                    // Just shift right
    }
  }
  return crc;
}

int main(int argc, char *argv[]) {
	if (argc > 1) {
		if((hpio = open(argv[1], O_RDWR)) <= 0)	{
			printf("Error open %s...\n", argv[1]);
			exit(1);
		}
		else
			printf("Port %s open Ok\n", argv[1]);
		tcgetattr(hpio, &ti);
		ti.c_lflag&=(~(ICANON | ECHO | ISIG | TOSTOP));  // for feasycom
		//ti.c_cc[VMIN] = 1;
		//ti.c_cc[VTIME] = 0;
		//ti.c_cc[VLNEXT] = 0;
		//ti.c_cc[NCCS] = (~(VINTR | VQUIT | VERASE | VKILL | VEOF | VEOL | VEOL2 | VSTART\
		//	 | VSTOP | VSUSP | VREPRINT | VDISCARD | VWERASE | VLNEXT));
		//ti.c_iflag &= ~(ICRNL | IXON | IXOFF);
		ti.c_cflag = (ti.c_cflag & ~CBAUD ) | SPEED;
		tcsetattr(hpio, TCSANOW, &ti);

		/* Текущие скорость и направление ветра. */
		len_data = 8;
		send_buf[0] = 0x01;
		send_buf[1] = 0x03;
		send_buf[2] = 0x00;
		send_buf[3] = 0x00;
		send_buf[4] = 0x00;
		send_buf[5] = 0x02;
		send_buf[6] = 0xC4;
		send_buf[7] = 0x0B;

		write(hpio, send_buf, len_data);
		if((nbyte = read(hpio, receive_buf, sizeof(receive_buf))) > 0) {
			for (int ii; ii < nbyte; ii++) {
				printf("%d: %02x\n", ii, receive_buf[ii]);
			}
		}
		close(hpio);
		u_int16_t CRC = CRC16(receive_buf, receive_buf[2] + 3), CRC_RES;
		CRC_RES = (receive_buf[receive_buf[2] + 3]) | ((receive_buf[receive_buf[2] + 4]) << 8);
		if (CRC == CRC_RES) {
			printf("CRC is Ok.\n");
			float speed_wind = ((receive_buf[4]) | ((receive_buf[3]) << 8)) / 100.0;
			printf("Speed: %.2f m/s\n", speed_wind);
			float direct = ((receive_buf[6]) | ((receive_buf[5]) << 8));
			printf("Direct: %.0f\n", direct);
		} else {
			printf("CRC - wrong. CRC_RECEIVE: %04x, CRC_REAL: %04x\n", CRC_RES, CRC);
		}
	} else {
		printf("Usege: %s <TTY>\n", argv[0]);
	}
	
}

