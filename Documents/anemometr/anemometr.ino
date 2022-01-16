
#include <AltSoftSerial.h>
#include <SPI.h>
#include <Ethernet.h>

// AltSoftSerial always uses these pins:
//
// Board          Transmit  Receive   PWM Unusable
// -----          --------  -------   ------------
// Arduino Uno        9         8         10

//#define DEB
#define SERIAL_OUT
#define LEN_DATA 8
#define ZABBIXPORT 10051      // Zabbix erver Port
#define ZABBIXMAXLEN 128
#define ZABBIXAGHOST "Ed"  // Zabbix item's host name
#define ZABBIXSENDPERIOD 300 // Period in secoonds
#define wdt_on true

#if wdt_on
#include <avr/wdt.h>
#endif

#if defined(WIZ550io_WITH_MACADDRESS) // Use assigned MAC address of WIZ550io
  ;
#else
  byte mac[] = { 0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x32 };
#endif

AltSoftSerial RS485;
uint8_t send_buf[10], rec_buf[32];
char buf[4];
int nbyte, index;
float SPD, DIR;
int len;
uint8_t res[ZABBIXMAXLEN];
EthernetClient client;
IPAddress server(192, 168, 1, 6); // Zabbix server IP.
//IPAddress ip(192, 168, 1, 19);

void sendToZabbix(String key, float val) {
  String str;

  for (int ii = 0; ii < ZABBIXMAXLEN; ii++) {
    res[ii] = 0;
  }
  str = "";
  str = str + "{\"request\":\"sender data\",\"data\":[{\"host\":\"";
  str = str + ZABBIXAGHOST;
  str = str + "\",\"key\":\""; 
  str = str + key;
  str = str + "\",\"value\":\"";
  str = str + val;
  str = str + "\"}]}";
  len = str.length();
  res[0] = 'Z';
  res[1] = 'B';
  res[2] = 'X';
  res[3] = 'D';
  res[4] = 0x01;
  res[5] = (byte) len;
  res[6] = 0;
  res[7] = 0;
  res[8] = 0;
  res[9] = 0;
  res[10] = 0;
  res[11] = 0;
  res[12] = 0;
  str.getBytes(&(res[13]), ZABBIXMAXLEN - 12);
  #ifdef SERIAL_OUT
  Serial.print(" Len: ");
  Serial.print(len);
  Serial.print(" ");
  Serial.print(str);
  //for( int i = 0; i < len + 13; i++) {
  //  Serial.print((char)(res[i]));
  //}
  Serial.println();
  #endif
  len = len + 13;
  if (client.connected()) {
    client.write(res, len);
  } else {
    if (client.connect(server, ZABBIXPORT)) {
      client.write(res, len);
    } else {
      #ifdef SERIAL_OUT
      Serial.println("Not connect. Reset client.");
      #endif
      client.stop();
    }      
  }
  client.stop();
}

void setup() {
  #if wdt_on
  wdt_disable();
  #endif
  #ifdef SERIAL_OUT
  Serial.begin(9600); 
  while (!Serial);
  Serial.println("Start.");
  #endif

  RS485.begin(4800);
  
  send_buf[0] = 0x01;
  send_buf[1] = 0x03;
  send_buf[2] = 0x00;
  send_buf[3] = 0x00;
  send_buf[4] = 0x00;
  send_buf[5] = 0x02;
  send_buf[6] = 0xC4;
  send_buf[7] = 0x0B;
  /*
  send_buf[0] = 0x01;
  send_buf[1] = 0x03;
  send_buf[2] = 0x00;
  send_buf[3] = 0x01;
  send_buf[4] = 0x00;
  send_buf[5] = 0x01;
  send_buf[6] = 0xD5;
  send_buf[7] = 0xCA;
  */
  //Serial.println(CRC16(send_buf, 6));
  #if wdt_on
    wdt_enable(WDTO_8S);
  #endif
  
  #if defined(WIZ550io_WITH_MACADDRESS) // Use assigned MAC address of WIZ550io
    if (Ethernet.begin() == 0) {
  #else
    if (Ethernet.begin(mac) == 0) {
  #endif
  #ifdef SERIAL_OUT
      Serial.println("Ethernet error.");
  #endif
      for(;;);
    }
    
  #ifdef SERIAL_OUT
  Serial.print("IP addr: ");
  Serial.println(Ethernet.localIP());
  #endif
}

void loop() {
  #if wdt_on
    wdt_reset();
  #endif
  for (int i = 0; i < sizeof(rec_buf); i++) {
    rec_buf[i] = 0;
  }

  for (int ii = 0; ii < LEN_DATA; ii++) {
    RS485.write(send_buf[ii]);
  }

  index = 0;
  while ((nbyte = RS485.available()) > 0) {
    rec_buf[index++] = RS485.read();
  }
  #ifdef DEB
  for (int ii = 0; ii < index; ii++) {
    Serial.print(ii);
    Serial.print(" 0x");
    sprintf(buf, "%02x", rec_buf[ii]);
    Serial.print(buf);
    Serial.print(", ");
  }
  Serial.println();
  #endif

  if (rec_buf[0] == 1 and rec_buf[1] == 3 and rec_buf[2] == 4) {
    uint16_t CRC_REC, CRC = CRC16(rec_buf, 7);
    CRC_REC = rec_buf[7] + (rec_buf[8] << 8);
    if (CRC_REC == CRC) {
      SPD = (float) (rec_buf[4] + (rec_buf[3] << 8)) / 100.0;
      DIR =  (float) (rec_buf[6] + (rec_buf[5] << 8));
      #ifdef DEB
      Serial.print("CRC16: ");
      Serial.print(CRC_REC);
      Serial.print(" CRC16: ");
      Serial.print(CRC);
      #endif
      #ifdef SERIAL_OUT
      Serial.print(" Speed: ");
      Serial.print(SPD);
      Serial.print(" Direct: ");
      Serial.println(DIR);
      #endif
      //sendToZabbix("ALTIM_SPEED", SPD);
      if (SPD > 0) {
        //sendToZabbix("ALTIM_DIRECT", DIR);
      }
    } else {
      #ifdef SERIAL_OUT
      Serial.print("Error CRC16. rec:");
      Serial.print(CRC_REC);
      Serial.print(", act: ");
      Serial.println(CRC);
      #endif
    }
  }
  
  delay(5000);
}

/* CRC16 Modbus */
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
