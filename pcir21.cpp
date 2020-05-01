#include <Arduino.h>

#include "pcir21.h"

void PCIR21::reset(uint32_t reset_pin) {
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, LOW);
  delay(100);
  digitalWrite(reset_pin, HIGH);
  delay(10);
  read_response();//digest all esp32 startup traffic
  delay(1000);
}

//just ignore the response for now
//TODO check if command was successful
void PCIR21::read_response() {
    delayMicroseconds(50);//50us > 10 bits at 230400 baud 
    while(serial->available()){
      // char c = serial->read();
      // if(c > 31 && c < 127)
      //   Serial->print(c);
      // else {
      //   Serial->print("0x");
      //   Serial->print(c, HEX);
      // }
      // Serial->print(' ');
      delayMicroseconds(50);
    }
    // Serial.println();
}

PCIR21::PCIR21(Uart* port): 
  serial(port)
{
  serial->begin(230400);
}

void PCIR21::query_version() {
  serial->print("CMDV");
  serial->print('\x00');
  serial->print('\x23');
  read_response();
}

void PCIR21::eval_mode(EvalMode_t mode) {
  uint8_t crc[]={0x1A, 0x19, 0x1B};
  serial->print("CMDE");
  serial->print((char) mode);
  serial->print((char) crc[mode]);
  read_response();
}

void PCIR21::set_fps(Fps_t fps) {
  uint8_t crc[]={0x1A, 0x1B, 0x1C, 0x1D};
  serial->print("CMDF");
  serial->print((char) fps);
  serial->print((char) (crc[fps]));
  read_response();
}

void PCIR21::set_mode(Mode_t mode) {
  uint8_t crc[]={0x19, 0x18, 0x1A};
  serial->print("CMDC");
  serial->print((char) mode);
  serial->print((char) crc[mode]);
  read_response();
}

void PCIR21::set_frame_mode(FrameMode_t mode) {
  uint8_t crc[]={0x21, 0x22};
  serial->print("CMDM");
  serial->print((char) mode);
  serial->print((char) (crc[mode]));
  read_response();
}

void PCIR21::set_range(Range_t range) {
  uint8_t crc[]={0x23, 0x24};
  serial->print("CMDO");
  serial->print((char) range);
  serial->print((char) (crc[range]));
  read_response();
}

void PCIR21::sleep() {
  serial->print("CMDS");
  serial->print('\x01');
  serial->print('\x28');
  read_response();
}

void PCIR21::read_pixel_data() {
  int i = 0;
  //we skip data one length here, start from 5
  //also at the end is 4 bytes of CRC and \r\n which we ignore
  for(int j = 5; j < RX_BUF_LEN - 6; j+=4) {
    uint32_t t = ((uint32_t)rx_buff[j+3] << 24) | 
                 ((uint32_t)rx_buff[j+2] << 16) | 
                 ((uint32_t)rx_buff[j+1] << 8) | 
                 ((uint32_t)rx_buff[j+0] & 0x000000FF);
    float f;
    memcpy(&f, &t, 4);
    temp[i++] = f;
  }
}

float PCIR21::calculate_temperature() {
  float max = 0;
  for(int i = 1; i < 16*4+1; i++) {
    if(temp[i] > max) {
      max = temp[i];
    }
  }
  return max;
}

bool PCIR21::data_header_valid(uint32_t packet_length) {
  uint32_t data_length  = 0;
  
  if(packet_length > 4) {
    data_length = rx_buff[3] << 8 * rx_buff[4];
  }

  return RX_BUF_LEN == packet_length && 'D' == rx_buff[0] && 'A' == rx_buff[1] && 'A' == rx_buff[2] && 64 == data_length;
}

//Frame format
//DAT[data lenght, 2 bytes mbs lsb][ambient temperature, 4 bytes float][pixel data, 4 bytes float]*16*4
void PCIR21::read_data(float* temperature) {
  if(serial->available()) {
    uint32_t i = 0;
    //TODO make it non-blocking, read one char at a time.
    while(serial->available()){
      if(i < RX_BUF_LEN) {
        rx_buff[i++] = serial->read();
      }
      delayMicroseconds(50);//slack off one byte at 230400 baud. 
    }
    
    if(data_header_valid(i)) {
      read_pixel_data();
      *temperature = calculate_temperature();
    }
  }
}
