/*
Copyright 2020, Albertas Mickenas, albertas@technarium.lt

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <https://www.gnu.org/licenses/>.
*/
#include <Arduino.h>

#include "pcir21.h"

PCIR21::PCIR21(Uart& port) : _serial(port) {}

void PCIR21::setup()
{
  _serial.begin(230400);
  while(!_serial);
}

void PCIR21::reset(uint32_t reset_pin) {
  pinMode(reset_pin, OUTPUT);
  digitalWrite(reset_pin, LOW);
  delay(100);
  digitalWrite(reset_pin, HIGH);
  while(!_serial.available());
  while(_serial.available()) {
    Serial.write(_serial.read());
  }  
  // read_response();//digest all esp32 startup traffic
  delay(2000);
  // read_response();//digest all esp32 startup traffic
  while(!_serial.available());
  while(_serial.available()) {
    Serial.write(_serial.read());
  }  
  Serial.println();
  Serial.println("--------------------------------------");
  Serial.println();
}

//just ignore the response for now
//TODO check if command was successful
//PCIR21 response format
//On success: RET[the command being acknowldged]\r\n Example: RETCMDE\x00\x1A\r\n
//On error: RETERR[the command being acknowldged]\r\n] Example: RETERRCMDE\x00\x1A\r\n
void PCIR21::read_response() {
  uint32_t bytesRead = _serial.readBytesUntil('\n', rx_buff, 14);
  if(bytesRead > 0) {
    rx_buff[bytesRead] = 0;
    // Serial.println((char *)rx_buff);
    for (int i = 0; i < bytesRead; i++) {
      char c = rx_buff[i];  
      if(c > 31 && c < 127)
        Serial.print(c);
      else {
        Serial.print("0x");
        Serial.print(c, HEX);
      }
      Serial.print(' ');
    }
    Serial.println();
  }
  // while(!_serial.available());  
  // Serial.println("read response:");
  // while(_serial.available()){
  //   char c = _serial.read();
  //   if(c > 31 && c < 127)
  //     Serial.print(c);
  //   else {
  //     Serial.print("0x");
  //     Serial.print(c, HEX);
  //   }
  //   Serial.print(' ');
  //   // delay(1);
  //   delayMicroseconds(500);//50us > 10 bits at 230400 baud 
  // }
  // Serial.println();
}

void PCIR21::query_version() {
  _serial.print("CMDV");
  _serial.print('\x00');
  _serial.print('\x23');
  read_response();
}

void PCIR21::eval_mode(EvalMode_t mode) {
  uint8_t crc[]={0x1A, 0x19, 0x1B};
  _serial.print("CMDE");
  _serial.print((char) mode);
  _serial.print((char) crc[mode]);
  read_response();
}

void PCIR21::set_fps(Fps_t fps) {
  uint8_t crc[]={0x1A, 0x1B, 0x1C, 0x1D};
  _serial.print("CMDF");
  _serial.print((char) fps);
  _serial.print((char) (crc[fps]));
  read_response();
}

void PCIR21::set_mode(Mode_t mode) {
  uint8_t crc[]={0x19, 0x18, 0x1A};
  _serial.print("CMDC");
  _serial.print((char) mode);
  _serial.print((char) crc[mode]);
  read_response();
}

void PCIR21::set_frame_mode(FrameMode_t mode) {
  uint8_t crc[]={0x21, 0x22};
  _serial.print("CMDM");
  _serial.print((char) mode);
  _serial.print((char) (crc[mode]));
  read_response();
}

void PCIR21::set_range(Range_t range) {
  uint8_t crc[]={0x23, 0x24};
  _serial.print("CMDO");
  _serial.print((char) range);
  _serial.print((char) (crc[range]));
  read_response();
}

void PCIR21::sleep() {
  _serial.print("CMDS");
  _serial.print('\x01');
  _serial.print('\x28');
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

bool PCIR21::is_data_header_valid(uint32_t packet_length) {
  uint32_t data_length  = 0;
  
  if(RX_BUF_LEN == packet_length) {
    data_length = rx_buff[3] << 8 | rx_buff[4];
    return 'D' == rx_buff[0] && 'A' == rx_buff[1] && 'T' == rx_buff[2] && 64 == data_length;
  } else {
    return false;
  }
}

//Frame format
//DAT[data lenght, 2 bytes mbs lsb][ambient temperature, 4 bytes float][pixel data, 4 bytes float]*16*4
bool PCIR21::read_data(float* temperature) {
  // while(!_serial.available());
  if(_serial.available()) {
    //Frame transmission should take 11.5ms @ 230400
    //Fasters refresh rate is 3 FPS - i.e. frame every 333s
    //Reasonable interframe time is ~320ms
    //We timeout at frame time + 1/3 of interframe time so PCIR21 has time to push the data 
    _serial.setTimeout(320/3+20);
    uint32_t bytesRead = _serial.readBytes(rx_buff, RX_BUF_LEN);   
    // Serial.print("Got DATA: ");
    // Serial.println(bytesRead);    

    // for (int i = 0; i < bytesRead; i++) {
    //   char c = rx_buff[i];  
    //   if(c > 31 && c < 127)
    //     Serial.print(c);
    //   else {
    //     Serial.print("0x");
    //     Serial.print(c, HEX);
    //   }
    //   Serial.print(' ');
    // }
    // Serial.println();

    // Serial.print("Header valid: ");
    // Serial.println(is_data_header_valid(bytesRead));

    if(is_data_header_valid(bytesRead)) {
      read_pixel_data();
      *temperature = calculate_temperature();
      return true;
    } else {
      // Serial.print("Dropping data: ");
      // Serial.println(_serial.available());
      while(_serial.available()) {
        _serial.read();
      }
      //if we got garbade, poll serial for some time to hopefully 
      //catch a gap between frames we can synchronize on
      // uint32_t bytesRead = _serial.readBytes(rx_buff, RX_BUF_LEN);
    }
  }
  return false;
}
/*
void pcir_setup() {
  _serial.begin(230400);
  while(!_serial);
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(11, PIO_SERCOM);
}

void pcir_reset() {
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  delay(100);
  digitalWrite(8, HIGH);
  delay(10);
  pcir_read_response();
  delay(1000);
}

void pcir_read_response() {
    Serial.println("Read response:");
    delay(1);
    while(_serial.available()){
      char c = _serial.read();
      if(c > 31 && c < 127)
        Serial.print(c);
      else {
        Serial.print("0x");
        Serial.print(c, HEX);
      }
      Serial.print(' ');
      delay(1);
    }
    Serial.println();
}

void pcir_operate() {
  _serial.print("CMDE");
  _serial.print('\x00');
  _serial.print('\x19');
}

void pcir_set_fps(int fps) {
  uint8_t crc[]={0x1A, 0x1B, 0x1C, 0x1D};
  _serial.print("CMDF");
  _serial.print((char) fps);
  _serial.print((char) (crc[fps]));
}

void pcir_open() {
  _serial.print("CMDC");
  _serial.print('\x01');
  _serial.print('\x18');
}

uint8_t rx_buff[RX_BUF_LEN];
float temp[16*4+1];

void pcir_read_data(float* temperature) {
  if(_serial.available()) {
    int i = 0;
    while(_serial.available()){
      rx_buff[i++] = _serial.read();
      delay(1);
    }
    Serial.print("Got data ");
    Serial.println(i);
    
    if(267 == i && 'D' == rx_buff[0]) {
      float* p = (float*)((char*)rx_buff + 5);
      float env_temp = *p;
      Serial.print("environment temperature: ");
      Serial.println(env_temp);
      int i = 0;

      Serial.print("Length:");
      Serial.println((rx_buff[3]<<8) | (rx_buff[4])); 

//      for(int i = 0; i < 16*4; i++) {
//        float f = *(p+i);
//        temp[i++] = f;
//      }
      for(int j = 5; j < 261; j+=4) {
        uint32_t t = ((uint32_t)rx_buff[j+3] << 24) | ((uint32_t)rx_buff[j+2] << 16) | ((uint32_t)rx_buff[j+1] << 8) | ((uint32_t)(rx_buff[j+0]) & 0x000000FF);
        float f;
        memcpy(&f, &t, 4);
        temp[i++] = f;
      }
      float sum = 0;
      for(int i = 1; i < 16*4+1; i++) {
        sum += temp[i];
      }
      *temperature = sum/64;
      Serial.println(*temperature);
    }
  }
}

*/