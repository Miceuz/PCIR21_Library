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
#ifndef __PCIR21_H
#define __PCIR21_H

#define RX_BUF_LEN 267

class PCIR21 {
	public:
		enum Fps_t {
		  FPS_05,
		  FPS_1,
		  FPS_2,
		  FPS_3
		};

		enum Range_t {
		  RANGE_FULL,
		  RANGE_BODY
		};

		enum EvalMode_t {
		  MODE_COMMAND,
		  MODE_EVALUATE,
		  GET_MODE
		};

		enum Mode_t {
			MODE_STOP,
			MODE_OPEN,
			MODE_SINGLE_FRAME
		};

		enum FrameMode_t {
			MODE_SINGLE,
			MODE_CONTINUOUS
		};
		PCIR21(Uart* port);
		~PCIR21(){};
		void reset(uint32_t pin);
		void query_version();
		void eval_mode(EvalMode_t mode);
		void set_fps(Fps_t fps);
		void set_mode(Mode_t mode);
		void set_frame_mode(FrameMode_t mode);
		void read_data(float* temperature);
		void set_range(Range_t range);
		void sleep();

	private:
		uint8_t rx_buff[RX_BUF_LEN];
		float temp[16*4+1];
		Uart* serial;
		void read_response();

		void read_pixel_data();
		float calculate_temperature();
		bool data_header_valid(uint32_t packet_length);
};
#endif
