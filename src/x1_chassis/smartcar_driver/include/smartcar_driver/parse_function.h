#ifndef PARSE_FUNCTION_H  //  防止parse_function.h文件被重复引用
#define PARSE_FUNCTION_H

//Motorola格式解析函数
static uint32_t Analysis_Mot_Signal(uint8_t data[8], uint8_t start_bit, uint8_t signal_len)
{
	if ((signal_len + (start_bit % 8)) > 8) //跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t start_byte_len = 8 - (start_bit % 8);
		int8_t remaind_len = signal_len - start_byte_len;
		int8_t totoal_bytes = 1 + ((remaind_len - 1) / 8) + 1;
		int8_t end_byte = start_byte - totoal_bytes + 1;
		int8_t end_byte_len = (remaind_len - 1) % 8 + 1;
		int8_t end_bit = end_byte * 8 + end_byte_len - 1;

		uint8_t end_byte_val = data[end_byte] & (0xff >> (8 - end_byte_len));
		uint8_t start_byte_val = (data[start_byte] >> (start_bit % 8)) & ((1 << signal_len) - 1);//只把要解析的数据长度提取出来
		uint8_t left_move_cnt = signal_len - end_byte_len;
		uint32_t result = end_byte_val << left_move_cnt;

		for (uint8_t mid_byte = end_byte + 1; mid_byte < start_byte; ++mid_byte) {
			left_move_cnt -= 8;
			result += (data[mid_byte] << left_move_cnt);
		}

		result += start_byte_val;
		return result;
	}
	else //不跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t end_bit = (start_bit % 8) + signal_len - 1;

		uint8_t start_byte_val = (data[start_byte] >> (start_bit % 8)) & ((1 << signal_len) - 1);
		return start_byte_val;
	}
}
//Motorola格式打包函数
static void Gather_Mot_Signal(uint8_t data[8], uint8_t start_bit, uint8_t signal_len, uint32_t value)
{
	if ((signal_len + (start_bit % 8)) > 8) //跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t start_byte_len = 8 - (start_bit % 8);
		int8_t remaind_len = signal_len - start_byte_len;
		int8_t totoal_bytes = 1 + ((remaind_len - 1) / 8) + 1;
		int8_t end_byte = start_byte - totoal_bytes + 1;
		int8_t end_byte_len = (remaind_len - 1) % 8 + 1;
		int8_t end_bit = end_byte * 8 + end_byte_len - 1;

		uint8_t right_move_cnt = signal_len - end_byte_len;
		data[end_byte] &= (0xff << end_byte_len);  ////跨字节：只将目标位置的数据清零，其他位置的不变
		data[end_byte] |= (value >> right_move_cnt);

		for (uint8_t mid_byte = end_byte + 1; mid_byte < start_byte; ++mid_byte) {
			right_move_cnt -= 8;
			data[mid_byte] = (value >> right_move_cnt);
		}

		data[start_byte] &= (0xff >> signal_len);//跨字节：只将目标位置的数据清零，其他位置的不变
		data[start_byte] |= (value << (start_bit % 8));
	}
	else //不跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t start_byte_len = 8 - (start_bit % 8);
		int8_t remaind_len = signal_len - start_byte_len;
		int8_t totoal_bytes = 1 + ((remaind_len - 1) / 8) + 1;
		int8_t end_byte = start_byte + totoal_bytes - 1;
		int8_t end_byte_len = (remaind_len - 1) % 8 + 1;
		int8_t end_bit = end_byte * 8 + end_byte_len - 1;

		data[start_byte] &= ((0xff << ((end_bit % 8) + 1)) + ((1 << (start_bit % 8)) - 1));//不跨字节：只将目标位置的数据清零，其他位置的不变
		data[start_byte] |= (value << (start_bit % 8));
	}
}

//Intel格式解析函数
static int32_t Analysis_Intel_Signal(uint8_t data[8], uint8_t start_bit, uint8_t signal_len, bool is_signed = false)
{
	if ((signal_len + (start_bit % 8)) > 8) //跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t start_byte_len = 8 - (start_bit % 8);
		int8_t remaind_len = signal_len - start_byte_len;
		int8_t totoal_bytes = 1 + ((remaind_len - 1) / 8) + 1;
		int8_t end_byte = start_byte + totoal_bytes - 1;
		int8_t end_byte_len = (remaind_len - 1) % 8 + 1;
		int8_t end_bit = end_byte * 8 + end_byte_len - 1;

		uint8_t end_byte_val = data[end_byte] & (0xff >> (8 - end_byte_len));
		uint8_t start_byte_val = data[start_byte] >> (start_bit % 8) & ((1 << signal_len) - 1);

		uint8_t left_move_cnt = signal_len - end_byte_len;
    int32_t result = end_byte_val << left_move_cnt;

		for (int mid_byte = end_byte - 1; mid_byte > start_byte; --mid_byte) {
			left_move_cnt -= 8;
			result += (data[mid_byte] << left_move_cnt);
		}

		result += start_byte_val;
		
    if(is_signed){
        if(result & (1 << (signal_len-1))){  // 1 << (signal_len-1))-取result的符号位（为1则表示负数，为0则表示正数）
            return result - (1 << (signal_len));  // 高端方法
		    }
		
		}
		
		return result;
	}
	else //不跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t end_bit = (start_bit % 8) + signal_len - 1;

		uint8_t start_byte_val = (data[start_byte] >> (start_bit % 8)) & ((1 << signal_len) - 1); 

    if(is_signed){
        if(start_byte_val & (1 << (signal_len-1))){  // 1 << (signal_len-1))-取result的符号位（为1则表示负数，为0则表示正数）
            return start_byte_val - (1 << (signal_len));  // 同样的原码，正数的值 + 负数的值 = 2^len（1 << (signal_len)）
        }
    }

		return start_byte_val;
	}
}


//Inte格式打包函数
static void Gather_Intel_Signal(uint8_t data[8], uint8_t start_bit, uint8_t signal_len, int32_t value, bool is_signed = false)
{
	if ((signal_len + (start_bit % 8)) > 8) //跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t start_byte_len = 8 - (start_bit % 8);
		int8_t remaind_len = signal_len - start_byte_len;
		int8_t totoal_bytes = 1 + ((remaind_len - 1) / 8) + 1;
		int8_t end_byte = start_byte + totoal_bytes - 1;
		int8_t end_byte_len = (remaind_len - 1) % 8 + 1;
		int8_t end_bit = end_byte * 8 + end_byte_len - 1;

    if(is_signed){
      if(value < 0){
        value += (1 << signal_len);  // 【【【求得这个负数对应的正数即可！】】】
      }
    }

		uint8_t right_move_cnt = signal_len - end_byte_len;
		data[end_byte] &= (0xff << end_byte_len);
		data[end_byte] |= (value >> right_move_cnt);

		for (int mid_byte = end_byte - 1; mid_byte > start_byte; --mid_byte) {
			right_move_cnt -= 8;
			data[mid_byte] = (value >> right_move_cnt);
		}

		data[start_byte] &= (0xff >> signal_len);
		data[start_byte] |= (value << (start_bit % 8));
	}
	else //不跨字节
	{
		int8_t start_byte = start_bit / 8;
		int8_t start_byte_len = 8 - (start_bit % 8);
		int8_t remaind_len = signal_len - start_byte_len;
		int8_t totoal_bytes = 1 + ((remaind_len - 1) / 8) + 1;
		int8_t end_byte = start_byte + totoal_bytes - 1;
		int8_t end_byte_len = (remaind_len - 1) % 8 + 1;
		int8_t end_bit = end_byte * 8 + end_byte_len - 1;

    if(is_signed){
      if(value < 0){
        value += (1 << signal_len);  // 【【【求得这个负数对应的正数即可！】】】
      }
    }

		data[start_byte] &= ((0xff << ((end_bit % 8) + 1)) + ((1 << (start_bit % 8)) - 1));
		data[start_byte] |= (value << (start_bit % 8));
	}
}

#endif
