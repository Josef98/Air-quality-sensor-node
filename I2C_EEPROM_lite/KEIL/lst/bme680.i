#line 1 "bst_driver\\bme680.c"













































 


 
#line 1 "bst_driver\\bme680.h"














































 

 


 



 





 
#line 1 "bst_driver\\bme680_defs.h"














































 


 



 



 
 
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
 
 





 









     
#line 27 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"
     











#line 46 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"





 

     

     
typedef   signed          char int8_t;
typedef   signed short     int int16_t;
typedef   signed           int int32_t;
typedef   signed       __int64 int64_t;

     
typedef unsigned          char uint8_t;
typedef unsigned short     int uint16_t;
typedef unsigned           int uint32_t;
typedef unsigned       __int64 uint64_t;

     

     
     
typedef   signed          char int_least8_t;
typedef   signed short     int int_least16_t;
typedef   signed           int int_least32_t;
typedef   signed       __int64 int_least64_t;

     
typedef unsigned          char uint_least8_t;
typedef unsigned short     int uint_least16_t;
typedef unsigned           int uint_least32_t;
typedef unsigned       __int64 uint_least64_t;

     

     
typedef   signed           int int_fast8_t;
typedef   signed           int int_fast16_t;
typedef   signed           int int_fast32_t;
typedef   signed       __int64 int_fast64_t;

     
typedef unsigned           int uint_fast8_t;
typedef unsigned           int uint_fast16_t;
typedef unsigned           int uint_fast32_t;
typedef unsigned       __int64 uint_fast64_t;

     




typedef   signed           int intptr_t;
typedef unsigned           int uintptr_t;


     
typedef   signed     long long intmax_t;
typedef unsigned     long long uintmax_t;




     

     





     





     





     

     





     





     





     

     





     





     





     

     






     






     






     

     


     


     


     

     
#line 216 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     



     






     
    
 



#line 241 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"

     







     










     











#line 305 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stdint.h"






 
#line 65 "bst_driver\\bme680_defs.h"
#line 1 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"
 






 

 
 
 





 





#line 34 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"




  typedef signed int ptrdiff_t;



  



    typedef unsigned int size_t;    
#line 57 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



   



      typedef unsigned short wchar_t;  
#line 82 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



    




   




  typedef long double max_align_t;









#line 114 "C:\\Keil_v5\\ARM\\ARMCC\\Bin\\..\\include\\stddef.h"



 

#line 66 "bst_driver\\bme680_defs.h"


 
 
 





















 

 
#line 102 "bst_driver\\bme680_defs.h"

 


 



 


 




 



 


 

 





 



 



 
 






 


 



 
#line 163 "bst_driver\\bme680_defs.h"

 



 


 


 



 



 
#line 189 "bst_driver\\bme680_defs.h"

 
#line 199 "bst_driver\\bme680_defs.h"

 



 


 



 


 



 






 
#line 235 "bst_driver\\bme680_defs.h"

 



 
#line 261 "bst_driver\\bme680_defs.h"

 






 
#line 304 "bst_driver\\bme680_defs.h"

 
#line 313 "bst_driver\\bme680_defs.h"

 


 






 





 









 
typedef int8_t (*bme680_com_fptr_t)(uint8_t dev_id, uint8_t reg_addr, uint8_t *data, uint16_t len);




 
typedef void (*bme680_delay_fptr_t)(uint32_t period);



 
enum bme680_intf {
	 
	BME680_SPI_INTF,
	 
	BME680_I2C_INTF
};

 


 
struct	bme680_field_data {
	 
	uint8_t status;
	 
	uint8_t gas_index;
	 
	uint8_t meas_index;
	 
	int16_t temperature;
	 
	uint32_t pressure;
	 
	uint32_t humidity;
	 
	uint32_t gas_resistance;
};



 
struct	bme680_calib_data {
	 
	uint16_t par_h1;
	 
	uint16_t par_h2;
	 
	int8_t par_h3;
	 
	int8_t par_h4;
	 
	int8_t par_h5;
	 
	uint8_t par_h6;
	 
	int8_t par_h7;
	 
	int8_t par_gh1;
	 
	int16_t par_gh2;
	 
	int8_t par_gh3;
	 
	uint16_t par_t1;
	 
	int16_t par_t2;
	 
	int8_t par_t3;
	 
	uint16_t par_p1;
	 
	int16_t par_p2;
	 
	int8_t par_p3;
	 
	int16_t par_p4;
	 
	int16_t par_p5;
	 
	int8_t par_p6;
	 
	int8_t par_p7;
	 
	int16_t par_p8;
	 
	int16_t par_p9;
	 
	uint8_t par_p10;
	 
	int32_t t_fine;
	 
	uint8_t res_heat_range;
	 
	int8_t res_heat_val;
	 
	int8_t range_sw_err;
};




 
struct	bme680_tph_sett {
	 
	uint8_t os_hum;
	 
	uint8_t os_temp;
	 
	uint8_t os_pres;
	 
	uint8_t filter;
};




 
struct	bme680_gas_sett {
	 
	uint8_t nb_conv;
	 
	uint8_t heatr_ctrl;
	 
	uint8_t run_gas;
	 
	uint16_t heatr_temp;
	 
	uint16_t heatr_dur;
};



 
struct	bme680_dev {
	 
	uint8_t chip_id;
	 
	uint8_t dev_id;
	 
	enum bme680_intf intf;
	 
	uint8_t mem_page;
	 
	int8_t amb_temp;
	 
	struct bme680_calib_data calib;
	 
	struct bme680_tph_sett tph_sett;
	 
	struct bme680_gas_sett gas_sett;
	 
	uint8_t power_mode;
	 
	uint8_t new_fields;
	 
	uint8_t info_msg;
	 
	bme680_com_fptr_t read;
	 
	bme680_com_fptr_t write;
	 
	bme680_delay_fptr_t delay_ms;
	 
	int8_t com_rslt;
};




 
 
#line 64 "bst_driver\\bme680.h"

 








 
int8_t bme680_init(struct bme680_dev *dev);













 
int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev);











 
int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev);








 
int8_t bme680_soft_reset(struct bme680_dev *dev);














 
int8_t bme680_set_sensor_mode(struct bme680_dev *dev);














 
int8_t bme680_get_sensor_mode(struct bme680_dev *dev);








 
void bme680_set_profile_dur(uint16_t duration, struct bme680_dev *dev);








 
void bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev);











 
int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev);



























 
int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev);











 
int8_t bme680_get_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev);




 
#line 51 "bst_driver\\bme680.c"

 
 
uint32_t lookupTable1[16] = { (2147483647u), (2147483647u), (2147483647u), (2147483647u),
	(2147483647u), (2126008810u), (2147483647u), (2130303777u), (2147483647u),
	(2147483647u), (2143188679u), (2136746228u), (2147483647u), (2126008810u),
	(2147483647u), (2147483647u) };
 
uint32_t lookupTable2[16] = { (4096000000u), (2048000000u), (1024000000u), (512000000u),
	(255744255u), (127110228u), (64000000u), (32258064u), (16016016u), (8000000u), (4000000u), (2000000u), (1000000u), (500000u), (250000u),

	(125000u) };















 
static int8_t get_calib_data(struct bme680_dev *dev);








 
static int8_t set_gas_config(struct bme680_dev *dev);








 
static int8_t get_gas_config(struct bme680_dev *dev);







 
static uint8_t calc_heater_dur(uint16_t dur);








 
static int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev);








 
static uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev);








 
static uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev);









 
static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev);








 
static uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev);








 
static int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev);
















 
static int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev);















 
static int8_t get_mem_page(struct bme680_dev *dev);









 
static int8_t null_ptr_check(const struct bme680_dev *dev);












 
static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev);

 



 
int8_t bme680_init(struct bme680_dev *dev)
{
	int8_t rslt;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		 
		rslt = bme680_soft_reset(dev);
		if (rslt == (0)) {
			rslt = bme680_get_regs((0xd0u), &dev->chip_id, 1, dev);
			if (rslt == (0)) {
				if (dev->chip_id == (0x61u)) {
					 
					rslt = get_calib_data(dev);
				} else {
					rslt = (-3);
				}
			}
		}
	}

	return rslt;
}



 
int8_t bme680_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len, struct bme680_dev *dev)
{
	int8_t rslt;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		if (dev->intf == BME680_SPI_INTF) {
			 
			rslt = set_mem_page(reg_addr, dev);
			if (rslt == (0))
				reg_addr = reg_addr | (0x80u);
		}
		dev->com_rslt = dev->read(dev->dev_id, reg_addr, reg_data, len);
		if (dev->com_rslt != 0)
			rslt = (-2);
	}

	return rslt;
}




 
int8_t bme680_set_regs(const uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len, struct bme680_dev *dev)
{
	int8_t rslt;
	 
	uint8_t tmp_buff[(40u)] = { 0 };
	uint16_t index;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		if ((len > 0) && (len < (40u) / 2)) {
			 
			for (index = 0; index < len; index++) {
				if (dev->intf == BME680_SPI_INTF) {
					 
					rslt = set_mem_page(reg_addr[index], dev);
					tmp_buff[(2 * index)] = reg_addr[index] & (0x7fu);
				} else {
					tmp_buff[(2 * index)] = reg_addr[index];
				}
				tmp_buff[(2 * index) + 1] = reg_data[index];
			}
			 
			if (rslt == (0)) {
				dev->com_rslt = dev->write(dev->dev_id, tmp_buff[0], &tmp_buff[1], (2 * len) - 1);
				if (dev->com_rslt != 0)
					rslt = (-2);
			}
		} else {
			rslt = (-4);
		}
	}

	return rslt;
}



 
int8_t bme680_soft_reset(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr = (0xe0u);
	 
	uint8_t soft_rst_cmd = (0xb6u);

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		if (dev->intf == BME680_SPI_INTF)
			rslt = get_mem_page(dev);

		 
		if (rslt == (0)) {
			rslt = bme680_set_regs(&reg_addr, &soft_rst_cmd, 1, dev);
			 
			dev->delay_ms((10u));

			if (rslt == (0)) {
				 
				if (dev->intf == BME680_SPI_INTF)
					rslt = get_mem_page(dev);
			}
		}
	}

	return rslt;
}




 
int8_t bme680_set_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg_addr;
	uint8_t data = 0;
	uint8_t count = 0;
	uint8_t reg_array[(6u)] = { 0 };
	uint8_t data_array[(6u)] = { 0 };
	uint8_t intended_power_mode = dev->power_mode;  

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		if (desired_settings & (8u))
			rslt = set_gas_config(dev);

		dev->power_mode = (0u);
		if (rslt == (0))
			rslt = bme680_set_sensor_mode(dev);

		 
		if (desired_settings & (16u)) {
			rslt = boundary_check(&dev->tph_sett.filter, (0u), (7u), dev);
			reg_addr = (0x75u);

			if (rslt == (0))
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & (16u))
				data = ((data & ~((0X1Cu))) | ((dev->tph_sett . filter << (2u)) & (0X1Cu)));

			reg_array[count] = reg_addr;  
			data_array[count] = data;
			count++;
		}

		 
		if (desired_settings & (32u)) {
			rslt = boundary_check(&dev->gas_sett.heatr_ctrl, (0x00u),
				(0x08u), dev);
			reg_addr = (0x70u);

			if (rslt == (0))
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);
			data = ((data & ~((0x08u))) | (dev->gas_sett . heatr_ctrl & (0x08u)));

			reg_array[count] = reg_addr;  
			data_array[count] = data;
			count++;
		}

		 
		if (desired_settings & ((1u) | (2u))) {
			rslt = boundary_check(&dev->tph_sett.os_temp, (0u), (5u), dev);
			reg_addr = (0x74u);

			if (rslt == (0))
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & (1u))
				data = ((data & ~((0XE0u))) | ((dev->tph_sett . os_temp << (5u)) & (0XE0u)));

			if (desired_settings & (2u))
				data = ((data & ~((0X1Cu))) | ((dev->tph_sett . os_pres << (2u)) & (0X1Cu)));

			reg_array[count] = reg_addr;
			data_array[count] = data;
			count++;
		}

		 
		if (desired_settings & (4u)) {
			rslt = boundary_check(&dev->tph_sett.os_hum, (0u), (5u), dev);
			reg_addr = (0x72u);

			if (rslt == (0))
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);
			data = ((data & ~((0X07u))) | (dev->tph_sett . os_hum & (0X07u)));

			reg_array[count] = reg_addr;  
			data_array[count] = data;
			count++;
		}

		 
		if (desired_settings & ((64u) | (128u))) {
			rslt = boundary_check(&dev->gas_sett.run_gas, (0u),
				(1u), dev);
			if (rslt == (0)) {
				 
				rslt = boundary_check(&dev->gas_sett.nb_conv, (0u),
					(10u), dev);
			}

			reg_addr = (0x71u);

			if (rslt == (0))
				rslt = bme680_get_regs(reg_addr, &data, 1, dev);

			if (desired_settings & (64u))
				data = ((data & ~((0x10u))) | ((dev->gas_sett . run_gas << (4u)) & (0x10u)));

			if (desired_settings & (128u))
				data = ((data & ~((0X0Fu))) | (dev->gas_sett . nb_conv & (0X0Fu)));

			reg_array[count] = reg_addr;  
			data_array[count] = data;
			count++;
		}

		if (rslt == (0))
			rslt = bme680_set_regs(reg_array, data_array, count, dev);

		 
		dev->power_mode = intended_power_mode;
	}

	return rslt;
}




 
int8_t bme680_get_sensor_settings(uint16_t desired_settings, struct bme680_dev *dev)
{
	int8_t rslt;
	 
	uint8_t reg_addr = (0x70u);
	uint8_t data_array[(6u)] = { 0 };

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		rslt = bme680_get_regs(reg_addr, data_array, (6u), dev);

		if (rslt == (0)) {
			if (desired_settings & (8u))
				rslt = get_gas_config(dev);

			 
			if (desired_settings & (16u))
				dev->tph_sett.filter = ((data_array[(5u)] & ((0X1Cu))) >> ((2u)));


			if (desired_settings & ((1u) | (2u))) {
				dev->tph_sett.os_temp = ((data_array[(4u)] & ((0XE0u))) >> ((5u)));
				dev->tph_sett.os_pres = ((data_array[(4u)] & ((0X1Cu))) >> ((2u)));
			}

			if (desired_settings & (4u))
				dev->tph_sett.os_hum = (data_array[(2u)] & ((0X07u)));


			 
			if (desired_settings & (32u))
				dev->gas_sett.heatr_ctrl = (data_array[(0u)] & ((0x08u)));


			if (desired_settings & ((64u) | (128u))) {
				dev->gas_sett.nb_conv = (data_array[(1u)] & ((0X0Fu)));

				dev->gas_sett.run_gas = ((data_array[(1u)] & ((0x10u))) >> ((4u)));

			}
		}
	} else {
		rslt = (-1);
	}

	return rslt;
}



 
int8_t bme680_set_sensor_mode(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t tmp_pow_mode;
	uint8_t pow_mode = 0;
	uint8_t reg_addr = (0x74u);

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		 
		do {
			rslt = bme680_get_regs((0x74u), &tmp_pow_mode, 1, dev);
			if (rslt == (0)) {
				 
				pow_mode = (tmp_pow_mode & (0x03u));

				if (pow_mode != (0u)) {
					tmp_pow_mode = tmp_pow_mode & (~(0x03u));  
					rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
					dev->delay_ms((10u));
				}
			}
		} while (pow_mode != (0u));

		 
		if (dev->power_mode != (0u)) {
			tmp_pow_mode = (tmp_pow_mode & ~(0x03u)) | (dev->power_mode & (0x03u));
			if (rslt == (0))
				rslt = bme680_set_regs(&reg_addr, &tmp_pow_mode, 1, dev);
		}
	}

	return rslt;
}



 
int8_t bme680_get_sensor_mode(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t mode;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		rslt = bme680_get_regs((0x74u), &mode, 1, dev);
		 
		dev->power_mode = mode & (0x03u);
	}

	return rslt;
}



 
void bme680_set_profile_dur(uint16_t duration, struct bme680_dev *dev)
{
	uint32_t tph_dur;  

	 
	tph_dur = ((uint32_t) (dev->tph_sett.os_temp + dev->tph_sett.os_pres + dev->tph_sett.os_hum) * (1963u));
	tph_dur += (477 * 4u);  
	tph_dur += (477 * 5u);  
	tph_dur += (500u);  
	tph_dur /= (1000u);  

	tph_dur += (1u);  
	 
	dev->gas_sett.heatr_dur = duration - (uint16_t) tph_dur;
}



 
void bme680_get_profile_dur(uint16_t *duration, const struct bme680_dev *dev)
{
	uint32_t tph_dur;  

	 
	tph_dur = ((uint32_t) (dev->tph_sett.os_temp + dev->tph_sett.os_pres + dev->tph_sett.os_hum) * (1963u));
	tph_dur += (477 * 4u);  
	tph_dur += (477 * 5u);  
	tph_dur += (500u);  
	tph_dur /= (1000u);  

	tph_dur += (1u);  

	*duration = (uint16_t) tph_dur;

	 
	if (dev->gas_sett.run_gas) {
		 
		*duration += dev->gas_sett.heatr_dur;
	}
}





 
int8_t bme680_get_sensor_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
	int8_t rslt;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		 
		rslt = read_field_data(data, dev);
		if (rslt == (0)) {
			if (data->status & (0x80u))
				dev->new_fields = 1;
			else
				dev->new_fields = 0;
		}
	}

	return rslt;
}



 
static int8_t get_calib_data(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t coeff_array[(0x41u)] = { 0 };
	uint8_t temp_var = 0;  

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		rslt = bme680_get_regs((0x89u), coeff_array, (25u), dev);
		 
		if (rslt == (0))
			rslt = bme680_get_regs((0xe1u), &coeff_array[(25u)]
			, (16u), dev);

		 
		dev->calib.par_t1 = (uint16_t) ((((uint16_t)coeff_array[(34)] << 8) | (uint16_t)coeff_array[(33)]));

		dev->calib.par_t2 = (int16_t) ((((uint16_t)coeff_array[(2)] << 8) | (uint16_t)coeff_array[(1)]));

		dev->calib.par_t3 = (int8_t) (coeff_array[(3)]);

		 
		dev->calib.par_p1 = (uint16_t) ((((uint16_t)coeff_array[(6)] << 8) | (uint16_t)coeff_array[(5)]));

		dev->calib.par_p2 = (int16_t) ((((uint16_t)coeff_array[(8)] << 8) | (uint16_t)coeff_array[(7)]));

		dev->calib.par_p3 = (int8_t) coeff_array[(9)];
		dev->calib.par_p4 = (int16_t) ((((uint16_t)coeff_array[(12)] << 8) | (uint16_t)coeff_array[(11)]));

		dev->calib.par_p5 = (int16_t) ((((uint16_t)coeff_array[(14)] << 8) | (uint16_t)coeff_array[(13)]));

		dev->calib.par_p6 = (int8_t) (coeff_array[(16)]);
		dev->calib.par_p7 = (int8_t) (coeff_array[(15)]);
		dev->calib.par_p8 = (int16_t) ((((uint16_t)coeff_array[(20)] << 8) | (uint16_t)coeff_array[(19)]));

		dev->calib.par_p9 = (int16_t) ((((uint16_t)coeff_array[(22)] << 8) | (uint16_t)coeff_array[(21)]));

		dev->calib.par_p10 = (uint8_t) (coeff_array[(23)]);

		 
		dev->calib.par_h1 = (uint16_t) (((uint16_t) coeff_array[(27)] << (4u))
			| (coeff_array[(26)] & (0x0Fu)));
		dev->calib.par_h2 = (uint16_t) (((uint16_t) coeff_array[(25)] << (4u))
			| ((coeff_array[(26)]) >> (4u)));
		dev->calib.par_h3 = (int8_t) coeff_array[(28)];
		dev->calib.par_h4 = (int8_t) coeff_array[(29)];
		dev->calib.par_h5 = (int8_t) coeff_array[(30)];
		dev->calib.par_h6 = (uint8_t) coeff_array[(31)];
		dev->calib.par_h7 = (int8_t) coeff_array[(32)];

		 
		dev->calib.par_gh1 = (int8_t) coeff_array[(37)];
		dev->calib.par_gh2 = (int16_t) ((((uint16_t)coeff_array[(36)] << 8) | (uint16_t)coeff_array[(35)]));

		dev->calib.par_gh3 = (int8_t) coeff_array[(38)];

		 
		if (rslt == (0)) {
			rslt = bme680_get_regs((0x02u), &temp_var, 1, dev);

			dev->calib.res_heat_range = ((temp_var & (0x30u)) / 16);
			if (rslt == (0)) {
				rslt = bme680_get_regs((0x00u), &temp_var, 1, dev);

				dev->calib.res_heat_val = (int8_t) temp_var;
				if (rslt == (0))
					rslt = bme680_get_regs((0x04u), &temp_var, 1, dev);
			}
		}
		dev->calib.range_sw_err = ((int8_t) temp_var & (int8_t) (0xf0u)) / 16;
	}

	return rslt;
}



 
static int8_t set_gas_config(struct bme680_dev *dev)
{
	int8_t rslt;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {

		uint8_t reg_addr[2] = {0};
		uint8_t reg_data[2] = {0};

		if (dev->power_mode == (1u)) {
			reg_addr[0] = (0x5au);
			reg_data[0] = calc_heater_res(dev->gas_sett.heatr_temp, dev);
			reg_addr[1] = (0x64u);
			reg_data[1] = calc_heater_dur(dev->gas_sett.heatr_dur);
			dev->gas_sett.nb_conv = 0;
		} else {
			rslt = (1);
		}
		if (rslt == (0))
			rslt = bme680_set_regs(reg_addr, reg_data, 2, dev);
	}

	return rslt;
}



 
static int8_t get_gas_config(struct bme680_dev *dev)
{
	int8_t rslt;
	 
	uint8_t reg_addr1 = (0x5Au);
	uint8_t reg_addr2 = (0x64u);
	uint8_t data_array[(10u)] = { 0 };
	uint8_t index;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		if (BME680_SPI_INTF == dev->intf) {
			 
			rslt = set_mem_page(reg_addr1, dev);
		}

		if (rslt == (0)) {
			rslt = bme680_get_regs(reg_addr1, data_array, (10u), dev);
			if (rslt == (0)) {
				for (index = 0; index < (10u); index++)
					dev->gas_sett.heatr_temp = data_array[index];
			}

			rslt = bme680_get_regs(reg_addr2, data_array, (10u), dev);
			if (rslt == (0)) {
				for (index = 0; index < (10u); index++)
					dev->gas_sett.heatr_dur = data_array[index];
			}
		}
	}

	return rslt;
}



 
static int16_t calc_temperature(uint32_t temp_adc, struct bme680_dev *dev)
{
	int64_t var1;
	int64_t var2;
	int64_t var3;
	int16_t calc_temp;

	var1 = ((int32_t) temp_adc >> 3) - ((int32_t) dev->calib.par_t1 << 1);
	var2 = (var1 * (int32_t) dev->calib.par_t2) >> 11;
	var3 = ((var1 >> 1) * (var1 >> 1)) >> 12;
	var3 = ((var3) * ((int32_t) dev->calib.par_t3 << 4)) >> 14;
	dev->calib.t_fine = (int32_t) (var2 + var3);
	calc_temp = (int16_t) (((dev->calib.t_fine * 5) + 128) >> 8);

	return calc_temp;
}



 
static uint32_t calc_pressure(uint32_t pres_adc, const struct bme680_dev *dev)
{
	int32_t var1 = 0;
	int32_t var2 = 0;
	int32_t var3 = 0;
	int32_t var4 = 0;
	int32_t pressure_comp = 0;

	var1 = (((int32_t)dev->calib.t_fine) >> 1) - 64000;
	var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
		(int32_t)dev->calib.par_p6) >> 2;
	var2 = var2 + ((var1 * (int32_t)dev->calib.par_p5) << 1);
	var2 = (var2 >> 2) + ((int32_t)dev->calib.par_p4 << 16);
	var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
		((int32_t)dev->calib.par_p3 << 5)) >> 3) +
		(((int32_t)dev->calib.par_p2 * var1) >> 1);
	var1 = var1 >> 18;
	var1 = ((32768 + var1) * (int32_t)dev->calib.par_p1) >> 15;
	pressure_comp = 1048576 - pres_adc;
	pressure_comp = (int32_t)((pressure_comp - (var2 >> 12)) * ((uint32_t)3125));
	var4 = (1 << 31);
	if (pressure_comp >= var4)
		pressure_comp = ((pressure_comp / (uint32_t)var1) << 1);
	else
		pressure_comp = ((pressure_comp << 1) / (uint32_t)var1);
	var1 = ((int32_t)dev->calib.par_p9 * (int32_t)(((pressure_comp >> 3) *
		(pressure_comp >> 3)) >> 13)) >> 12;
	var2 = ((int32_t)(pressure_comp >> 2) *
		(int32_t)dev->calib.par_p8) >> 13;
	var3 = ((int32_t)(pressure_comp >> 8) * (int32_t)(pressure_comp >> 8) *
		(int32_t)(pressure_comp >> 8) *
		(int32_t)dev->calib.par_p10) >> 17;

	pressure_comp = (int32_t)(pressure_comp) + ((var1 + var2 + var3 +
		((int32_t)dev->calib.par_p7 << 7)) >> 4);

	return (uint32_t)pressure_comp;

}



 
static uint32_t calc_humidity(uint16_t hum_adc, const struct bme680_dev *dev)
{
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t var6;
	int32_t temp_scaled;
	int32_t calc_hum;

	temp_scaled = (((int32_t) dev->calib.t_fine * 5) + 128) >> 8;
	var1 = (int32_t) (hum_adc - ((int32_t) ((int32_t) dev->calib.par_h1 * 16)))
		- (((temp_scaled * (int32_t) dev->calib.par_h3) / ((int32_t) 100)) >> 1);
	var2 = ((int32_t) dev->calib.par_h2
		* (((temp_scaled * (int32_t) dev->calib.par_h4) / ((int32_t) 100))
			+ (((temp_scaled * ((temp_scaled * (int32_t) dev->calib.par_h5) / ((int32_t) 100))) >> 6)
				/ ((int32_t) 100)) + (int32_t) (1 << 14))) >> 10;
	var3 = var1 * var2;
	var4 = (int32_t) dev->calib.par_h6 << 7;
	var4 = ((var4) + ((temp_scaled * (int32_t) dev->calib.par_h7) / ((int32_t) 100))) >> 4;
	var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
	var6 = (var4 * var5) >> 1;
	calc_hum = (((var3 + var6) >> 10) * ((int32_t) 1000)) >> 12;

	if (calc_hum > 100000)  
		calc_hum = 100000;
	else if (calc_hum < 0)
		calc_hum = 0;

	return (uint32_t) calc_hum;
}



 
static uint32_t calc_gas_resistance(uint16_t gas_res_adc, uint8_t gas_range, const struct bme680_dev *dev)
{
	int64_t var1;
	uint64_t var2;
	int64_t var3;
	uint32_t calc_gas_res;

	var1 = (int64_t) ((1340 + (5 * (int64_t) dev->calib.range_sw_err)) *
		((int64_t) lookupTable1[gas_range])) >> 16;
	var2 = (((int64_t) ((int64_t) gas_res_adc << 15) - (int64_t) (16777216)) + var1);
	var3 = (((int64_t) lookupTable2[gas_range] * (int64_t) var1) >> 9);
	calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);

	return calc_gas_res;
}



 
static uint8_t calc_heater_res(uint16_t temp, const struct bme680_dev *dev)
{
	uint8_t heatr_res;
	int32_t var1;
	int32_t var2;
	int32_t var3;
	int32_t var4;
	int32_t var5;
	int32_t heatr_res_x100;

	if (temp < 200)  
		temp = 200;
	else if (temp > 400)
		temp = 400;

	var1 = (((int32_t) dev->amb_temp * dev->calib.par_gh3) / 1000) * 256;
	var2 = (dev->calib.par_gh1 + 784) * (((((dev->calib.par_gh2 + 154009) * temp * 5) / 100) + 3276800) / 10);
	var3 = var1 + (var2 / 2);
	var4 = (var3 / (dev->calib.res_heat_range + 4));
	var5 = (131 * dev->calib.res_heat_val) + 65536;
	heatr_res_x100 = (int32_t) (((var4 / var5) - 250) * 34);
	heatr_res = (uint8_t) ((heatr_res_x100 + 50) / 100);

	return heatr_res;
}



 
static uint8_t calc_heater_dur(uint16_t dur)
{
	uint8_t factor = 0;
	uint8_t durval;

	if (dur >= 0xfc0) {
		durval = 0xff;  
	} else {
		while (dur > 0x3F) {
			dur = dur / 4;
			factor += 1;
		}
		durval = (uint8_t) (dur + (factor * 64));
	}

	return durval;
}



 
static int8_t read_field_data(struct bme680_field_data *data, struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t buff[(15u)] = { 0 };
	uint8_t gas_range;
	uint32_t adc_temp;
	uint32_t adc_pres;
	uint16_t adc_hum;
	uint16_t adc_gas_res;
	uint8_t tries = 10;

	 
	rslt = null_ptr_check(dev);
	do {
		if (rslt == (0)) {
			rslt = bme680_get_regs(((uint8_t) ((0x1du))), buff, (uint16_t) (15u),
				dev);

			data->status = buff[0] & (0x80u);
			data->gas_index = buff[0] & (0x0fu);
			data->meas_index = buff[1];

			 
			adc_pres = (uint32_t) (((uint32_t) buff[2] * 4096) | ((uint32_t) buff[3] * 16)
				| ((uint32_t) buff[4] / 16));
			adc_temp = (uint32_t) (((uint32_t) buff[5] * 4096) | ((uint32_t) buff[6] * 16)
				| ((uint32_t) buff[7] / 16));
			adc_hum = (uint16_t) (((uint32_t) buff[8] * 256) | (uint32_t) buff[9]);
			adc_gas_res = (uint16_t) ((uint32_t) buff[13] * 4 | (((uint32_t) buff[14]) / 64));
			gas_range = buff[14] & (0x0fu);

			data->status |= buff[14] & (0x20u);
			data->status |= buff[14] & (0x10u);

			if (data->status & (0x80u)) {
				data->temperature = calc_temperature(adc_temp, dev);
				data->pressure = calc_pressure(adc_pres, dev);
				data->humidity = calc_humidity(adc_hum, dev);
				data->gas_resistance = calc_gas_resistance(adc_gas_res, gas_range, dev);
				break;
			}
			 
			dev->delay_ms((10u));
		}
		tries--;
	} while (tries);

	if (!tries)
		rslt = (2);

	return rslt;
}



 
static int8_t set_mem_page(uint8_t reg_addr, struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg;
	uint8_t mem_page;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		if (reg_addr > 0x7f)
			mem_page = (0x00u);
		else
			mem_page = (0x10u);

		if (mem_page != dev->mem_page) {
			dev->mem_page = mem_page;

			dev->com_rslt = dev->read(dev->dev_id, (0xf3u) | (0x80u), &reg, 1);
			if (dev->com_rslt != 0)
				rslt = (-2);

			if (rslt == (0)) {
				reg = reg & (~(0x10u));
				reg = reg | (dev->mem_page & (0x10u));

				dev->com_rslt = dev->write(dev->dev_id, (0xf3u) & (0x7fu),
					&reg, 1);
				if (dev->com_rslt != 0)
					rslt = (-2);
			}
		}
	}

	return rslt;
}



 
static int8_t get_mem_page(struct bme680_dev *dev)
{
	int8_t rslt;
	uint8_t reg;

	 
	rslt = null_ptr_check(dev);
	if (rslt == (0)) {
		dev->com_rslt = dev->read(dev->dev_id, (0xf3u) | (0x80u), &reg, 1);
		if (dev->com_rslt != 0)
			rslt = (-2);
		else
			dev->mem_page = reg & (0x10u);
	}

	return rslt;
}




 
static int8_t boundary_check(uint8_t *value, uint8_t min, uint8_t max, struct bme680_dev *dev)
{
	int8_t rslt = (0);

	if (value != 0) {
		 
		if (*value < min) {
			 
			*value = min;
			dev->info_msg |= (1u);
		}
		 
		if (*value > max) {
			 
			*value = max;
			dev->info_msg |= (2u);
		}
	} else {
		rslt = (-1);
	}

	return rslt;
}




 
static int8_t null_ptr_check(const struct bme680_dev *dev)
{
	int8_t rslt;

	if ((dev == 0) || (dev->read == 0) || (dev->write == 0) || (dev->delay_ms == 0)) {
		 
		rslt = (-1);
	} else {
		 
		rslt = (0);
	}

	return rslt;
}
