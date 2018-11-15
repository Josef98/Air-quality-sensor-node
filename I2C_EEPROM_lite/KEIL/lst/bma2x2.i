#line 1 "bst_driver\\bma2x2.c"


















































 

 
#line 1 "bst_driver\\bma2x2.h"





















































 


 


 
 
 






 




 
#line 92 "bst_driver\\bma2x2.h"



 
#line 222 "bst_driver\\bma2x2.h"




 



 
#line 259 "bst_driver\\bma2x2.h"
 
typedef	signed char  s8; 
typedef	signed short int s16; 
typedef	signed int s32; 
typedef	signed long long int s64; 

 
typedef	unsigned char u8; 
typedef	unsigned short int u16; 
typedef	unsigned int u32; 
typedef	unsigned long long int u64; 


 
#line 292 "bst_driver\\bma2x2.h"

 
 
 

















 























 





















 


 


























 








 
 
 










 







 



 
 
 



	 





	 




	 
#line 450 "bst_driver\\bma2x2.h"

 
 
 





 
 
 

 

 
 
 




 
#line 480 "bst_driver\\bma2x2.h"
 





 
#line 493 "bst_driver\\bma2x2.h"
 
#line 503 "bst_driver\\bma2x2.h"
 
#line 521 "bst_driver\\bma2x2.h"
 





 


 




 
 
 




 
 
 
 



 
 
 


 
struct bma2x2_accel_data {
s16 x, 
y, 
z; 
};



 
struct bma2x2_accel_data_temp {
s16 x, 
y, 
z; 
s8 temp; 
};



 
struct  bma2x2_accel_eight_resolution {
s8 x, 
y, 
z; 
};


 
struct bma2x2_accel_eight_resolution_temp {
s8 x, 
y, 
z; 
s8 temp; 
};











 
struct bma2x2_t {
u8 power_mode_u8; 
u8 chip_id; 
u8 ctrl_mode_reg; 
u8 low_mode_reg; 
u8 dev_addr; 
u8 fifo_config; 
s8(*bus_write)(u8, u8, u8 *, u8); 
s8(*bus_read)(u8, u8, u8 *, u8); 
s8(*burst_read)(u8, u8, u8 *, u32); 
void (*delay_msec)(u32);

 
};

 
 
 
 
 
 





 
 
 





























 
 
 





























 
 
 






























 
 
 





 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 



















 
 
 



















 
 
 



















 
 
 




 
 
 





 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 

 
 
 














 
 
 




 
 
 




 
 
 




 
 
 




 
 
 














 
 
 




 
 
 




 
 
 




 
 
 




 
 
 

















 
 
 





 
 
 




 
 
 




 
 
 




 
 
 





 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 





 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 





 
 
 




 
 
 




 
 
 



















 
 
 




 
 
 




 
 
 




 
 
 





 
 
 




 
 
 




 
 
 





 
 
 





 
 
 





 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 





 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 




 
 
 









 
 
 




 
 
 




 
 
 




 
 
 




 
 
 














 
 
 




 
 
 




 
 
 




 
 
 




 
 
 














 
 
 




 
 
 





 
 
 







 
 
 
 
 
 
 

 

 

 

 

 
 
 

 

 

 
 
 

 

 

 
 
 

 

 

 
 
 

 

 

 

 

 

 
 
 

 

 

 

 



  

 

 

  

  

 

 

  

 
 
 

 

 

 

 

 

 

 

 

 

 

 

 
 
 

 

 

 

  

  

  

  

  

  

  

  

  

 

 

  

 

 
 
 
#line 1776 "bst_driver\\bma2x2.h"

 
 
 

 

 

 

 
 
 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 
 
 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 

 
 
 
#line 1876 "bst_driver\\bma2x2.h"






 
 
 





 
 
 














 
 
 



 
 
 




 
 
 


 
 
 




 
 
 






 
 






 
 
 




 
 
 




 
 
 


 
 
 
#line 1979 "bst_driver\\bma2x2.h"
 
 
 


 
 
 


 
 
 



 
 
 




#line 2009 "bst_driver\\bma2x2.h"





#line 2021 "bst_driver\\bma2x2.h"






 







 







 







 








 


 
 
 
 
 
 
















 
s8 bma2x2_burst_read(u8 addr_u8,
u8 *data_u8, u32 len_u32);
 
 
 






















 
s8 bma2x2_init(struct bma2x2_t *bma2x2);

















 
s8 bma2x2_write_reg(u8 adr_u8,
u8 *data_u8, u8 len_u8);
















 
s8 bma2x2_read_reg(u8 adr_u8,
u8 *data_u8, u8 len_u8);
 
 
 



















 
s8 bma2x2_read_accel_x(s16 *accel_x_s16);















 
s8 bma2x2_read_accel_eight_resolution_x(
s8 *accel_x_s8);


















 
s8 bma2x2_read_accel_y(s16 *accel_y_s16);
















 
s8 bma2x2_read_accel_eight_resolution_y(
s8 *accel_y_s8);


















 
s8 bma2x2_read_accel_z(s16 *accel_z_s16);
















 
s8 bma2x2_read_accel_eight_resolution_z(
s8 *accel_z_s8);
















 
s8 bma2x2_read_accel_xyz(
struct bma2x2_accel_data *accel);

















 
s8 bma2x2_read_accel_eight_resolution_xyz(
struct bma2x2_accel_eight_resolution *accel);
 
 
 














 
s8 bma2x2_get_intr_tap_stat(
u8 *stat_tap_u8);














 
s8 bma2x2_get_intr_orient_stat(
u8 *stat_orient_u8);













 
s8 bma2x2_get_fifo_stat(
u8 *stat_fifo_u8);














 
s8 bma2x2_get_fifo_frame_count(
u8 *frame_count_u8);














 
s8 bma2x2_get_fifo_overrun(
u8 *fifo_overrun_u8);
 
 
 















 
s8 bma2x2_get_intr_stat(
u8 *intr_stat_u8);




















 
s8 bma2x2_get_range(u8 *range_u8);




















 
s8 bma2x2_set_range(u8 range_u8);
 
 
 
























 
s8 bma2x2_get_bw(u8 *bw_u8);
























 
s8 bma2x2_set_bw(u8 bw_u8);
 
 
 
























 
s8 bma2x2_get_power_mode(
u8 *power_mode_u8);
























 
s8 bma2x2_set_power_mode(u8 power_mode_u8);
























 
s8 bma2x2_set_mode_value(u8 power_mode_u8);
 
 
 






























 
s8 bma2x2_get_sleep_durn(u8 *sleep_durn_u8);






























 
s8 bma2x2_set_sleep_durn(u8 sleep_durn_u8);



















 
s8 bma2x2_get_sleep_timer_mode(
u8 *sleep_timer_u8);



















 
s8 bma2x2_set_sleep_timer_mode(u8 sleep_timer_u8);
 
 
 
















 
s8 bma2x2_get_high_bw(u8 *high_bw_u8);
















 
s8 bma2x2_set_high_bw(u8 high_bw_u8);

















 
s8 bma2x2_get_shadow_dis(u8 *shadow_dis_u8);

















 
s8 bma2x2_set_shadow_dis(u8 shadow_dis_u8);
 
 
 












 
s8 bma2x2_soft_rst(void);















 
s8 bma2x2_update_image(void);
 
 
 











































 
s8 bma2x2_get_intr_enable(u8 intr_type_u8,
u8 *value_u8);











































 
s8 bma2x2_set_intr_enable(u8 intr_type_u8,
u8 value_u8);



















 
s8 bma2x2_get_intr_fifo_full(u8 *fifo_full_u8);



















 
s8 bma2x2_set_intr_fifo_full(u8 fifo_full_u8);





















 
s8 bma2x2_get_intr_fifo_wm(u8 *fifo_wm_u8);





















 
s8 bma2x2_set_intr_fifo_wm(u8 fifo_wm_u8);



























 
s8 bma2x2_get_slow_no_motion(u8 channel_u8,
u8 *slow_no_motion_u8);


























 
s8 bma2x2_set_slow_no_motion(u8 channel_u8,
u8 slow_no_motion_u8);




























 
s8 bma2x2_get_intr_low_g(u8 channel_u8,
u8 *intr_low_g_u8);




























 
s8 bma2x2_set_intr_low_g(u8 channel_u8,
u8 intr_low_u8);

























 
s8 bma2x2_get_intr_high_g(u8 channel_u8,
u8 *intr_high_g_u8);

























 
s8 bma2x2_set_intr_high_g(u8 channel_u8,
u8 intr_high_g_u8);



























 
s8 bma2x2_get_intr_slope(u8 channel_u8,
u8 *intr_slope_u8);



























 
s8 bma2x2_set_intr_slope(u8 channel_u8,
u8 intr_slope_u8);




























 
s8 bma2x2_get_intr_slow_no_motion(u8 channel_u8,
u8 *intr_slow_no_motion_u8);




























 
s8 bma2x2_set_intr_slow_no_motion(u8 channel_u8,
u8 intr_slow_no_motion_u8);



























 
s8 bma2x2_get_intr_double_tap(u8 channel_u8,
u8 *intr_double_tap_u8);



























 
s8 bma2x2_set_intr_double_tap(u8 channel_u8,
u8 intr_double_tap_u8);


























 
s8 bma2x2_get_intr_single_tap(u8 channel_u8,
u8 *intr_single_tap_u8);


























 
s8 bma2x2_set_intr_single_tap(u8 channel_u8,
u8 intr_single_tap_u8);

























 
s8 bma2x2_get_intr_orient(u8 channel_u8,
u8 *intr_orient_u8);

























 
s8 bma2x2_set_intr_orient(u8 channel_u8,
u8 intr_orient_u8);



























 
s8 bma2x2_get_intr_flat(u8 channel_u8,
u8 *intr_flat_u8);



























 
s8 bma2x2_set_intr_flat(u8 channel_u8,
u8 intr_flat_u8);


























 
s8 bma2x2_get_new_data(u8 channel_u8,
u8 *intr_newdata_u8);


























 
s8 bma2x2_set_new_data(u8 channel_u8,
u8 intr_newdata_u8);
 
 
 

















 
s8 bma2x2_get_intr1_fifo_wm(u8 *intr1_fifo_wm_u8);

















 
s8 bma2x2_set_intr1_fifo_wm(u8 intr1_fifo_wm_u8);

















 
s8 bma2x2_get_intr2_fifo_wm(u8 *intr2_fifo_wm_u8);

















 
s8 bma2x2_set_intr2_fifo_wm(u8 intr2_fifo_wm_u8);



















 
s8 bma2x2_get_intr1_fifo_full(
u8 *intr1_fifo_full_u8);



















 
s8 bma2x2_set_intr1_fifo_full(u8 intr1_fifo_full_u8);




















 
s8 bma2x2_get_intr2_fifo_full(
u8 *intr2_fifo_full_u8);




















 
s8 bma2x2_set_intr2_fifo_full(u8 intr2_fifo_full_u8);
 
 
 




























 
s8 bma2x2_get_source(u8 channel_u8,
u8 *intr_source_u8);




























 
s8 bma2x2_set_source(u8 channel_u8,
u8 intr_source_u8);
 
 
 

























 
s8 bma2x2_get_intr_output_type(u8 channel_u8,
u8 *intr_output_type_u8);

























 
s8 bma2x2_set_intr_output_type(u8 channel_u8,
u8 intr_output_type_u8);
























 
s8 bma2x2_get_intr_level(u8 channel_u8,
u8 *intr_level_u8);
























 
s8 bma2x2_set_intr_level(u8 channel_u8,
u8 intr_level_u8);
 
 
 


















 
s8 bma2x2_rst_intr(u8 rst_intr_u8);
 
 
 































 
s8 bma2x2_get_latch_intr(u8 *latch_intr_u8);































 
s8 bma2x2_set_latch_intr(u8 latch_intr_u8);
 
 
 






































 
s8 bma2x2_get_durn(u8 channel_u8,
u8 *durn_u8);






































 
s8 bma2x2_set_durn(u8 channel_u8,
u8 durn_u8);
 
 
 



























































 
s8 bma2x2_get_thres(u8 channel_u8,
u8 *thres_u8);



























































 
s8 bma2x2_set_thres(u8 channel_u8,
u8 thres_u8);
 
 
 































 
s8 bma2x2_get_low_high_g_hyst(u8 channel_u8,
u8 *hyst_u8);































 
s8 bma2x2_set_low_high_g_hyst(u8 channel_u8,
u8 hyst_u8);
 
 
 

















 
s8 bma2x2_get_low_g_mode(u8 *low_g_mode_u8);

















 
s8 bma2x2_set_low_g_mode(u8 low_g_mode_u8);
 
 
 
























 
s8 bma2x2_get_tap_durn(u8 *tap_durn_u8);
























 
s8 bma2x2_set_tap_durn(u8 tap_durn_u8);
 
 
 


















 
s8 bma2x2_get_tap_shock(u8 *tap_shock_u8);


















 
s8 bma2x2_set_tap_shock(u8 tap_shock_u8);
 
 
 

















 
s8 bma2x2_get_tap_quiet(u8 *tap_quiet_u8);

















 
s8 bma2x2_set_tap_quiet(u8 tap_quiet_u8);
 
 
 






















 
s8 bma2x2_get_tap_thres(u8 *tap_thres_u8);






















 
s8 bma2x2_set_tap_thres(u8 tap_thres_u8);
 
 
 




















 
s8 bma2x2_get_tap_sample(u8 *tap_sample_u8);




















 
s8 bma2x2_set_tap_sample(u8 tap_sample_u8);
 
 
 





















 
s8 bma2x2_get_orient_mode(u8 *orient_mode_u8);





















 
s8 bma2x2_set_orient_mode(u8 orient_mode_u8);
 
 
 


























 
s8 bma2x2_get_orient_block(
u8 *orient_block_u8);


























 
s8 bma2x2_set_orient_block(u8 orient_block_u8);
 
 
 















 
s8 bma2x2_get_orient_hyst(u8 *orient_hyst_u8);















 
s8 bma2x2_set_orient_hyst(u8 orient_hyst_u8);
 
 
 























 
s8 bma2x2_get_theta(u8 channel_u8,
u8 *theta_u8);























 
s8 bma2x2_set_theta(u8 channel_u8,
u8 theta_u8);
 
 
 


















 
s8 bma2x2_get_orient_enable(
u8 *orient_enable_u8);


















 
s8 bma2x2_set_orient_enable(u8 orient_enable_u8);
 
 
 

















 
s8 bma2x2_get_flat_hyst(u8 *flat_hyst_u8);

















 
s8 bma2x2_set_flat_hyst(u8 flat_hyst_u8);
 
 
 





















 
s8 bma2x2_get_flat_hold_time(
u8 *flat_hold_time_u8);





















 
s8 bma2x2_set_flat_hold_time(
u8 flat_hold_time_u8);
 
 
 
















 
s8 bma2x2_get_fifo_wml_trig(
u8 *fifo_wml_trig);
















 
s8 bma2x2_set_fifo_wml_trig(
u8 fifo_wml_trig);
 
 
 




















 
s8 bma2x2_get_selftest_axis(
u8 *selftest_axis_u8);




















 
s8 bma2x2_set_selftest_axis(
u8 selftest_axis_u8);


















 
s8 bma2x2_get_selftest_sign(
u8 *selftest_sign_u8);


















 
s8 bma2x2_set_selftest_sign(
u8 selftest_sign_u8);
 
 
 
















 
s8 bma2x2_get_nvmprog_mode(
u8 *nvmprog_mode_u8);
















 
s8 bma2x2_set_nvmprog_mode(u8 nvmprog_mode_u8);



















 
s8 bma2x2_set_nvprog_trig(u8 nvprog_trig_u8);


















 
s8 bma2x2_get_nvmprog_ready(u8 *nvprog_ready_u8);


















 
s8 bma2x2_get_nvmprog_remain(u8 *nvprog_remain_u8);
 
 
 



















 
s8 bma2x2_get_spi3(u8 *spi3_u8);



















 
s8 bma2x2_set_spi3(u8 spi3_u8);






























 
s8 bma2x2_get_i2c_wdt(u8 channel_u8,
u8 *i2c_wdt_u8);






























 
s8 bma2x2_set_i2c_wdt(u8 channel_u8,
u8 i2c_wdt_u8);
 
 
 




























 
s8 bma2x2_get_slow_comp(u8 channel_u8,
u8 *slow_comp_u8);




























 
s8 bma2x2_set_slow_comp(u8 channel_u8,
u8 slow_comp_u8);

















 
s8 bma2x2_get_cal_rdy(u8 *cal_rdy_u8);

















 
s8 bma2x2_set_cal_trigger(u8 cal_trigger_u8);
















 
s8 bma2x2_set_offset_rst(u8 offset_rst_u8);





































 
s8 bma2x2_get_offset_target(u8 channel_u8,
u8 *offset_u8);





































 
s8 bma2x2_set_offset_target(u8 channel_u8,
u8 offset_u8);
























 
s8 bma2x2_get_offset(u8 channel_u8,
s8 *offset_u8);
























 
s8 bma2x2_set_offset(u8 channel_u8,
s8 offset_u8);
 
 
 




















 
s8 bma2x2_get_fifo_mode(u8 *fifo_mode_u8);




















 
s8 bma2x2_set_fifo_mode(u8 fifo_mode_u8);




















 
s8 bma2x2_get_fifo_data_select(
u8 *fifo_data_select_u8);




















 
s8 bma2x2_set_fifo_data_select(
u8 fifo_data_select_u8);














 
s8 bma2x2_get_fifo_data_output_reg(
u8 *output_reg_u8);
 
 
 















 
s8 bma2x2_read_temp(s8 *temp_s8);
 
 
 














 
s8 bma2x2_read_accel_xyzt(
struct bma2x2_accel_data_temp *accel);














 
s8 bma2x2_read_accel_eight_resolution_xyzt(
struct bma2x2_accel_eight_resolution_temp *accel);


#line 55 "bst_driver\\bma2x2.c"
 
static struct bma2x2_t *p_bma2x2;
 
u8 V_BMA2x2RESOLUTION_U8 = (2);

















 
s8 bma2x2_burst_read(u8 addr_u8,
u8 *data_u8, u32 len_u32)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->burst_read(p_bma2x2->dev_addr, addr_u8, data_u8, len_u32);

		}
	return com_rslt;
}






















 
s8 bma2x2_init(struct bma2x2_t *bma2x2)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);
	u8 config_data_u8 = ((u8)0);
	 
	p_bma2x2 = bma2x2;
	 
	com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x00), &data_u8, ((u8)1));


	p_bma2x2->chip_id = data_u8;     
	
 
	com_rslt += bma2x2_read_reg((0x3E),
	&config_data_u8, ((u8)1));
	p_bma2x2->fifo_config = config_data_u8;
	return com_rslt;
}

















 
s8 bma2x2_write_reg(u8 adr_u8,
u8 *data_u8, u8 len_u8)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, adr_u8, data_u8, len_u8);

	}
	return com_rslt;
}
















 
s8 bma2x2_read_reg(u8 adr_u8,
u8 *data_u8, u8 len_u8)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, adr_u8, data_u8, len_u8);

		}
	return com_rslt;
}



















 
s8 bma2x2_read_accel_x(s16 *accel_x_s16)
{
	
 
	s8 com_rslt = ((s8)-1);
	


 
	u8	data_u8[(2)] = {
	((u8)0), ((u8)0)};
	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)2));



			*accel_x_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xF0)));
			*accel_x_s16 = *accel_x_s16 >>
			((u8)4);
		break;
		 
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)2));



			*accel_x_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xC0)));
			*accel_x_s16 = *accel_x_s16 >>
			((u8)6);
		break;
		 
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)2));



			*accel_x_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xFC)));
			*accel_x_s16 = *accel_x_s16 >>
			((u8)2);
		break;
		default:
		break;
		}
	}
	return com_rslt;
}















 
s8 bma2x2_read_accel_eight_resolution_x(
s8 *accel_x_s8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8	data = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x03), &data, ((u8)1));



			*accel_x_s8 = ((data & (0xFF)) >> (0));

		}
	return com_rslt;
}


















 
s8 bma2x2_read_accel_y(s16 *accel_y_s16)
{
	
 
	s8 com_rslt = ((s8)-1);
	


 
	u8 data_u8[(2)] = {((u8)0),
	((u8)0)};

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x04), data_u8, ((u8)2));



			*accel_y_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xF0)));
			*accel_y_s16 = *accel_y_s16 >>
			((u8)4);
		break;
		 
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x04), data_u8, ((u8)2));



			*accel_y_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xC0)));
			*accel_y_s16 = *accel_y_s16 >>
			((u8)6);
		break;
		 
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x04), data_u8, ((u8)2));



			*accel_y_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xFC)));
			*accel_y_s16 = *accel_y_s16 >>
			((u8)2);
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
















 
s8 bma2x2_read_accel_eight_resolution_y(
s8 *accel_y_s8)
{
		
 
	s8 com_rslt = ((s8)-1);
	u8	data = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x05), &data, ((u8)1));



			*accel_y_s8 = ((data & (0xFF)) >> (0));

		}
	return com_rslt;
}


















 
s8 bma2x2_read_accel_z(s16 *accel_z_s16)
{
	
 
	s8 com_rslt = ((s8)-1);
	


 
	u8 data_u8[(2)] = {((u8)0),
	((u8)0)};

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x06), data_u8, ((u8)2));



			*accel_z_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)]
			& (0xF0)));
			*accel_z_s16 = *accel_z_s16 >>
			((u8)4);
		break;
		 
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x06), data_u8, ((u8)2));



			*accel_z_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)]
			& (0xC0)));
			*accel_z_s16 = *accel_z_s16 >>
			((u8)6);
		break;
		 
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x06), data_u8, ((u8)2));



			*accel_z_s16 = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)]
			& (0xFC)));
			*accel_z_s16 = *accel_z_s16 >>
			((u8)2);
		break;
		default:
		break;
		}
	}
	return com_rslt;
}
















 
s8 bma2x2_read_accel_eight_resolution_z(
s8 *accel_z_s8)
{
		
 
	s8 com_rslt = ((s8)-1);
	u8	data = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x07), &data, ((u8)1));



			*accel_z_s8 = ((data & (0xFF)) >> (0));

		}
	return com_rslt;
}
















 
s8 bma2x2_read_accel_xyz(
struct bma2x2_accel_data *accel)
{
	
 
	s8 com_rslt = ((s8)-1);
	






 
	u8 data_u8[(6)] = {
	((u8)0), ((u8)0),
	((u8)0), ((u8)0),
	((u8)0), ((u8)0)};

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)6));


			 
			accel->x = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xF0)));
			accel->x = accel->x >> ((u8)4);

			 
			accel->y = (s16)((((s32)((s8)
			data_u8[(3)]))
			<< ((u8)8)) |
			(data_u8[(2)] &
			(0xF0)));
			accel->y = accel->y >> ((u8)4);

			 
			accel->z = (s16)((((s32)((s8)
			data_u8[(5)]))
			<< ((u8)8)) |
			(data_u8[(4)] &
			(0xF0)));
			accel->z = accel->z >> ((u8)4);

		break;
		case (1):
		 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)6));


			 
			accel->x = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8)) |
			(data_u8[(0)] &
			(0xC0)));
			accel->x = accel->x >> ((u8)6);

			 
			accel->y = (s16)((((s32)((s8)
			data_u8[(3)]))
			<< ((u8)8)) |
			(data_u8[(2)] &
			(0xC0)));
			accel->y = accel->y >> ((u8)6);

			 
			accel->z = (s16)((((s32)((s8)
			data_u8[(5)]))
			<< ((u8)8)) |
			(data_u8[(4)]
			& (0xC0)));
			accel->z = accel->z >> ((u8)6);
		break;
		 
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)6));



			 
			accel->x = (s16)((((s32)((s8)
			data_u8[(1)]))<<
			((u8)8)) |
			(data_u8[(0)]
			& (0xFC)));
			accel->x = accel->x >> ((u8)2);

			 
			accel->y = (s16)((((s32)((s8)
			data_u8[(3)]))<<
			((u8)8)) |
			(data_u8[(2)]
			& (0xFC)));
			accel->y = accel->y >> ((u8)2);

			 
			accel->z = (s16)((((s32)((s8)
			data_u8[(5)]))<<
			((u8)8)) |
			(data_u8[(4)]
			& (0xFC)));
			accel->z = accel->z >> ((u8)2);
		break;
		default:
		break;
		}
	}
	return com_rslt;
}

















 
s8 bma2x2_read_accel_eight_resolution_xyz(
struct bma2x2_accel_eight_resolution *accel)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8	data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x03), &data_u8, ((u8)1));



		accel->x = ((data_u8 & (0xFF)) >> (0));


		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x05), &data_u8, ((u8)1));



		accel->y = ((data_u8 & (0xFF)) >> (0));


		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x07), &data_u8, ((u8)1));



		accel->z = ((data_u8 & (0xFF)) >> (0));

		}
	return com_rslt;
}














 
s8 bma2x2_get_intr_tap_stat(
u8 *stat_tap_u8)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0B), stat_tap_u8, ((u8)1));


		}
	return com_rslt;
}














 
s8 bma2x2_get_intr_orient_stat(
u8 *stat_orient_u8)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0C), stat_orient_u8, ((u8)1));


		}
	return com_rslt;
}













 
s8 bma2x2_get_fifo_stat(
u8 *stat_fifo_u8)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0E), stat_fifo_u8, ((u8)1));



		}
	return com_rslt;
}














 
s8 bma2x2_get_fifo_frame_count(
u8 *frame_count_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0E), &data_u8, ((u8)1));



			*frame_count_u8 = ((data_u8 & (0x7F)) >> (0));

		}
	return com_rslt;
}














 
s8 bma2x2_get_fifo_overrun(
u8 *fifo_overrun_u8)
{
		
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0E), &data_u8, ((u8)1));



			*fifo_overrun_u8 = ((data_u8 & (0x80)) >> (7));

		}
	return com_rslt;
}















 
s8 bma2x2_get_intr_stat(
u8 *intr_stat_u8)
{
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x09), intr_stat_u8, ((u8)4));



		}
	return com_rslt;
}




















 
s8 bma2x2_get_range(u8 *range_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0F), &data_u8, ((u8)1));


		data_u8 = ((data_u8 & (0x0F)) >> (0));
		*range_u8 = data_u8;
	}
	return com_rslt;
}




















 
s8 bma2x2_set_range(u8 range_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if ((range_u8 == (3)) ||
		(range_u8 == (5)) ||
		(range_u8 == (8)) ||
		(range_u8 == (12))) {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x0F), &data_u8, ((u8)1));



			switch (range_u8) {
			case (3):
				data_u8  = ((data_u8 & ~(0x0F)) | (((3)<<(0))&(0x0F)));


			break;
			case (5):
				data_u8  = ((data_u8 & ~(0x0F)) | (((5)<<(0))&(0x0F)));


			break;
			case (8):
				data_u8  = ((data_u8 & ~(0x0F)) | (((8)<<(0))&(0x0F)));


			break;
			case (12):
				data_u8  = ((data_u8 & ~(0x0F)) | (((12)<<(0))&(0x0F)));


			break;
			default:
			break;
			}
			 
			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x0F), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}
























 
s8 bma2x2_get_bw(u8 *bw_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x10), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & (0x1F)) >> (0));
			*bw_u8 = data_u8;
		}
	return com_rslt;
}

























 
s8 bma2x2_set_bw(u8 bw_u8)
{

 
s8 com_rslt = ((s8)-1);
u8 data_u8 = ((u8)0);
u8 data_bw_u8 = ((u8)0);
if (p_bma2x2 == ((void *)0)) {
		 
		com_rslt = ((s8)-127);
	} else {
	 
	if (p_bma2x2->chip_id == (0xFB)) {
		if (bw_u8 > ((u8)7) &&
		bw_u8 < ((u8)15)) {
			switch (bw_u8) {
			case (0x08):
				data_bw_u8 = (0x08);

				 
			break;
			case (0x09):
				data_bw_u8 = (0x09);

			 
			break;
			case (0x0A):
				data_bw_u8 = (0x0A);

			 
			break;
			case (0x0B):
				data_bw_u8 = (0x0B);

			 
			break;
			case (0x0C):
				data_bw_u8 = (0x0C);

			 
			break;
			case (0x0D):
				data_bw_u8 = (0x0D);

			 
			break;
			case (0x0E):
				data_bw_u8 = (0x0E);

			 
			break;
			default:
			break;
			}
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x10), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x1F)) | ((data_bw_u8<<(0))&(0x1F)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x10), &data_u8, ((u8)1));



			} else {
			com_rslt = ((s8)-2);
			}
		} else {
		if (bw_u8 > ((u8)7) &&
		bw_u8 < ((u8)16)) {
			switch (bw_u8) {
			case (0x08):
				data_bw_u8 = (0x08);

			 
			break;
			case (0x09):
				data_bw_u8 = (0x09);

			 
			break;
			case (0x0A):
				data_bw_u8 = (0x0A);

			 
			break;
			case (0x0B):
				data_bw_u8 = (0x0B);

			 
			break;
			case (0x0C):
				data_bw_u8 = (0x0C);

			 
			break;
			case (0x0D):
				data_bw_u8 = (0x0D);

			 
			break;
			case (0x0E):
				data_bw_u8 = (0x0E);

			 
			break;
			case (0x0F):
				data_bw_u8 = (0x0F);

			 
			break;
			default:
			break;
			}
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x10), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x1F)) | ((data_bw_u8<<(0))&(0x1F)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x10), &data_u8, ((u8)1));



			} else {
			com_rslt = ((s8)-2);
			}
		}
	}
	return com_rslt;
}
























 
s8 bma2x2_get_power_mode(
u8 *power_mode_u8)
{
	
 
s8 com_rslt = ((s8)-1);
u8 data_u8 = ((u8)0);
u8 data2_u8 = ((u8)0);
if (p_bma2x2 == ((void *)0)) {
	 
		com_rslt = ((s8)-127);
	} else {
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));


		com_rslt += p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x12), &data2_u8, ((u8)1));



		data_u8  = (data_u8 &
		(0xE0)) >>
		((u8)5);
		data2_u8  = (data2_u8 &
		(0x40)) >>
		((u8)6);

	if ((data_u8 ==
	(0x00)) &&
	(data2_u8 ==
	(0x00))) {
		*power_mode_u8  = (0);
		} else {
		if ((data_u8 ==
		(0x02)) &&
		(data2_u8 ==
		(0x00))) {
			*power_mode_u8  =
			(1);
			} else {
			if ((data_u8 ==
			(0x04)
			|| data_u8 ==
			(0x06)) &&
			(data2_u8 ==
			(0x00))) {
				*power_mode_u8  =
				(2);
				} else {
				if (((data_u8 &
				(0x01))
				== (0x01))) {
					*power_mode_u8  =
					(3);
					} else {
					if ((data_u8 ==
					(0x02))
					&& (data2_u8 ==
					(0x01))) {
						*power_mode_u8  =
						(4);
					} else {
					if ((data_u8 ==
					(0x04)) &&
					(data2_u8 ==
					(0x01)))
						*power_mode_u8  =
							(5);
					else
						*power_mode_u8 =
						(3);
						}
					}
				}
			}
		}
	}
	p_bma2x2->power_mode_u8 = *power_mode_u8;
return com_rslt;
}
























 
s8 bma2x2_set_power_mode(u8 power_mode_u8)
{
		
 
	s8 com_rslt = ((s8)-1);
	u8 mode_ctr_eleven_reg = ((u8)0);
	u8 mode_ctr_twel_reg = ((u8)0);
	u8 data_u8 = ((u8)0);
	u8 data2_u8 = ((u8)0);
	u8 pre_fifo_config_data = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		com_rslt = ((s8)-127);
	} else {
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));



		com_rslt += p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x12), &data2_u8, ((u8)1));




		com_rslt += bma2x2_set_mode_value(power_mode_u8);
		mode_ctr_eleven_reg = p_bma2x2->ctrl_mode_reg;
		mode_ctr_twel_reg =  p_bma2x2->low_mode_reg;
		
 
		data2_u8  = ((data2_u8 & ~(0x40)) | ((mode_ctr_twel_reg<<(6))&(0x40)));


		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x12), &data2_u8, ((u8)1));


		


 
		p_bma2x2->delay_msec(((u8)1));
		 
		data_u8  = ((data_u8 & ~(0xE0)) | ((((u8)4)<<(5))&(0xE0)));


		 
		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));


		


 
		p_bma2x2->delay_msec(((u8)1));
		 
		pre_fifo_config_data = p_bma2x2->fifo_config;
		com_rslt += bma2x2_write_reg((0x3E),
		&pre_fifo_config_data, ((u8)1));
		


 
		p_bma2x2->delay_msec(((u8)1));
		com_rslt += p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));



		 
		data_u8  = ((data_u8 & ~(0xE0)) | ((mode_ctr_eleven_reg<<(5))&(0xE0)));


		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));


		


 
		p_bma2x2->delay_msec(((u8)1));
	}
	return com_rslt;
}
























 
s8 bma2x2_set_mode_value(u8 power_mode_u8)
{
	s8 com_rslt = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		com_rslt = ((s8)-127);
	} else {
	if (power_mode_u8 < ((u8)6)) {
		switch (power_mode_u8)	{
		case (0):
			p_bma2x2->ctrl_mode_reg =
			(0x00);
			p_bma2x2->low_mode_reg =
			(0x00);
		break;
		case (1):
			p_bma2x2->ctrl_mode_reg =
			(0x02);
			p_bma2x2->low_mode_reg =
			(0x00);
		break;
		case (4):
			p_bma2x2->ctrl_mode_reg =
			(0x02);
			p_bma2x2->low_mode_reg =
			(0x01);
		break;
		case (2):
			p_bma2x2->ctrl_mode_reg =
			(0x04);
			p_bma2x2->low_mode_reg =
			(0x00);
		break;
		case (5):
			p_bma2x2->ctrl_mode_reg =
			(0x04);
			p_bma2x2->low_mode_reg =
			(0x01);
		break;
		case (3):
			p_bma2x2->ctrl_mode_reg =
			(0x01);
		break;
		}
		} else {
			com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_get_sleep_durn(u8 *sleep_durn_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));


			*sleep_durn_u8 = ((data_u8 & (0x1E)) >> (1));

		}
	return com_rslt;
}






























 
s8 bma2x2_set_sleep_durn(u8 sleep_durn_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_sleep_durn_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (sleep_durn_u8 > ((u8)4) &&
		sleep_durn_u8 < ((u8)16)) {
			switch (sleep_durn_u8) {
			case (0x05):
				data_sleep_durn_u8 = (0x05);

				 
			break;
			case (0x06):
				data_sleep_durn_u8 = (0x06);

				 
			break;
			case (0x07):
				data_sleep_durn_u8 = (0x07);

				 
			break;
			case (0x08):
				data_sleep_durn_u8 = (0x08);

				 
			break;
			case (0x09):
				data_sleep_durn_u8 = (0x09);

				 
			break;
			case (0x0A):
				data_sleep_durn_u8 = (0x0A);

				 
			break;
			case (0x0B):
				data_sleep_durn_u8 = (0x0B);

				 
			break;
			case (0x0C):
				data_sleep_durn_u8 = (0x0C);

				 
			break;
			case (0x0D):
				data_sleep_durn_u8 = (0x0D);

				 
			break;
			case (0x0E):
				data_sleep_durn_u8 = (0x0E);

				 
			break;
			case (0x0F):
				data_sleep_durn_u8 = (0x0F);

				 
			break;
			default:
			break;
			}
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x1E)) | ((data_sleep_durn_u8<<(1))&(0x1E)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x11), &data_u8, ((u8)1));


		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}



















 
s8 bma2x2_get_sleep_timer_mode(
u8 *sleep_timer_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x12), &data_u8, ((u8)1));


			*sleep_timer_u8 = ((data_u8 & (0x20)) >> (5));

		}
	return com_rslt;
}



















 
s8 bma2x2_set_sleep_timer_mode(u8 sleep_timer_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (sleep_timer_u8 < ((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x12), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x20)) | ((sleep_timer_u8<<(5))&(0x20)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x12), &data_u8, ((u8)1));


		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}
















 
s8 bma2x2_get_high_bw(u8 *high_bw_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		return  ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x13), &data_u8, ((u8)1));


			*high_bw_u8 = ((data_u8 & (0x80)) >> (7));

		}
	return com_rslt;
}
















 
s8 bma2x2_set_high_bw(u8 high_bw_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		return  ((s8)-127);
		}  else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x13), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x80)) | ((high_bw_u8<<(7))&(0x80)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x13), &data_u8, ((u8)1));



		}
	return com_rslt;
}

















 
s8 bma2x2_get_shadow_dis(u8 *shadow_dis_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		return  ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x13), &data_u8, ((u8)1));



			*shadow_dis_u8 = ((data_u8 & (0x40)) >> (6));

		}
	return com_rslt;
}

















 
s8 bma2x2_set_shadow_dis(u8 shadow_dis_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		return  ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x13), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x40)) | ((shadow_dis_u8<<(6))&(0x40)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x13), &data_u8, ((u8)1));


		}
	return com_rslt;
}














 
s8 bma2x2_soft_rst(void)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = (0xB6);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		}  else {
			
 
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x14), &data_u8, ((u8)1));


		}
	return com_rslt;
}















 
s8 bma2x2_update_image(void)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		return  ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x08)) | ((((u8)1)<<(3))&(0x08)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));


		}
	return com_rslt;
}











































 
s8 bma2x2_get_intr_enable(u8 intr_type_u8,
u8 *value_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (intr_type_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x08)) >> (3));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (3):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		case (4):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x10)) >> (4));

		break;
		case (5):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (6):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (7):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		case (8):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x20)) >> (5));

		break;
		case (9):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x10)) >> (4));

		break;
		case (10):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x40)) >> (6));

		break;
		case (11):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));



			*value_u8 = ((data_u8 & (0x80)) >> (7));

		break;
		default:
		com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}











































 
s8 bma2x2_set_intr_enable(u8 intr_type_u8,
u8 value_u8)
{
		
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);
	u8 data2_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));


		com_rslt += p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data2_u8, ((u8)1));


		value_u8 = value_u8 & ((u8)1);
		switch (intr_type_u8) {
		case (0):
			 
			data2_u8 = ((data2_u8 & ~(0x08)) | ((value_u8<<(3))&(0x08)));

		break;
		case (1):
			 
			data2_u8 = ((data2_u8 & ~(0x01)) | ((value_u8<<(0))&(0x01)));

		break;
		case (2):
			 
			data2_u8 = ((data2_u8 & ~(0x02)) | ((value_u8<<(1))&(0x02)));

		break;
		case (3):
			 
			data2_u8 = ((data2_u8 & ~(0x04)) | ((value_u8<<(2))&(0x04)));

		break;
		case (4):
			 
			data2_u8 = ((data2_u8 & ~(0x10)) | ((value_u8<<(4))&(0x10)));

		break;
		case (5):
			 
			data_u8 = ((data_u8 & ~(0x01)) | ((value_u8<<(0))&(0x01)));

		break;
		case (6):
			 
			data_u8 = ((data_u8 & ~(0x02)) | ((value_u8<<(1))&(0x02)));

		break;
		case (7):
			 
			data_u8 = ((data_u8 & ~(0x04)) | ((value_u8<<(2))&(0x04)));

		break;
		case (8):
			 
			data_u8 = ((data_u8 & ~(0x20)) | ((value_u8<<(5))&(0x20)));

		break;
		case (9):
			 
			data_u8 = ((data_u8 & ~(0x10)) | ((value_u8<<(4))&(0x10)));

		break;
		case (10):
			 
			data_u8 = ((data_u8 & ~(0x40)) | ((value_u8<<(6))&(0x40)));

		break;
		case (11):
			 
			data_u8 = ((data_u8 & ~(0x80)) | ((value_u8<<(7))&(0x80)));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
		 
		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x16), &data_u8, ((u8)1));


		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x17), &data2_u8, ((u8)1));


	}
	return com_rslt;
}



















 
s8 bma2x2_get_intr_fifo_full(u8 *fifo_full_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*fifo_full_u8 = ((data_u8 & (0x20)) >> (5));

		}
	return com_rslt;
}



















 
s8 bma2x2_set_intr_fifo_full(u8 fifo_full_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (fifo_full_u8 < ((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x20)) | ((fifo_full_u8<<(5))&(0x20)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}





















 
s8 bma2x2_get_intr_fifo_wm(u8 *fifo_wm_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			*fifo_wm_u8 = ((data_u8 & (0x40)) >> (6));

		}
	return com_rslt;
}





















 
s8 bma2x2_set_intr_fifo_wm(u8 fifo_wm_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (fifo_wm_u8 < ((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x40)) | ((fifo_wm_u8<<(6))&(0x40)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x17), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}


























 
s8 bma2x2_get_slow_no_motion(u8 channel_u8,
u8 *slow_no_motion_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			*slow_no_motion_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			*slow_no_motion_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			*slow_no_motion_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		case (3):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			*slow_no_motion_u8 = ((data_u8 & (0x08)) >> (3));


		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_set_slow_no_motion(u8 channel_u8,
u8 slow_no_motion_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((slow_no_motion_u8<<(0))&(0x01)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((slow_no_motion_u8<<(1))&(0x02)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((slow_no_motion_u8<<(2))&(0x04)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



		break;
		case (3):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x08)) | ((slow_no_motion_u8<<(3))&(0x08)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x18), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_get_intr_low_g(u8 channel_u8,
u8 *intr_low_g_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_low_g_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_low_g_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_set_intr_low_g(u8 channel_u8,
u8 intr_low_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((intr_low_u8<<(0))&(0x01)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((intr_low_u8<<(0))&(0x01)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

























 
s8 bma2x2_get_intr_high_g(u8 channel_u8,
u8 *intr_high_g_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_high_g_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_high_g_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		default:
		com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

























 
s8 bma2x2_set_intr_high_g(u8 channel_u8,
u8 intr_high_g_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((intr_high_g_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((intr_high_g_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
		com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_get_intr_slope(u8 channel_u8,
u8 *intr_slope_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_slope_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_slope_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		default:
		com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_set_intr_slope(u8 channel_u8,
u8 intr_slope_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((intr_slope_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((intr_slope_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_get_intr_slow_no_motion(u8 channel_u8,
u8 *intr_slow_no_motion_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_slow_no_motion_u8 = ((data_u8 & (0x08)) >> (3));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_slow_no_motion_u8 = ((data_u8 & (0x08)) >> (3));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_set_intr_slow_no_motion(u8 channel_u8,
u8 intr_slow_no_motion_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x08)) | ((intr_slow_no_motion_u8<<(3))&(0x08)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x08)) | ((intr_slow_no_motion_u8<<(3))&(0x08)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_get_intr_double_tap(u8 channel_u8,
u8 *intr_double_tap_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_double_tap_u8 = ((data_u8 & (0x10)) >> (4));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_double_tap_u8 = ((data_u8 & (0x10)) >> (4));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_set_intr_double_tap(u8 channel_u8,
u8 intr_double_tap_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x10)) | ((intr_double_tap_u8<<(4))&(0x10)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x10)) | ((intr_double_tap_u8<<(4))&(0x10)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
		com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}


























 
s8 bma2x2_get_intr_single_tap(u8 channel_u8,
u8 *intr_single_tap_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_single_tap_u8 = ((data_u8 & (0x20)) >> (5));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_single_tap_u8 = ((data_u8 & (0x20)) >> (5));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}


























 
s8 bma2x2_set_intr_single_tap(u8 channel_u8,
u8 intr_single_tap_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x20)) | ((intr_single_tap_u8<<(5))&(0x20)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x20)) | ((intr_single_tap_u8<<(5))&(0x20)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

























 
s8 bma2x2_get_intr_orient(u8 channel_u8,
u8 *intr_orient_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_orient_u8 = ((data_u8 & (0x40)) >> (6));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_orient_u8 = ((data_u8 & (0x40)) >> (6));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

























 
s8 bma2x2_set_intr_orient(u8 channel_u8,
u8 intr_orient_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x40)) | ((intr_orient_u8<<(6))&(0x40)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x40)) | ((intr_orient_u8<<(6))&(0x40)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_get_intr_flat(u8 channel_u8,
u8 *intr_flat_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			*intr_flat_u8 = ((data_u8 & (0x80)) >> (7));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			*intr_flat_u8 = ((data_u8 & (0x80)) >> (7));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























 
s8 bma2x2_set_intr_flat(u8 channel_u8,
u8 intr_flat_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x80)) | ((intr_flat_u8<<(7))&(0x80)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x19), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x80)) | ((intr_flat_u8<<(7))&(0x80)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1B), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}


























 
s8 bma2x2_get_new_data(u8 channel_u8,
u8 *intr_newdata_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			*intr_newdata_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			*intr_newdata_u8 = ((data_u8 & (0x80)) >> (7));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}


























 
s8 bma2x2_set_new_data(u8 channel_u8,
u8 intr_newdata_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((intr_newdata_u8<<(0))&(0x01)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x80)) | ((intr_newdata_u8<<(7))&(0x80)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

















 
s8 bma2x2_get_intr1_fifo_wm(u8 *intr1_fifo_wm_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			*intr1_fifo_wm_u8 = ((data_u8 & (0x02)) >> (1));

		}
	return com_rslt;
}

















 
s8 bma2x2_set_intr1_fifo_wm(u8 intr1_fifo_wm_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (intr1_fifo_wm_u8 <
		((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((intr1_fifo_wm_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}

















 
s8 bma2x2_get_intr2_fifo_wm(u8 *intr2_fifo_wm_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			*intr2_fifo_wm_u8 = ((data_u8 & (0x40)) >> (6));

		}
	return com_rslt;
}

















 
s8 bma2x2_set_intr2_fifo_wm(u8 intr2_fifo_wm_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (intr2_fifo_wm_u8 <
		((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x40)) | ((intr2_fifo_wm_u8<<(6))&(0x40)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}



















 
s8 bma2x2_get_intr1_fifo_full(u8 *intr1_fifo_full_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			*intr1_fifo_full_u8 = ((data_u8 & (0x04)) >> (2));

		}
	return com_rslt;
}



















 
s8 bma2x2_set_intr1_fifo_full(u8 intr1_fifo_full_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (intr1_fifo_full_u8 <
		((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((intr1_fifo_full_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			} else {
			com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}




















 
s8 bma2x2_get_intr2_fifo_full(u8 *intr2_fifo_full_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			*intr2_fifo_full_u8 = ((data_u8 & (0x20)) >> (5));

		}
	return com_rslt;
}




















 
s8 bma2x2_set_intr2_fifo_full(u8 intr2_fifo_full_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (intr2_fifo_full_u8 <
		((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x20)) | ((intr2_fifo_full_u8<<(5))&(0x20)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1A), &data_u8, ((u8)1));



			} else {
			com_rslt = ((s8)-2);
			}
		}
	return com_rslt;
}




























 
s8 bma2x2_get_source(u8 channel_u8,
u8 *intr_source_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		return  ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			*intr_source_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			*intr_source_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			*intr_source_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		case (3):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			*intr_source_u8 = ((data_u8 & (0x08)) >> (3));

		break;
		case (4):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			*intr_source_u8 = ((data_u8 & (0x10)) >> (4));

		break;
		case (5):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			*intr_source_u8 = ((data_u8 & (0x20)) >> (5));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
			}
		}
	return com_rslt;
}




























 
s8 bma2x2_set_source(u8 channel_u8,
u8 intr_source_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);
		if (p_bma2x2 == ((void *)0)) {
			com_rslt = ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((intr_source_u8<<(0))&(0x01)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((intr_source_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((intr_source_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



		break;
		case (3):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x08)) | ((intr_source_u8<<(3))&(0x08)));



			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



		break;
		case (4):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x10)) | ((intr_source_u8<<(4))&(0x10)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



		break;
		case (5):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x20)) | ((intr_source_u8<<(5))&(0x20)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x1E), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

























 
s8 bma2x2_get_intr_output_type(u8 channel_u8,
u8 *intr_output_type_u8)
{
		u8 data_u8 = ((u8)0);
		
 
		s8 com_rslt = ((s8)-1);

		if (p_bma2x2 == ((void *)0)) {
			com_rslt = ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			*intr_output_type_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			*intr_output_type_u8 = ((data_u8 & (0x08)) >> (3));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

























 
s8 bma2x2_set_intr_output_type(u8 channel_u8,
u8 intr_output_type_u8)
{
		u8 data_u8 = ((u8)0);
		
 
		s8 com_rslt = ((s8)-1);

		if (p_bma2x2 == ((void *)0)) {
			com_rslt = ((s8)-127);
		}  else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((intr_output_type_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x08)) | ((intr_output_type_u8<<(3))&(0x08)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}
























 
s8 bma2x2_get_intr_level(u8 channel_u8,
u8 *intr_level_u8)
{
		u8 data_u8 = ((u8)0);
		
 
		s8 com_rslt = ((s8)-1);

		if (p_bma2x2 == ((void *)0)) {
			com_rslt = ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			*intr_level_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			*intr_level_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}
























 
s8 bma2x2_set_intr_level(u8 channel_u8,
u8 intr_level_u8)
{
		u8 data_u8 = ((u8)0);
		
 
		s8 com_rslt = ((s8)-1);

		if (p_bma2x2 == ((void *)0)) {
			com_rslt = ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((intr_level_u8<<(0))&(0x01)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((intr_level_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x20), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}


















 
s8 bma2x2_rst_intr(u8 rst_intr_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x21), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x80)) | ((rst_intr_u8<<(7))&(0x80)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x21), &data_u8, ((u8)1));


		}
	return com_rslt;
}































 
s8 bma2x2_get_latch_intr(u8 *latch_intr_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x21), &data_u8, ((u8)1));


			*latch_intr_u8 = ((data_u8 & (0x0F)) >> (0));

		}
	return com_rslt;
}































 
s8 bma2x2_set_latch_intr(u8 latch_intr_u8)
{
u8 data_u8 = ((u8)0);

 
s8 com_rslt = ((s8)-1);
u8 latch_durn_u8 = ((u8)0);
if (p_bma2x2 == ((void *)0))  {
		 
		return ((s8)-127);
		} else  {
		if (latch_intr_u8 < ((u8)16)) {
			switch (latch_intr_u8) {
			case (0x00):
				latch_durn_u8 = (0x00);

				 
			break;
			case (0x01):
				latch_durn_u8 = (0x01);

				 
			break;
			case (0x02):
				latch_durn_u8 = (0x02);

				 
			break;
			case (0x03):
				latch_durn_u8 = (0x03);

				 
			break;
			case (0x04):
				latch_durn_u8 = (0x04);

				 
			break;
			case (0x05):
				latch_durn_u8 = (0x05);

				 
			break;
			case (0x06):
				latch_durn_u8 = (0x06);

				 
			break;
			case (0x07):
				latch_durn_u8 = (0x07);

				 
			break;
			case (0x08):
				latch_durn_u8 = (0x08);

				 
			break;
			case (0x09):
				latch_durn_u8 = (0x09);

				 
			break;
			case (0x0A):
				latch_durn_u8 = (0x0A);

				 
			break;
			case (0x0B):
				latch_durn_u8 = (0x0B);

				 
			break;
			case (0x0C):
				latch_durn_u8 = (0x0C);

				 
			break;
			case (0x0D):
				latch_durn_u8 = (0x0D);

				 
			break;
			case (0x0E):
				latch_durn_u8 = (0x0E);

				 
			break;
			case (0x0F):
				latch_durn_u8 = (0x0F);

				 
			break;
			default:
			break;
			}
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x21), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x0F)) | ((latch_durn_u8<<(0))&(0x0F)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x21), &data_u8, ((u8)1));


		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}






































 
s8 bma2x2_get_durn(u8 channel_u8,
u8 *durn_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		 
		switch (channel_u8) {
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x22), &data_u8, ((u8)1));


			*durn_u8 = data_u8;
		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x25), &data_u8, ((u8)1));


			*durn_u8 = data_u8;
		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x27), &data_u8, ((u8)1));


			*durn_u8 = ((data_u8 & (0x03)) >> (0));

		break;
		case (3):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x27), &data_u8, ((u8)1));



			*durn_u8 = ((data_u8 & (0xFC)) >> (2));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}






































 
s8 bma2x2_set_durn(u8 channel_u8,
u8 durn_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0))  {
		 
		return ((s8)-127);
		}  else  {
		 
		switch (channel_u8)   {
		case (0):
			 
			data_u8 = durn_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x22), &data_u8, ((u8)1));


		break;
		case (1):
			 
			data_u8 = durn_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x25), &data_u8, ((u8)1));



		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x27), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x03)) | ((durn_u8<<(0))&(0x03)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x27), &data_u8, ((u8)1));



		break;
		case (3):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x27), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0xFC)) | ((durn_u8<<(2))&(0xFC)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x27), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























































 
s8 bma2x2_get_thres(u8 channel_u8,
u8 *thres_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x23), &data_u8, ((u8)1));


			*thres_u8 = data_u8;
		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x26), &data_u8, ((u8)1));



			*thres_u8 = data_u8;
		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x28), &data_u8, ((u8)1));



			*thres_u8 = data_u8;
		break;
		case (3):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x29), &data_u8, ((u8)1));



			*thres_u8 = data_u8;
		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}



























































 
s8 bma2x2_set_thres(u8 channel_u8,
u8 thres_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			 
			data_u8 = thres_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x23), &data_u8, ((u8)1));



		break;
		case (1):
			 
			data_u8 = thres_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x26), &data_u8, ((u8)1));



		break;
		case (2):
			 
			data_u8 = thres_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x28), &data_u8, ((u8)1));



		break;
		case (3):
			 
			data_u8 = thres_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x29), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}































 
s8 bma2x2_get_low_high_g_hyst(u8 channel_u8,
u8 *hyst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



			*hyst_u8 = ((data_u8 & (0x03)) >> (0));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



			*hyst_u8 = ((data_u8 & (0xC0)) >> (6));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}































 
s8 bma2x2_set_low_high_g_hyst(u8 channel_u8,
u8 hyst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0x03)) | ((hyst_u8<<(0))&(0x03)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0xC0)) | ((hyst_u8<<(6))&(0xC0)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

















 
s8 bma2x2_get_low_g_mode(u8 *low_g_mode_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));


			*low_g_mode_u8 = ((data_u8 & (0x04)) >> (2));

		}
	return com_rslt;
}

















 
s8 bma2x2_set_low_g_mode(u8 low_g_mode_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((low_g_mode_u8<<(2))&(0x04)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x24), &data_u8, ((u8)1));



		}
	return com_rslt;
}
























 
s8 bma2x2_get_tap_durn(u8 *tap_durn_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



			*tap_durn_u8 = ((data_u8 & (0x07)) >> (0));

		}
	return com_rslt;
}
























 
s8 bma2x2_set_tap_durn(u8 tap_durn_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x07)) | ((tap_durn_u8<<(0))&(0x07)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



		}
	return com_rslt;
}


















 
s8 bma2x2_get_tap_shock(u8 *tap_shock_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



			*tap_shock_u8 = ((data_u8 & (0x40)) >> (6));

		}
	return com_rslt;
}


















 
s8 bma2x2_set_tap_shock(u8 tap_shock_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x40)) | ((tap_shock_u8<<(6))&(0x40)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



		}
	return com_rslt;
}

















 
s8 bma2x2_get_tap_quiet(u8 *tap_quiet_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



			*tap_quiet_u8 = ((data_u8 & (0x80)) >> (7));

		}
	return com_rslt;
}

















 
s8 bma2x2_set_tap_quiet(u8 tap_quiet_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x80)) | ((tap_quiet_u8<<(7))&(0x80)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2A), &data_u8, ((u8)1));



		}
	return com_rslt;
}






















 
s8 bma2x2_get_tap_thres(u8 *tap_thres_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2B), &data_u8, ((u8)1));



			*tap_thres_u8 = ((data_u8 & (0x1F)) >> (0));

		}
	return com_rslt;
}





















 
s8 bma2x2_set_tap_thres(u8 tap_thres_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2B), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x1F)) | ((tap_thres_u8<<(0))&(0x1F)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2B), &data_u8, ((u8)1));



		}
	return com_rslt;
}




















 
s8 bma2x2_get_tap_sample(u8 *tap_sample_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2B), &data_u8, ((u8)1));



			*tap_sample_u8 = ((data_u8 & (0xC0)) >> (6));

		}
	return com_rslt;
}




















 
s8 bma2x2_set_tap_sample(u8 tap_sample_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2B), &data_u8, ((u8)1));


			data_u8 = ((data_u8 & ~(0xC0)) | ((tap_sample_u8<<(6))&(0xC0)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2B), &data_u8, ((u8)1));



		}
	return com_rslt;
}





















 
s8 bma2x2_get_orient_mode(u8 *orient_mode_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



			*orient_mode_u8 = ((data_u8 & (0x03)) >> (0));

		}
	return com_rslt;
}





















 
s8 bma2x2_set_orient_mode(u8 orient_mode_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x03)) | ((orient_mode_u8<<(0))&(0x03)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



		}
	return com_rslt;
}


























 
s8 bma2x2_get_orient_block(
u8 *orient_block_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



			*orient_block_u8 = ((data_u8 & (0x0C)) >> (2));

		}
	return com_rslt;
}


























 
s8 bma2x2_set_orient_block(u8 orient_block_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x0C)) | ((orient_block_u8<<(2))&(0x0C)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



		}
	return com_rslt;
}















 
s8 bma2x2_get_orient_hyst(u8 *orient_hyst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



			*orient_hyst_u8 = ((data_u8 & (0x70)) >> (4));

		}
	return com_rslt;
}















 
s8 bma2x2_set_orient_hyst(u8 orient_hyst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x70)) | ((orient_hyst_u8<<(4))&(0x70)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2C), &data_u8, ((u8)1));



		}
	return com_rslt;
}























 
s8 bma2x2_get_theta(u8 channel_u8,
u8 *theta_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2D), &data_u8, ((u8)1));



			*theta_u8 = ((data_u8 & (0x3F)) >> (0));

		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2E), &data_u8, ((u8)1));



			*theta_u8 = data_u8;
		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}























 
s8 bma2x2_set_theta(u8 channel_u8,
u8 theta_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		 
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2D), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x3F)) | ((theta_u8<<(0))&(0x3F)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2D), &data_u8, ((u8)1));



		break;
		case (1):
			 
			data_u8 = theta_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2E), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}


















 
s8 bma2x2_get_orient_enable(u8 *orient_enable_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2D), &data_u8, ((u8)1));



			*orient_enable_u8 = ((data_u8 & (0x40)) >> (6));

		}
	return com_rslt;
}


















 
s8 bma2x2_set_orient_enable(u8 orient_enable_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2D), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x40)) | ((orient_enable_u8<<(6))&(0x40)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2D), &data_u8, ((u8)1));



		}
	return com_rslt;
}

















 
s8 bma2x2_get_flat_hyst(u8 *flat_hyst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2F), &data_u8, ((u8)1));



			*flat_hyst_u8 = ((data_u8 & (0x07)) >> (0));

		}
	return com_rslt;
}

















 
s8 bma2x2_set_flat_hyst(u8 flat_hyst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2F), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x07)) | ((flat_hyst_u8<<(0))&(0x07)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2F), &data_u8, ((u8)1));



		}
	return com_rslt;
}





















 
s8 bma2x2_get_flat_hold_time(
u8 *flat_hold_time_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2F), &data_u8, ((u8)1));



			*flat_hold_time_u8 = ((data_u8 & (0x30)) >> (4));

		}
	return com_rslt;
}





















 
s8 bma2x2_set_flat_hold_time(
u8 flat_hold_time_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x2F), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x30)) | ((flat_hold_time_u8<<(4))&(0x30)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x2F), &data_u8, ((u8)1));



		}
	return com_rslt;
}
















 
s8 bma2x2_get_fifo_wml_trig(
u8 *fifo_wml_trig)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x30), &data_u8, ((u8)1));



			*fifo_wml_trig = ((data_u8 & (0x3F)) >> (0));

		}
	return com_rslt;
}
















 
s8 bma2x2_set_fifo_wml_trig(
u8 fifo_wml_trig)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (fifo_wml_trig < ((u8)32)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x30), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x3F)) | ((fifo_wml_trig<<(0))&(0x3F)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x30), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}




















 
s8 bma2x2_get_selftest_axis(
u8 *selftest_axis_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x32), &data_u8, ((u8)1));



			*selftest_axis_u8 = ((data_u8 & (0x03)) >> (0));

		}
	return com_rslt;
}




















 
s8 bma2x2_set_selftest_axis(
u8 selftest_axis_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (selftest_axis_u8 < ((u8)4)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x32), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x03)) | ((selftest_axis_u8<<(0))&(0x03)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x32), &data_u8, ((u8)1));



		 } else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}


















 
s8 bma2x2_get_selftest_sign(
u8 *selftest_sign_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x32), &data_u8, ((u8)1));



			*selftest_sign_u8 = ((data_u8 & (0x04)) >> (2));

		}
	return com_rslt;
}


















 
s8 bma2x2_set_selftest_sign(
u8 selftest_sign_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (selftest_sign_u8 <
		((u8)2)) {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x32), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((selftest_sign_u8<<(2))&(0x04)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x32), &data_u8, ((u8)1));



		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}
















 
s8 bma2x2_get_nvmprog_mode(
u8 *nvmprog_mode_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		com_rslt = ((s8)-127);
	} else {
		 
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



		*nvmprog_mode_u8 = ((data_u8 & (0x01)) >> (0));

	}
	return com_rslt;
}
















 
s8 bma2x2_set_nvmprog_mode(u8 nvmprog_mode_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		com_rslt = ((s8)-127);
	} else {
		 
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



		data_u8 = ((data_u8 & ~(0x01)) | ((nvmprog_mode_u8<<(0))&(0x01)));

		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



	}
	return com_rslt;
}



















 
s8 bma2x2_set_nvprog_trig(u8 nvprog_trig_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		com_rslt = ((s8)-127);
	} else {
		 
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



		data_u8 = ((data_u8 & ~(0x02)) | ((nvprog_trig_u8<<(1))&(0x02)));

		com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



	}
	return com_rslt;
}


















 
s8 bma2x2_get_nvmprog_ready(u8 *nvprog_ready_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		com_rslt = ((s8)-127);
	} else {
		 
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



		*nvprog_ready_u8 = ((data_u8 & (0x04)) >> (2));

	}
	return com_rslt;
}


















 
s8 bma2x2_get_nvmprog_remain(u8 *nvprog_remain_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);
	 
	if (((void *)0) == p_bma2x2) {
		com_rslt = ((s8)-127);
	} else {
		 
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x33), &data_u8, ((u8)1));



		*nvprog_remain_u8 = ((data_u8 & (0xF0)) >> (4));

	}
	return com_rslt;
}



















 
s8 bma2x2_get_spi3(u8 *spi3_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



			*spi3_u8 = ((data_u8 & (0x01)) >> (0));

		}
	return com_rslt;
}



















 
s8 bma2x2_set_spi3(u8 spi3_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((spi3_u8<<(0))&(0x01)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



		}
	return com_rslt;
}






























 
s8 bma2x2_get_i2c_wdt(u8 channel_u8,
u8 *i2c_wdt_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



			*i2c_wdt_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



			*i2c_wdt_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		default:
		com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}






























 
s8 bma2x2_set_i2c_wdt(u8 channel_u8,
u8 i2c_wdt_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((i2c_wdt_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((i2c_wdt_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x34), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_get_slow_comp(u8 channel_u8,
u8 *slow_comp_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			*slow_comp_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			*slow_comp_u8 = ((data_u8 & (0x02)) >> (1));

		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			*slow_comp_u8 = ((data_u8 & (0x04)) >> (2));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




























 
s8 bma2x2_set_slow_comp(u8 channel_u8,
u8 slow_comp_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((slow_comp_u8<<(0))&(0x01)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x02)) | ((slow_comp_u8<<(1))&(0x02)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x04)) | ((slow_comp_u8<<(2))&(0x04)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}

















 
s8 bma2x2_get_cal_rdy(u8 *cal_rdy_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



		*cal_rdy_u8 = ((data_u8 & (0x10)) >> (4));

	}
	return com_rslt;
}

















 
s8 bma2x2_set_cal_trigger(u8 cal_trigger_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x60)) | ((cal_trigger_u8<<(5))&(0x60)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



		}
	return com_rslt;
}
















 
s8 bma2x2_set_offset_rst(u8 offset_rst_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x80)) | ((offset_rst_u8<<(7))&(0x80)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x36), &data_u8, ((u8)1));



		}
	return com_rslt;
}





































 
s8 bma2x2_get_offset_target(u8 channel_u8,
u8 *offset_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			*offset_u8 = ((data_u8 & (0x01)) >> (0));

		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			*offset_u8 = ((data_u8 & (0x06)) >> (1));

		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			*offset_u8 = ((data_u8 & (0x18)) >> (3));

		break;
		case (3):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			*offset_u8 = ((data_u8 & (0x60)) >> (5));

		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}





































 
s8 bma2x2_set_offset_target(u8 channel_u8,
u8 offset_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x01)) | ((offset_u8<<(0))&(0x01)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



		break;
		case (1):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x06)) | ((offset_u8<<(1))&(0x06)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



		break;
		case (2):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x18)) | ((offset_u8<<(3))&(0x18)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



		break;
		case (3):
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x60)) | ((offset_u8<<(5))&(0x60)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x37), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}
























 
s8 bma2x2_get_offset(u8 channel_u8,
s8 *offset_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x38), &data_u8, ((u8)1));



			*offset_u8 = (s8)data_u8;
		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x39), &data_u8, ((u8)1));



			*offset_u8 = (s8)data_u8;
		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x3A), &data_u8, ((u8)1));



			*offset_u8 = (s8)data_u8;
		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}
























 
s8 bma2x2_set_offset(u8 channel_u8,
s8 offset_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (channel_u8) {
		case (0):
			data_u8 = offset_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x38), &data_u8, ((u8)1));



		break;
		case (1):
			data_u8 = offset_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x39), &data_u8, ((u8)1));



		break;
		case (2):
			data_u8 = offset_u8;
			com_rslt = p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x3A), &data_u8, ((u8)1));



		break;
		default:
			com_rslt = ((s8)-2);
		break;
		}
	}
	return com_rslt;
}




















 
s8 bma2x2_get_fifo_mode(u8 *fifo_mode_u8)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x3E), &data_u8, ((u8)1));



			*fifo_mode_u8 = ((data_u8 & (0xC0)) >> (6));

		}
	return com_rslt;
}




















 
s8 bma2x2_set_fifo_mode(u8 fifo_mode_u8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);
	u8 config_data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (fifo_mode_u8 < ((u8)4)) {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x3E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0xC0)) | ((fifo_mode_u8<<(6))&(0xC0)));

			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x3E), &data_u8, ((u8)1));



			if (com_rslt == ((u8)0)) {
				com_rslt += bma2x2_read_reg(
				(0x3E),
				&config_data_u8,
				((u8)1));
				p_bma2x2->fifo_config = config_data_u8;
			}
		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}




















 
s8 bma2x2_get_fifo_data_select(
u8 *fifo_data_select_u8)
{
		
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x3E), &data_u8, ((u8)1));



			*fifo_data_select_u8 = ((data_u8 & (0x03)) >> (0));

		}
	return com_rslt;
}




















 
s8 bma2x2_set_fifo_data_select(
u8 fifo_data_select_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);
	u8 config_data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		if (fifo_data_select_u8 < ((u8)4)) {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x3E), &data_u8, ((u8)1));



			data_u8 = ((data_u8 & ~(0x03)) | ((fifo_data_select_u8<<(0))&(0x03)));


			com_rslt += p_bma2x2->bus_write(p_bma2x2->dev_addr, (0x3E), &data_u8, ((u8)1));



			if (com_rslt == ((u8)0)) {
				com_rslt += bma2x2_read_reg(
				(0x3E),
				 &config_data_u8,
				 ((u8)1));
				p_bma2x2->fifo_config = config_data_u8;
			}
		} else {
		com_rslt = ((s8)-2);
		}
	}
	return com_rslt;
}














 
s8 bma2x2_get_fifo_data_output_reg(
u8 *output_reg_u8)
{
	u8 data_u8 = ((u8)0);
	
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			 
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x3F), &data_u8, ((u8)1));



			*output_reg_u8 = data_u8;
		}
	return com_rslt;
}















 
s8 bma2x2_read_temp(s8 *temp_s8)
{
	u8 data_u8 = ((u8)0);
		
 
	s8 com_rslt = ((s8)-1);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x08), &data_u8, ((u8)1));



			*temp_s8 = (s8)data_u8;
		}
	return com_rslt;
}














 
s8 bma2x2_read_accel_xyzt(
struct bma2x2_accel_data_temp *accel)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8 data_u8[(7)] = {
	((u8)0), ((u8)0),
	((u8)0), ((u8)0),
	((u8)0), ((u8)0),
	((u8)0)};
	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
		switch (V_BMA2x2RESOLUTION_U8) {
		case (0):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)7));



			 
			accel->x = (s16)((((s32)((s8)
			data_u8[(1)]))
			<< ((u8)8))|
			(data_u8[(0)]
			& (0xF0)));
			accel->x = accel->x >> ((u8)4);

			 
			accel->y = (s16)((((s32)((s8)
			data_u8[(3)]))
			<< ((u8)8))|
			(data_u8[(2)]
			& (0xF0)));
			accel->y = accel->y >> ((u8)4);

			 
			accel->z = (s16)((((s32)((s8)
			data_u8[(5)]))
			<< ((u8)8))|
			(data_u8[(4)]
			& (0xF0)));
			accel->z = accel->z >> ((u8)4);
			 
			accel->temp = (s8)data_u8[(6)];
		break;
		case (1):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)7));



			 
			accel->x = (s16)((((s32)((s8)
			data_u8[(1)]))<<
			((u8)8))|
			(data_u8[(0)]
			& (0xC0)));
			accel->x = accel->x >> ((u8)6);

			 
			accel->y = (s16)((((s32)((s8)
			data_u8[(3)]))<<
			((u8)8))|
			(data_u8[(2)]
			& (0xC0)));
			accel->y = accel->y >> ((u8)6);

			 
			accel->z = (s16)((((s32)((s8)
			data_u8[(5)]))<<
			((u8)8))|
			(data_u8[(4)]
			& (0xC0)));
			accel->z = accel->z >> ((u8)6);

			 
			 
			accel->temp = (s8)data_u8[(6)];
		break;
		case (2):
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x02), data_u8, ((u8)7));



			 
			accel->x = (s16)((((s32)((s8)
			data_u8[(1)]))<<
			((u8)8))|
			(data_u8[(0)]
			& (0xFC)));
			accel->x = accel->x >> ((u8)2);

			 
			accel->y = (s16)((((s32)((s8)
			data_u8[(3)]))<<
			((u8)8))|
			(data_u8[(2)]
			& (0xFC)));
			accel->y = accel->y >> ((u8)2);

			 
			accel->z = (s16)((((s32)((s8)
			data_u8[(5)]))<<
			((u8)8))|
			(data_u8[(4)]
			& (0xFC)));
			accel->z = accel->z >> ((u8)2);
			 
			 
			accel->temp = (s8)data_u8[(6)];
		break;
		default:
		break;
		}
	}
	return com_rslt;
}














 
s8 bma2x2_read_accel_eight_resolution_xyzt(
struct bma2x2_accel_eight_resolution_temp *accel)
{
	
 
	s8 com_rslt = ((s8)-1);
	u8	data_u8 = ((u8)0);

	if (p_bma2x2 == ((void *)0)) {
		 
		return ((s8)-127);
		} else {
			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x03), &data_u8, ((u8)1));



			accel->x = ((data_u8 & (0xFF)) >> (0));


			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x05), &data_u8, ((u8)1));



			accel->y = ((data_u8 & (0xFF)) >> (0));


			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x07), &data_u8, ((u8)1));



			accel->z = ((data_u8 & (0xFF)) >> (0));


			com_rslt = p_bma2x2->bus_read(p_bma2x2->dev_addr, (0x08), &data_u8, ((u8)1));


			accel->temp = (s8)data_u8;
		}
	return com_rslt;
}
