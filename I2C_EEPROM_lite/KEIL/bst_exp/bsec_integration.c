/*
 * Copyright (C) 2017 Robert Bosch. All Rights Reserved. 
 *
 * Disclaimer
 *
 * Common:
 * Bosch Sensortec products are developed for the consumer goods industry. They may only be used
 * within the parameters of the respective valid product data sheet.  Bosch Sensortec products are
 * provided with the express understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive systems or any system or device
 * that may lead to bodily harm or property damage if the system or device malfunctions. In addition,
 * Bosch Sensortec products are not fit for use in products which interact with motor vehicle systems.
 * The resale and/or use of products are at the purchasers own risk and his own responsibility. The
 * examination of fitness for the intended use is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims, including any claims for
 * incidental, or consequential damages, arising from any product use not covered by the parameters of
 * the respective valid product data sheet or not approved by Bosch Sensortec and reimburse Bosch
 * Sensortec for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products, particularly with regard to
 * product safety and inform Bosch Sensortec without delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary from the valid
 * technical specifications of the product series. They are therefore not intended or fit for resale to third
 * parties or for use in end products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series. Bosch Sensortec
 * assumes no liability for the use of engineering samples. By accepting the engineering samples, the
 * Purchaser agrees to indemnify Bosch Sensortec from all claims arising from the use of engineering
 * samples.
 *
 * Special:
 * This software module (hereinafter called "Software") and any information on application-sheets
 * (hereinafter called "Information") is provided free of charge for the sole purpose to support your
 * application work. The Software and Information is subject to the following terms and conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch Sensortec products by
 * personnel who have special experience and training. Do not use this Software if you do not have the
 * proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or implied warranties,
 * including without limitation, the implied warranties of merchantability and fitness for a particular
 * purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for the functional impairment
 * of this Software in terms of fitness, performance and safety. Bosch Sensortec and their
 * representatives and agents shall not be liable for any direct or indirect damages or injury, except as
 * otherwise stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch Sensortec assumes no
 * responsibility for the consequences of use of such Information nor for any infringement of patents or
 * other rights of third parties which may result from its use. No license is granted by implication or
 * otherwise under any patent or patent rights of Bosch. Specifications mentioned in the Information are
 * subject to change without notice.
 *
 * It is not allowed to deliver the source code of the Software to any third party without permission of
 * Bosch Sensortec.
 *
 */

/*!
 * @file bsec_integration.c
 *
 * @brief
 * Private part of the example for using of BSEC library.
 */

/*!
 * @addtogroup bsec_examples BSEC Examples
 * @brief BSEC usage examples
 * @{*/

/**********************************************************************************************************************/
/* header files */
/**********************************************************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>

#include "bsec_integration.h"
#include "Nano103.h"

/**********************************************************************************************************************/
/* local macro definitions */
/**********************************************************************************************************************/

#define NUM_USED_OUTPUTS 7

/**********************************************************************************************************************/
/* global variable declarations */
/**********************************************************************************************************************/

/*3.3V, 3sec, 4days*/
//uint8_t bsec_config_iaq[304] = {0,6,4,1,61,0,0,0,0,0,0,0,24,1,0,0,40,0,1,0,137,65,0,63,0,0,64,63,205,204,76,62,0,0,225,68,0,192,168,71,0,0,0,0,0,80,10,90,0,0,0,0,0,0,0,0,21,0,2,0,0,244,1,225,0,25,10,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,9,0,5,0,0,0,0,0,1,51,0,9,0,10,215,163,59,205,204,204,61,225,122,148,62,41,92,15,61,0,0,0,63,0,0,0,63,154,153,89,63,154,153,25,62,1,1,0,0,128,63,6,236,81,184,61,51,51,131,64,12,0,10,0,0,0,0,0,0,0,0,0,131,0,254,0,2,1,5,48,117,100,0,44,1,151,7,132,3,197,0,144,1,64,1,64,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,98,149,0,0};
//300sec
uint8_t bsec_config_iaq[304] = {0,6,4,1,61,0,0,0,0,0,0,0,24,1,0,0,40,0,1,0,137,65,0,63,0,0,64,63,205,204,76,62,0,0,225,68,0,192,168,71,0,0,0,0,0,80,10,90,0,0,0,0,0,0,0,0,21,0,2,0,0,244,1,225,0,25,10,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,9,0,5,0,0,0,0,0,1,51,0,9,0,10,215,163,59,205,204,204,61,225,122,148,62,41,92,15,61,0,0,0,63,0,0,0,63,154,153,89,63,154,153,25,62,1,1,0,0,128,63,6,236,81,184,61,51,51,131,64,12,0,10,0,0,0,0,0,0,0,0,0,131,0,254,0,2,1,5,48,117,100,0,44,1,151,7,132,3,197,0,144,1,64,1,64,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,48,117,0,0,0,0,240,156,0,0};


/* Global sensor APIs data structure */
static struct bme680_dev bme680_g;

/* Global temperature offset to be subtracted */
static float bme680_temperature_offset_g = 0.0f;

volatile float iaq = 0.0;
volatile uint8_t iaq_accuracy = 0;
volatile float raw_temp = 0.0;
volatile float raw_pressure = 0.0;
volatile float raw_humidity = 0.0;
volatile float raw_gas = 0.0;
volatile float sw_humidity = 0.0;
volatile float sw_temp = 0.0;

volatile uint8_t Temperature_MSB = 0;
volatile uint8_t Temperature_LSB = 0;
volatile uint8_t Temperature_Sign = 0;
volatile uint8_t Pressure_MSB = 0;
volatile uint8_t Pressure_LSB = 0;
volatile uint8_t Humidity_MSB = 0;
volatile uint8_t Humidity_LSB = 0;
volatile uint8_t IAQ_MSB = 0;
volatile uint8_t IAQ_LSB = 0;

volatile uint8_t Acc_X_MSB = 0;
volatile uint8_t Acc_X_LSB = 0;
volatile uint8_t Acc_Y_MSB = 0;
volatile uint8_t Acc_Y_LSB = 0;
volatile uint8_t Acc_Z_MSB = 0;
volatile uint8_t Acc_Z_LSB = 0;
/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/

/*!
 * @brief        Virtual sensor subscription
 *               Please call this function before processing of data using bsec_do_steps function
 *
 * @param[in]    sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 *  
 * @return       subscription result, zero when successful
 */
static bsec_library_return_t bme680_bsec_update_subscription(float sample_rate)
{
    bsec_sensor_configuration_t requested_virtual_sensors[NUM_USED_OUTPUTS];
    uint8_t n_requested_virtual_sensors = NUM_USED_OUTPUTS;
    
    bsec_sensor_configuration_t required_sensor_settings[BSEC_MAX_PHYSICAL_SENSOR];
    uint8_t n_required_sensor_settings = BSEC_MAX_PHYSICAL_SENSOR;
    
    bsec_library_return_t status = BSEC_OK;
 
    /* note: Virtual sensors as desired to be added here */
    requested_virtual_sensors[0].sensor_id = BSEC_OUTPUT_IAQ_ESTIMATE;
    requested_virtual_sensors[0].sample_rate = sample_rate;
    requested_virtual_sensors[1].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE;
    requested_virtual_sensors[1].sample_rate = sample_rate;
    requested_virtual_sensors[2].sensor_id = BSEC_OUTPUT_RAW_PRESSURE;
    requested_virtual_sensors[2].sample_rate = sample_rate;
    requested_virtual_sensors[3].sensor_id = BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY;
    requested_virtual_sensors[3].sample_rate = sample_rate;
    requested_virtual_sensors[4].sensor_id = BSEC_OUTPUT_RAW_GAS;
    requested_virtual_sensors[4].sample_rate = sample_rate;
    requested_virtual_sensors[5].sensor_id = BSEC_OUTPUT_RAW_TEMPERATURE;
    requested_virtual_sensors[5].sample_rate = sample_rate;
    requested_virtual_sensors[6].sensor_id = BSEC_OUTPUT_RAW_HUMIDITY;
    requested_virtual_sensors[6].sample_rate = sample_rate;
    
    /* Call bsec_update_subscription() to enable/disable the requested virtual sensors */
    status = bsec_update_subscription(requested_virtual_sensors, n_requested_virtual_sensors, required_sensor_settings,
        &n_required_sensor_settings);
    
    return status;
}

/*!
 * @brief       Initialize the BME680 sensor and the BSEC library
 *
 * @param[in]   sample_rate         mode to be used (either BSEC_SAMPLE_RATE_ULP or BSEC_SAMPLE_RATE_LP)
 * @param[in]   temperature_offset  device-specific temperature offset (due to self-heating)
 * @param[in]   bus_write           pointer to the bus writing function
 * @param[in]   bus_read            pointer to the bus reading function
 * @param[in]   sleep               pointer to the system specific sleep function
 * @param[in]   state_load          pointer to the system-specific state load function
 * @param[in]   config_load         pointer to the system-specific config load function
 *
 * @return      zero if successful, negative otherwise
 */
return_values_init bsec_iot_init(float sample_rate, float temperature_offset, bme680_com_fptr_t bus_write, 
                    bme680_com_fptr_t bus_read, sleep_fct sleep, state_load_fct state_load, config_load_fct config_load)
{
    return_values_init ret = {BME680_OK, BSEC_OK};
    
    //uint8_t bsec_state[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    //uint8_t bsec_config[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    //uint8_t work_buffer[BSEC_MAX_PROPERTY_BLOB_SIZE] = {0};
    int bsec_state_len, bsec_config_len;
    bsec_version_t ver;
		
    /* Fixed I2C configuration */
    bme680_g.dev_id = BME680_I2C_ADDR_PRIMARY;
    bme680_g.intf = BME680_I2C_INTF;
    /* User configurable I2C configuration */
    bme680_g.write = bus_write;
    bme680_g.read = bus_read;
    bme680_g.delay_ms = sleep;
	
    /* Initialize BME680 API */
    ret.bme680_status = bme680_init(&bme680_g);
		if (ret.bme680_status != BME680_OK)
		{
			return ret;
		}

    /* Initialize BSEC library */
    ret.bsec_status = bsec_init();
    if (ret.bsec_status != BSEC_OK)
    {
        return ret;
    }
		
		bsec_get_version(&ver);
		//printf("major=%d, minor=%d, major_bugfix=%d, minor_bugfix=%d\n",ver.major,ver.minor, ver.major_bugfix, ver.minor_bugfix);
 
		#if 0   
    /* Load library config, if available */
    bsec_config_len = config_load(bsec_config, sizeof(bsec_config));
    if (bsec_config_len > 0)
    {       
        //ret.bsec_status = bsec_set_configuration(bsec_config, bsec_config_len, work_buffer, sizeof(work_buffer));     
        ret.bsec_status = bsec_set_configuration(bsec_config_iaq, 304, work_buffer, sizeof(work_buffer));
			  printf(" set config ret.bsec_status=%d\n",ret.bsec_status);
			
			  if (ret.bsec_status != BSEC_OK)
        {
            return ret;
        }
    }
   
    /* Load previous library state, if available */
    bsec_state_len = state_load(bsec_state, sizeof(bsec_state));
		//printf(" state_load bsec_state_len=%d\n",bsec_state_len);
    if (bsec_state_len > 0)
    {       
        ret.bsec_status = bsec_set_state(bsec_state, bsec_state_len, work_buffer, sizeof(work_buffer)); 
				//printf(" state_load ret.bsec_status=%d\n",bsec_state_len);
        if (ret.bsec_status != BSEC_OK)
        {
            return ret;
        }
    }
#endif    
		
    /* Set temperature offset */
    bme680_temperature_offset_g = temperature_offset;

    /* Call to the function which sets the library with subscription information */
    ret.bsec_status = bme680_bsec_update_subscription(sample_rate);
		//printf(" update_subscription ret.bsec_status=%d\n",ret.bsec_status);
    if (ret.bsec_status != BSEC_OK)
    {
        return ret;
    }
    
    return ret;
}

/*!
 * @brief       Trigger the measurement based on sensor settings
 *
 * @param[in]   sensor_settings     settings of the BME680 sensor adopted by sensor control function
 * @param[in]   sleep               pointer to the system specific sleep function
 *
 * @return      none
 */
static void bme680_bsec_trigger_measurement(bsec_bme_settings_t *sensor_settings, sleep_fct sleep)
{
	uint16_t meas_period;
	uint8_t set_required_settings;
  int8_t bme680_status = BME680_OK;
        
    /* Check if a forced-mode measurement should be triggered now */
    if (sensor_settings->trigger_measurement)
    {
        /* Set sensor configuration */

        bme680_g.tph_sett.os_hum  = sensor_settings->humidity_oversampling;
        bme680_g.tph_sett.os_pres = sensor_settings->pressure_oversampling;
        bme680_g.tph_sett.os_temp = sensor_settings->temperature_oversampling;
				bme680_g.gas_sett.run_gas = sensor_settings->run_gas;
				bme680_g.gas_sett.heatr_temp = sensor_settings->heater_temperature; /* degree Celsius */
				bme680_g.gas_sett.heatr_dur  = sensor_settings->heating_duration; /* milliseconds */
		
				/* Select the power mode */
				/* Must be set before writing the sensor configuration */
				bme680_g.power_mode = BME680_FORCED_MODE;
				/* Set the required sensor settings needed */
				set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_GAS_SENSOR_SEL;
		
				/* Set the desired sensor configuration */
        bme680_status = bme680_set_sensor_settings(set_required_settings, &bme680_g);
             
        /* Set power mode as forced mode and trigger forced mode measurement */
        bme680_status = bme680_set_sensor_mode(&bme680_g);
        
				/* Get the total measurement duration so as to sleep or wait till the measurement is complete */
				bme680_get_profile_dur(&meas_period, &bme680_g);
        
        /* Delay till the measurement is ready. Timestamp resolution in ms */
        sleep((uint32_t)meas_period);
    }
    
    /* Call the API to get current operation mode of the sensor */
    bme680_status = bme680_get_sensor_mode(&bme680_g);  
    /* When the measurement is completed and data is ready for reading, the sensor must be in BME680_SLEEP_MODE.
     * Read operation mode to check whether measurement is completely done and wait until the sensor is no more
     * in BME680_FORCED_MODE. */
    while (bme680_g.power_mode == BME680_FORCED_MODE)
    {
        /* sleep for 5 ms */
        sleep(5);
        bme680_status = bme680_get_sensor_mode(&bme680_g);
    }
}

/*!
 * @brief       Read the data from registers and populate the inputs structure to be passed to do_steps function
 *
 * @param[in]   time_stamp_trigger      settings of the sensor returned from sensor control function
 * @param[in]   inputs                  input structure containing the information on sensors to be passed to do_steps
 * @param[in]   num_bsec_inputs         number of inputs to be passed to do_steps
 * @param[in]   bsec_process_data       process data variable returned from sensor_control
 *
 * @return      none
 */
static void bme680_bsec_read_data(int64_t time_stamp_trigger, bsec_input_t *inputs, uint8_t *num_bsec_inputs,
    int32_t bsec_process_data)
{
    static struct bme680_field_data data;
    int8_t bme680_status = BME680_OK;
    
    /* We only have to read data if the previous call the bsec_sensor_control() actually asked for it */
    if (bsec_process_data)
    {
        bme680_status = bme680_get_sensor_data(&data, &bme680_g);

        if (data.status & BME680_NEW_DATA_MSK)
        {
            /* Pressure to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_PRESSURE)
            {
                /* Place presssure sample into input struct */
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_PRESSURE;
                inputs[*num_bsec_inputs].signal = data.pressure;
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
            }
            /* Temperature to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_TEMPERATURE)
            {
                /* Place temperature sample into input struct */
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_TEMPERATURE;
                inputs[*num_bsec_inputs].signal = data.temperature / 100.0f;
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
                
                /* Also add optional heatsource input which will be subtracted from the temperature reading to 
                 * compensate for device-specific self-heating (supported in BSEC IAQ solution)*/
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_HEATSOURCE;
                inputs[*num_bsec_inputs].signal = bme680_temperature_offset_g;
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
            }
            /* Humidity to be processed by BSEC */
            if (bsec_process_data & BSEC_PROCESS_HUMIDITY)
            {
                /* Place humidity sample into input struct */
                inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_HUMIDITY;
                inputs[*num_bsec_inputs].signal = data.humidity / 1000.0f;
                inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                (*num_bsec_inputs)++;
            }
            /* Gas to be processed by BSEC */
						//printf("bsec_process_data = %X, BSEC_PROCESS_GAS=%X, data.status =%X, BME680_GASM_VALID_MSK = %X \n", bsec_process_data, BSEC_PROCESS_GAS,data.status,BME680_GASM_VALID_MSK);
            if (bsec_process_data & BSEC_PROCESS_GAS)
            {
                /* Check whether gas_valid flag is set */
                if(data.status & BME680_GASM_VALID_MSK)
                {
                    /* Place sample into input struct */
                    inputs[*num_bsec_inputs].sensor_id = BSEC_INPUT_GASRESISTOR;
									  //printf("data.gas_resistance = %ld\n", data.gas_resistance);
                    inputs[*num_bsec_inputs].signal = data.gas_resistance;
                    inputs[*num_bsec_inputs].time_stamp = time_stamp_trigger;
                    (*num_bsec_inputs)++;
                }
            }
        }
    }
}

/*!
 * @brief       This function is written to process the sensor data for the requested virtual sensors
 *
 * @param[in]   bsec_inputs         input structure containing the information on sensors to be passed to do_steps
 * @param[in]   num_bsec_inputs     number of inputs to be passed to do_steps
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 *
 * @return      none
 */
static void bme680_bsec_process_data(bsec_input_t *bsec_inputs, uint8_t num_bsec_inputs, output_ready_fct output_ready)
{
    /* Output buffer set to the maximum virtual sensor outputs supported */
    bsec_output_t bsec_outputs[BSEC_NUMBER_OUTPUTS];
    uint8_t num_bsec_outputs = 0;
    uint8_t index = 0;

    bsec_library_return_t bsec_status = BSEC_OK;
    
    int64_t timestamp = 0;
    float iaq = 0.0f;
    uint8_t iaq_accuracy = 0;
    float temp = 0.0f;
    float raw_temp = 0.0f;
    float raw_pressure = 0.0f;
    float humidity = 0.0f;
    float raw_humidity = 0.0f;
    float raw_gas = 0.0f;
	
    uint32_t full_pressure;
		uint32_t full_temp;
    uint32_t full_humidity;
		uint32_t full_iaq;
		
    /* Check if something should be processed by BSEC */
    if (num_bsec_inputs > 0)
    {
        /* Set number of outputs to the size of the allocated buffer */
        /* BSEC_NUMBER_OUTPUTS to be defined */
        num_bsec_outputs = BSEC_NUMBER_OUTPUTS;
        
        /* Perform processing of the data by BSEC 
           Note:
           * The number of outputs you get depends on what you asked for during bsec_update_subscription(). This is
             handled under bme680_bsec_update_subscription() function in this example file.
           * The number of actual outputs that are returned is written to num_bsec_outputs. */
        bsec_status = bsec_do_steps(bsec_inputs, num_bsec_inputs, bsec_outputs, &num_bsec_outputs);
        //printf("bsec_do_steps status=%d\n",bsec_status);
			
        /* Iterate through the outputs and extract the relevant ones. */
        for (index = 0; index < num_bsec_outputs; index++)
        {
            switch (bsec_outputs[index].sensor_id)
            {
                case BSEC_OUTPUT_IAQ_ESTIMATE:
                    iaq = bsec_outputs[index].signal;
                    iaq_accuracy = bsec_outputs[index].accuracy;
										full_iaq = (uint32_t)iaq;
										IAQ_MSB = (full_iaq/10);
										IAQ_LSB = (full_iaq%10);
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_TEMPERATURE:
                    temp = bsec_outputs[index].signal;
								case BSEC_OUTPUT_RAW_TEMPERATURE:
                    raw_temp = bsec_outputs[index].signal;
										full_temp = (uint32_t)(raw_temp*10);
										Temperature_MSB = (uint8_t)(full_temp/10);
										Temperature_LSB = (uint8_t)(full_temp%10);
										if(raw_temp<0)
										{
											Temperature_LSB = 0x10 | Temperature_LSB;
										}
										//printf("Temp_MSB=%d, Temp_LSB=%d\n",Temperature_MSB, Temperature_LSB);
                    break;
                case BSEC_OUTPUT_RAW_PRESSURE:
                    raw_pressure = bsec_outputs[index].signal;
										full_pressure = (uint32_t)(raw_pressure/10);
										Pressure_MSB = (full_pressure/100);
										Pressure_LSB = (full_pressure%100);
										//printf("Pressure_MSB=%d, Pressure_LSB=%d\n",Pressure_MSB, Pressure_LSB); 
                    break;
                case BSEC_OUTPUT_SENSOR_HEAT_COMPENSATED_HUMIDITY:
                    humidity = bsec_outputs[index].signal;
                case BSEC_OUTPUT_RAW_HUMIDITY:
                    raw_humidity = bsec_outputs[index].signal;
										full_humidity = (uint32_t)(raw_humidity*100);
										Humidity_MSB = (full_humidity/100);
										Humidity_LSB = (full_humidity%100);
										//printf("Humidity_MSB=%d, Humidity_LSB=%d\n",Humidity_MSB, Humidity_LSB); 
                    break;
                case BSEC_OUTPUT_RAW_GAS:
                    raw_gas = bsec_outputs[index].signal;
                    break;

                default:
                    continue;
            }
            
            /* Assume that all the returned timestamps are the same */
            timestamp = bsec_outputs[index].time_stamp;
        }
        
        /* Pass the extracted outputs to the user provided output_ready() function. */
        output_ready(timestamp, iaq, iaq_accuracy, temp, humidity, raw_pressure, raw_temp, 
            raw_humidity, raw_gas, bsec_status);
    }
}

/*!
 * @brief       Runs the main (endless) loop that queries sensor settings, applies them, and processes the measured data
 *
 * @param[in]   sleep               pointer to the system specific sleep function
 * @param[in]   get_timestamp_us    pointer to the system specific timestamp derivation function
 * @param[in]   output_ready        pointer to the function processing obtained BSEC outputs
 * @param[in]   state_save          pointer to the system-specific state save function
 * @param[in]   save_intvl          interval at which BSEC state should be saved (in samples)
 *
 * @return      none
 */
void bsec_iot_loop(sleep_fct sleep, get_timestamp_us_fct get_timestamp_us, output_ready_fct output_ready,
                    state_save_fct state_save, uint32_t save_intvl)
{
    /* Timestamp variables */
    int64_t time_stamp = 0;
	  int64_t time_test = 0;
    int64_t time_stamp_interval_ms = 0;
    uint8_t acc_temp_msb = 0; 
	  uint8_t acc_temp_lsb = 0;
	  uint32_t ts;
	
    /* Allocate enough memory for up to BSEC_MAX_PHYSICAL_SENSOR physical inputs*/
    bsec_input_t bsec_inputs[BSEC_MAX_PHYSICAL_SENSOR];
    
    /* Number of inputs to BSEC */
    uint8_t num_bsec_inputs = 0;
    
    /* BSEC sensor settings struct */
    bsec_bme_settings_t sensor_settings;
    
    /* Save state variables */
    uint8_t bsec_state[BSEC_MAX_STATE_BLOB_SIZE];
    uint8_t work_buffer[BSEC_MAX_STATE_BLOB_SIZE];
    uint32_t bsec_state_len = 0;
    uint32_t n_samples = 0;
    
    bsec_library_return_t bsec_status = BSEC_OK;

    while (1)
    {
        /* get the timestamp in nanoseconds before calling bsec_sensor_control() */
        time_stamp = get_timestamp_us() * 1000;
			  ts = (uint32_t)(time_stamp/1000000L);
        //printf("bsec_iot_loop time_stamp = %lld, ts=%d \n",time_stamp,ts);
			
        /* Retrieve sensor settings to be used in this time instant by calling bsec_sensor_control */
        bsec_status = bsec_sensor_control(time_stamp, &sensor_settings);
        //printf("bsec_sensor_control bsec_status = %d \n",bsec_status);
			
        /* Trigger a measurement if necessary */
        bme680_bsec_trigger_measurement(&sensor_settings, sleep);
        
        /* Read data from last measurement */
        num_bsec_inputs = 0;
        bme680_bsec_read_data(time_stamp, bsec_inputs, &num_bsec_inputs, sensor_settings.process_data);
        
        /* Time to invoke BSEC to perform the actual processing */
        bme680_bsec_process_data(bsec_inputs, num_bsec_inputs, output_ready);
        
        /* Increment sample counter */
        n_samples++;
        
        /* Retrieve and store state if the passed save_intvl */
        if (n_samples >= save_intvl)
        {
            //bsec_status = bsec_get_state(0, bsec_state, sizeof(bsec_state), work_buffer, sizeof(work_buffer), &bsec_state_len);
            //if (bsec_status == BSEC_OK)
            //{
            //    state_save(bsec_state, bsec_state_len);
            //}
            n_samples = 0;
        }
				
        /* Read accel sample code*/
			  I2C_MasterReadDataFromSlave(0x18, BMA2x2_X_AXIS_LSB_ADDR, &acc_temp_lsb, 1);
			  //printf("acc x LSB: %x\n", acc_temp_lsb);
			  Acc_X_LSB = acc_temp_lsb;
				I2C_MasterReadDataFromSlave(0x18, BMA2x2_X_AXIS_MSB_ADDR, &acc_temp_msb, 1);
			  //printf("acc x MSB: %x\n", acc_temp_msb);
				Acc_X_MSB = acc_temp_msb;
				
        I2C_MasterReadDataFromSlave(0x18, BMA2x2_Y_AXIS_LSB_ADDR, &acc_temp_lsb, 1);
			  //printf("acc y LSB: %x\n", acc_temp_lsb);
				Acc_Y_LSB = acc_temp_lsb;
			  I2C_MasterReadDataFromSlave(0x18, BMA2x2_Y_AXIS_MSB_ADDR, &acc_temp_msb, 1);
			  //printf("acc y MSB: %x\n", acc_temp_msb);
				Acc_Y_MSB = acc_temp_msb;
				
				I2C_MasterReadDataFromSlave(0x18, BMA2x2_Z_AXIS_LSB_ADDR, &acc_temp_lsb, 1);
			  //printf("acc z LSB: %x\n", acc_temp_lsb);
				Acc_Z_LSB = acc_temp_lsb;
			  I2C_MasterReadDataFromSlave(0x18, BMA2x2_Z_AXIS_MSB_ADDR, &acc_temp_msb, 1);
			  //printf("acc z MSB: %x\n", acc_temp_msb);
				Acc_Z_MSB = acc_temp_msb;
				
        /* Compute how long we can sleep until we need to call bsec_sensor_control() next */
        /* Time_stamp is converted from microseconds to nanoseconds first and then the difference to milliseconds */
        time_stamp_interval_ms = (sensor_settings.next_call - get_timestamp_us() * 1000) / 1000000;
        //printf("time_stamp_interval_ms = %lld \n",time_stamp_interval_ms);
				time_test = get_timestamp_us();
				//printf("before sleep time_test = %lld\n",time_test);
				if (time_stamp_interval_ms > 0)
        {
            sleep((uint32_t)time_stamp_interval_ms);
        }
				time_test = get_timestamp_us();
				//printf("after sleep time_test = %lld\n",time_test);
    }
}







