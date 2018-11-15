#include "bsec_datatypes.h"
#include "Nano103.h"
#include "bme680.h"
#include "bma2x2.h"
#define HCLK_FREQ 32000000	

#define DATA_FLASH_BASE	0xFE00	/* Data Flash start address */
#define BSEC_STATE_LEN 400			/* 400 byte*/
#define DATA_FLASH_END (DATA_FLASH_BASE + BSEC_STATE_LEN)

/**********************************************************************************************************************/
/* type definitions */
/**********************************************************************************************************************/

/* function pointer to the system specific sleep function */
typedef void (*sleep_fct)(uint32_t t_ms);

/* function pointer to the system specific timestamp derivation function */
typedef int64_t (*get_timestamp_us_fct)();

/* function pointer to the function processing obtained BSEC outputs */
/*typedef void (*output_ready_fct)(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float pressure,
    float humidity, float gas, bsec_library_return_t bsec_status);
*/

#define portTICK_RATE_US 1 //((double)1000000/(double)HCLK_FREQ) // Tick interval in us

#define REG_CHIP_ID 0x0
#define REG_TEMP_MSB 0x1
#define REG_TEMP_LSB 0x2
#define REG_PRESSURE_MSB 0x3
#define REG_PRESSURE_LSB 0x4
#define REG_HUMIDITY_MSB 0x5
#define REG_HUMIDITY_LSB 0x6
#define REG_IAQ_MSB 0x7
#define REG_IAQ_LSB 0x8
#define REG_IAQ_ACCURACY 0x9
#define REG_ACC_X_MSB 0xA
#define REG_ACC_X_LSB 0xB
#define REG_ACC_Y_MSB 0xC
#define REG_ACC_Y_LSB 0xD
#define REG_ACC_Z_MSB 0xE
#define REG_ACC_Z_LSB 0xF

int8_t I2C_MasterWriteDataToSlave(uint8_t slave_addr, uint8_t regr_addr, uint8_t *write_data, uint16_t data_len);
int8_t I2C_MasterReadDataFromSlave(uint8_t slave_addr, uint8_t regr_addr, uint8_t *reg_data_ptr, uint16_t data_len);
void I2C0_Init(void);
void I2C1_Init(void);
int64_t Get_stamp_us(void);
void delay_ms(uint32_t d_ms);
void I2C_SlaveTRx(uint32_t u32Status);
void I2C_MasterTx(uint32_t u32Status);
void I2C_MasterRx(uint32_t u32Status);
uint8_t I2C_HandleSlaveReadCmd(uint8_t RegAddr, uint8_t* ReturnData);
uint32_t Load_State_from_Flash(uint8_t *state_buffer, uint32_t n_buffer);
uint32_t Load_Config(uint8_t *state_buffer, uint32_t n_buffer);
void Save_State_to_Flash(const uint8_t *state_buffer, uint32_t length);
int set_data_flash_base(uint32_t u32DFBA);
void Output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
    float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status);
