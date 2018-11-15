#include <stdio.h>
#include "Nano103.h"
#include "Nano_support.h"
#include "bsec_integration.h"

#define SLAVE_ADDRESS 0x33

/*3.3V, 3sec, 4days*/
//uint8_t bsec_config_iaq[304] = {0,6,4,1,61,0,0,0,0,0,0,0,24,1,0,0,40,0,1,0,137,65,0,63,0,0,64,63,205,204,76,62,0,0,225,68,0,192,168,71,0,0,0,0,0,80,10,90,0,0,0,0,0,0,0,0,21,0,2,0,0,244,1,225,0,25,10,144,1,0,0,112,65,0,0,0,63,16,0,3,0,10,215,163,60,10,215,35,59,10,215,35,59,9,0,5,0,0,0,0,0,1,51,0,9,0,10,215,163,59,205,204,204,61,225,122,148,62,41,92,15,61,0,0,0,63,0,0,0,63,154,153,89,63,154,153,25,62,1,1,0,0,128,63,6,236,81,184,61,51,51,131,64,12,0,10,0,0,0,0,0,0,0,0,0,131,0,254,0,2,1,5,48,117,100,0,44,1,151,7,132,3,197,0,144,1,64,1,64,1,48,117,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,48,117,100,0,100,0,100,0,100,0,48,117,48,117,48,117,100,0,100,0,100,0,48,117,48,117,100,0,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,44,1,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,255,44,1,0,0,0,0,98,149,0,0};

/* I2C Master variables*/
uint8_t g_u8DeviceAddr;     // slave address
uint8_t g_u8SlaveRegrAddr;  // register addr at slave
uint8_t *g_u8TxData;         // master transmit buffer, only write 1 byte
uint8_t *g_u8RxData;        // master receive buffer, can support multiple read
volatile uint8_t g_u8DataLen;           // Number of data bytes to be written to slave or to read from slave
volatile uint8_t g_u8TxDataCounter = 0; // Count how many bytes have been transmitted from Master to Slave
volatile uint8_t g_u8RxDataCounter = 0; // Count how many bytes have been transmitted from Slave to Master
volatile uint8_t g_u8EndFlag = 0;       // function end flag

/* I2C Slave variables*/
uint32_t g_u32SlaveBuffAddr;        // slave receive buffer for address partuint8_t g_au8MasterTxData[3];       // master transmit buffer
uint8_t g_u8SlvData[256];           // slave receive buffer for data part
uint8_t g_au8SlaveRxData[3];        // slave receive buffer
uint8_t g_u8SlaveDataLen;           // count for slave transmit
uint8_t g_u8SlaveWriteDataLen = 0;  // Data count after SLA+W, 1st byte is register address
uint8_t g_u8SlaveRegisterAddr;      // After SLA+W, Register addr at slave side to be written or read  

/*Timer counter*/
volatile int64_t Global_timestamp_ms = 0;


extern float iaq;
extern uint8_t iaq_accuracy;
extern float raw_temp;
extern float raw_pressure;
extern float raw_humidity;
extern float raw_gas;

extern uint8_t Temperature_MSB;
extern uint8_t Temperature_LSB;
extern uint8_t Temperature_Sign;
extern uint8_t Pressure_MSB;
extern uint8_t Pressure_LSB;
extern uint8_t Humidity_MSB;
extern uint8_t Humidity_LSB;
extern uint8_t IAQ_MSB;
extern uint8_t IAQ_LSB;

extern uint8_t Acc_X_LSB;
extern uint8_t Acc_X_MSB;
extern uint8_t Acc_Y_LSB;
extern uint8_t Acc_Y_MSB;
extern uint8_t Acc_Z_LSB;
extern uint8_t Acc_Z_MSB;

/**********************************************************************************************************************/
/* functions */
/**********************************************************************************************************************/
/* I2C handler */
typedef void (*I2C_FUNC)(uint32_t u32Status);
volatile I2C_FUNC s_I2C1HandlerFn = NULL;


/*---------------------------------------------------------------------------------------------------------*/
/*  TMR0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void TMR0_IRQHandler(void)
{
    Global_timestamp_ms++;      //increment counter @ 1000Hz
    TIMER_ClearIntFlag(TIMER0); // clear timer interrupt flag

}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    /* clear interrupt flag */
    I2C1->INTSTS |= I2C_INTSTS_INTSTS_Msk;

    /* get status */
    u32Status = I2C_GET_STATUS(I2C1);

    /* check if time-out occur */
    if (I2C_GET_TIMEOUT_FLAG(I2C1)) 
		{
			  printf("I2C1_IRQHandler Timeout");
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    } 
		else 
		{
        /* jump to handler */
        if (s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    /* clear interrupt flag */
    I2C0->INTSTS |= I2C_INTSTS_INTSTS_Msk;

    /* get status */
    u32Status = I2C_GET_STATUS(I2C0);

    /* check if time-out occur */
    if (I2C_GET_TIMEOUT_FLAG(I2C0))
		{
			  printf("I2C0_IRQHandler Timeout");
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    } 
		else 
		{
        /* jump to handler */
       // if (s_I2C1HandlerFn != NULL)
       //     s_I2C1HandlerFn(u32Status);
				
			 I2C_SlaveTRx(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
	  //printf("I2C_MasterRx Status 0x%x \n", u32Status);
	
    if(u32Status == 0x08)														/* START has been transmitted and prepare SLA+W */
		{                          
        I2C_SET_DATA(I2C1, (g_u8DeviceAddr << 1));  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_SI);          /* Trigger I2C */
    } 
		else if(u32Status == 0x18)                      /* SLA+W has been transmitted and ACK has been received */
		{                 
        I2C_SET_DATA(I2C1, g_u8SlaveRegrAddr);      /* Write data to I2C buffer, send register addr to slave */
        I2C_SET_CONTROL_REG(I2C1, I2C_SI);          /* Trigger I2C */
    } 
		else if (u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
		{                     
        I2C_SET_CONTROL_REG(I2C1, I2C_STO | I2C_SI);  /* I2C STOP*/
    } 
		else if (u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
		{     
				/* generate Repeat START */
        I2C_SET_CONTROL_REG(I2C1, I2C_STA | I2C_SI);
			
    } 
		else if (u32Status == 0x10)															/* Repeat START has been transmitted and prepare SLA+R */ 
		{                           
        I2C_SET_DATA(I2C1, (g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_SI);                  /* Trigger I2C */
    } 
		else if (u32Status == 0x40)                             /* SLA+R has been transmitted and ACK has been received */
		{     
        if(g_u8DataLen >1)                                  /* read more than 1 byte*/
        {
				  I2C_SET_CONTROL_REG(I2C1, I2C_AA|I2C_SI);	        /* After receiving data from slave, master will ACK*/
				}
				else                                                /* read just one byte*/ 
				{	
          I2C_SET_CONTROL_REG(I2C1, I2C_SI);                /* After receiving data from slave, master will NACK */
			  }
    } 
		else if (u32Status == 0x50)                             /* DATA has been received and ACK has been returned to slave */
		{  
      g_u8RxDataCounter++;                                    /* Received one byte*/ 			
      //printf("g_u8RxDataCounter = %d, g_u8DataLen=%d\n ", g_u8RxDataCounter,g_u8DataLen);
			*g_u8RxData = I2C_GET_DATA(I2C1);   /* Get I2C data */
			//printf("*g_u8RxData = %x\n", *g_u8RxData);		
			g_u8RxData++;		
			
			if(g_u8RxDataCounter < g_u8DataLen-1 )
			{
				I2C_SET_CONTROL_REG(I2C1, I2C_AA|I2C_SI);					
			}
			else
			{
				/*If next byte is the last byte, prepare to send NACK to slave*/
				I2C_SET_CONTROL_REG(I2C1, I2C_SI);						  
			}					
    } 	
    else if(u32Status == 0x58)                              /* Last data byte has been received and NACK has been returned to slave */
    {
		    *g_u8RxData = I2C_GET_DATA(I2C1);                   /* Get I2C data */
			  //printf("*g_u8RxData = %x\n", *g_u8RxData);
        I2C_SET_CONTROL_REG(I2C1, I2C_STO | I2C_SI);        /* I2C STOP */
        g_u8EndFlag = 1;                                    /* Exit wait loop in main function */	
		}			
		else {
        /* TO DO */
        printf("I2C_MasterRx Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
	  //printf("I2C_MasterTx Status 0x%x \n", u32Status);
	  
	  if (u32Status == 0x08)                          /* START has been transmitted */
    {
 			  I2C_SET_DATA(I2C1, g_u8DeviceAddr << 1);    /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C1, I2C_SI);          /* Trigger I2C */
    } 
	  else if (u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
		{                           
        I2C_SET_DATA(I2C1, g_u8SlaveRegrAddr);      /* Send register addr to slave */
        I2C_SET_CONTROL_REG(I2C1, I2C_SI);          /* Trigger I2C */
    } 
		else if (u32Status == 0x20)                         /* SLA+W has been transmitted and NACK has been received */
		{                     
        I2C_SET_CONTROL_REG(I2C1, I2C_STO | I2C_SI);    /* I2C STOP */
    } 
		else if (u32Status == 0x28)                         /* Reg addr or DATA has been transmitted and ACK has been received */
		{  
        //printf("g_u8DataLen = %d, g_u8TxDataCounter = %d\n",g_u8DataLen, g_u8TxDataCounter);			
			  if(g_u8TxDataCounter == g_u8DataLen)
				{
					I2C_SET_CONTROL_REG(I2C1, I2C_STO | I2C_SI);    /* I2C STOP */
					g_u8EndFlag = 1;                                /* Exit wait loop in main function */  
				}	
        else
        {		
					I2C_SET_DATA(I2C1, *g_u8TxData);                 /* Write data to slave's register*/
					I2C_SET_CONTROL_REG(I2C1, I2C_SI);               /* Trigger I2C */
					g_u8TxData++;
					g_u8TxDataCounter++;
				}					
				
    } 
		else {
        /* TO DO */
        printf("I2C_MasterTx Status 0x%x is NOT processed\n", u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Tx and Rx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_SlaveTRx(uint32_t u32Status)
{
	  uint8_t ReturnData;
	  //printf("I2C_SlaveTRx Status 0x%x \n", u32Status);
	
    if(u32Status == 0x60)                               /* Own SLA+W has been receive; ACK has been return */
		{                             
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);     /* Set ACK from slave side */
    } 
		else if(u32Status == 0x80)                          /* After SLA+W, first data is register address, and then followed by data to write to slave; ACK has been returned*/                                                             
    {
			  if(g_u8SlaveWriteDataLen==0)
				{
					g_u8SlaveRegisterAddr = I2C_GET_DATA(I2C0);   /* register address*/
					//printf("I2C_SlaveTRx Status 0x%x, g_u8SlaveRegisterAddr=%d \n", u32Status, g_u8SlaveRegisterAddr);
					g_u8SlaveWriteDataLen++;
				}					
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);     /* Set ACK as slave side */
    } 
		else if (u32Status == 0xA0)                         /* A STOP or repeated START has been received while still addressed as Slave/Receiver*/                                               
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);     /* Set ACK as slave side */
    } 
		else if(u32Status == 0xA8)                          /* SLA+R has been receive; ACK has been return */
		{              
        g_u8SlaveWriteDataLen = 0;
			  printf(" sta 0x%x, Addr=%d \n", u32Status, g_u8SlaveRegisterAddr);
			  I2C_HandleSlaveReadCmd(g_u8SlaveRegisterAddr, &ReturnData);  /* Return data of designated reigster addr*/
			  I2C_SET_DATA(I2C0, ReturnData);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);                  /* Set ACK as slave side */
    } 
		else if(u32Status == 0xB8)                                       /* data has been received, ACK is returned to slave, continue to return data to host */
		{		  
			  g_u8SlaveRegisterAddr++;
			  printf("sta 0x%x, Addr=%d \n", u32Status, g_u8SlaveRegisterAddr);
			  I2C_HandleSlaveReadCmd(g_u8SlaveRegisterAddr, &ReturnData);  /* Return data of designated reigster addr*/
			  I2C_SET_DATA(I2C0, ReturnData);
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);                  /* Set ACK as slave side */
				
		}			
		else if(u32Status == 0xC0)                          /* Data byte or last data in I2CDAT has been transmitted, Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);     /* Set ACK as slave side */
    } 
		else if (u32Status == 0x88)                         /* Previously addressed with own SLA address; NOT ACK has been returned */                                                        
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);     /* Set ACK as slave side */
    } 
		else 
		{
        /* TO DO */
        printf("I2C_SlaveTRx Status 0x%x is NOT processed\n", u32Status);
    }
}

/* Return specific data to host*/
uint8_t I2C_HandleSlaveReadCmd(uint8_t RegAddr, uint8_t* ReturnData)
{
	uint8_t ErrorCode = 0;
	
	
	switch(RegAddr)
	{
		case REG_CHIP_ID:             /*CHIP ID*/
			*ReturnData = 0xAA;
			break;
		
		case REG_TEMP_MSB:             /*Temperature MSB*/
			*ReturnData = Temperature_MSB;
			break;
		
		case REG_TEMP_LSB:
			*ReturnData = Temperature_LSB;
			break;
		
		case REG_PRESSURE_MSB:
			*ReturnData = Pressure_MSB;
			break;
		
		case REG_PRESSURE_LSB:
			*ReturnData = Pressure_LSB;
			break;
		
		case REG_HUMIDITY_MSB:
			*ReturnData = Humidity_MSB;
			break;
		
		case REG_HUMIDITY_LSB:
			*ReturnData = Humidity_LSB;
			break;
		
		case REG_IAQ_MSB:
			*ReturnData = IAQ_MSB;
			break;
		
		case REG_IAQ_LSB:
			*ReturnData = IAQ_LSB;
			break;
		
		case REG_IAQ_ACCURACY:
			*ReturnData = iaq_accuracy;
			break;
		
		case REG_ACC_X_MSB:
			*ReturnData = Acc_X_MSB;
		  //printf("acc x LSB: %x\n", *ReturnData);
			break;
		
		case REG_ACC_X_LSB:
			*ReturnData = Acc_X_LSB;
			break;
		
		case REG_ACC_Y_MSB:
			*ReturnData = Acc_Y_MSB;
			break;
		
		case REG_ACC_Y_LSB:
			*ReturnData = Acc_Y_LSB;
			break;
		
		case REG_ACC_Z_MSB:
			*ReturnData = Acc_Z_MSB;
			break;
		
		case REG_ACC_Z_LSB:
			*ReturnData = Acc_Z_LSB;
			break;
	
    default:
			*ReturnData = 0;
			break;
	}
	//printf("Read reg addr=%d, Return Data = %X\n",RegAddr, *ReturnData);
  return ErrorCode;
	
}


int8_t I2C_MasterWriteDataToSlave(uint8_t slave_addr, uint8_t regr_addr, uint8_t *write_data, uint16_t data_len)
{
	/* Set global variables */
  g_u8DeviceAddr = slave_addr;
  g_u8SlaveRegrAddr = regr_addr;
	g_u8DataLen = data_len;
	g_u8TxDataCounter = 0;
	g_u8EndFlag = 0;
	g_u8TxData = write_data;
	
	/* Set I2C call-back function for write operation */
	s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterTx;

	/* I2C as master sends START signal */
  I2C_SET_CONTROL_REG(I2C1, I2C_STA);

  /* Wait I2C Tx Finish */
  while(g_u8EndFlag == 0);

  /* Make sure I2C1 STOP already */
  while(I2C1->CTL & I2C_CTL_STO_Msk);

  /* Reset variable */
  g_u8EndFlag = 0;

  /* Wait write operation complete */
	delay_ms(1);
	return 0;
}

int8_t I2C_MasterReadDataFromSlave(uint8_t slave_addr, uint8_t regr_addr, uint8_t *reg_data_ptr, uint16_t data_len)
{
	/* Set global variables */
  g_u8DeviceAddr = slave_addr;
	g_u8SlaveRegrAddr = regr_addr;
	g_u8DataLen = data_len;        // number of bytes to read from slave
	g_u8EndFlag = 0;
	g_u8RxDataCounter = 0;
	
	//printf("slave addr = %x, regr_addr=%x\n", slave_addr, regr_addr);
	
	/* Set I2C call-back function for read operation */
	s_I2C1HandlerFn = (I2C_FUNC)I2C_MasterRx;

	/* Pass the receive buffer address to I2C read function*/
	g_u8RxData = reg_data_ptr;
	
	/* I2C as master sends START signal */
	I2C_SET_CONTROL_REG(I2C1, I2C_STA);
	
	/* Wait I2C Rx Finish */
	while (g_u8EndFlag == 0);
	
	/* Make sure I2C1 STOP already */
	while(I2C1->CTL & I2C_CTL_STO_Msk);
	
	return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Configure I2C0 (Slave) to 100KHz speed and enable interrupt                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_Init(void)
{
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    //printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
    I2C_SetSlaveAddr(I2C0, 0, SLAVE_ADDRESS, I2C_GCMODE_DISABLE);   /* Set Slave Address (0x33)*/
    
    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
	
	  /* I2C enter no address SLV mode (slave) */
    I2C_SET_CONTROL_REG(I2C0, I2C_SI | I2C_AA);
}
/*---------------------------------------------------------------------------------------------------------*/
/* Configure I2C1 (Master) to 100KHz speed and enable interrupt                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void I2C1_Init(void)
{
    /* Open I2C1 and set clock to 100k */
    I2C_Open(I2C1, 100000);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

	  I2C_SetSlaveAddr(I2C1, 0, 0x15, I2C_GCMODE_DISABLE);   /* Slave Address : 0x15 */
    I2C_SetSlaveAddr(I2C1, 1, 0x35, I2C_GCMODE_DISABLE);   /* Slave Address : 0x35 */

    /* Enable I2C1 interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);

   
}

int64_t Get_stamp_us(void)
{
	int64_t timestamp_us = 0;
	
	timestamp_us = Global_timestamp_ms*1000;
  //printf("Get_stamp_us()timestamp_us = %lld \n",timestamp_us);
	
	return timestamp_us;
}

void delay_ms(uint32_t d_ms)
{
	int64_t target_time = Global_timestamp_ms + d_ms;
	
	//printf("1 Global_timestamp_ms = %lld, target_time=%lld\n",Global_timestamp_ms,target_time);
	
	while(Global_timestamp_ms < target_time);
	
	//printf("2 Global_timestamp_ms = %lld, target_time=%lld\n",Global_timestamp_ms,target_time);
	
}

uint32_t Load_State_from_Flash(uint8_t *state_buffer, uint32_t n_buffer)
{
	//0. Buffer length is n_buffer
	//1. Read data from flash
	//2. Copy data to state_buffer
	uint32_t u32Addr; 
	uint32_t u32data;
  uint32_t u32state_len;	
	uint32_t* temp_ptr;
	
	temp_ptr = (uint32_t *)state_buffer;
	u32state_len = FMC_Read(DATA_FLASH_BASE);
	printf("u32state_len = %x\n",u32state_len);
	
	if(u32state_len!=0xFFFFFFFF)
	{
		for(u32Addr = DATA_FLASH_BASE+4; u32Addr < (DATA_FLASH_BASE+4+u32state_len); u32Addr += 4)
		{
			u32data = FMC_Read(u32Addr);
			*temp_ptr = u32data;
			temp_ptr++;
		}
		return u32state_len;
	}
	else
		return 0;
	
}


uint32_t Load_Config(uint8_t *state_buffer, uint32_t n_buffer)
{
	//state_buffer = &bsec_config_iaq[0];
  return 304;	
}

void Save_State_to_Flash(const uint8_t *state_buffer, uint32_t length)
{
	//Save state_buffer to flash
	
	uint32_t u32Addr;                  /* flash address */
	uint32_t u32Pattern;
	uint8_t i = 0;
	uint32_t *temp_ptr;
	temp_ptr = (uint32_t *)state_buffer;
	
	/* Erase page */
  FMC_Erase(DATA_FLASH_BASE);
	
	/* Save the length into first byte of data flash*/
	FMC_Write(DATA_FLASH_BASE, length);
	
	/* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
  for(u32Addr = DATA_FLASH_BASE+4; u32Addr < (DATA_FLASH_BASE+4+length); u32Addr += 4)
	{		
			u32Pattern = *(temp_ptr+i);
		  printf("u32Pattern = %x\n",u32Pattern);
			FMC_Write(u32Addr, u32Pattern);          /* Program flash */
			i++;
  }
		
}

int set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];          /* User Configuration */

    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0) {
        printf("\nRead User Config failed!\n");       /* Error message */
        return -1;                     /* failed to read User Configuration */
    }

    /* Check if Data Flash is enabled and is expected address. */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;                      /* no need to modify User Configuration */

    FMC_ENABLE_CFG_UPDATE();           /* Enable User Configuration update. */

    au32Config[0] &= ~0x1;             /* Clear CONFIG0 bit 0 to enable Data Flash */
    au32Config[1] = u32DFBA;           /* Give Data Flash base address  */

    /* Update User Configuration settings. */
    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;                     /* failed to write user configuration */

    //printf("\nSet Data Flash base as 0x%x.\n", DATA_FLASH_TEST_BASE);  /* debug message */

    /* Perform chip reset to make new User Config take effect. */
    SYS->IPRST1 = SYS_IPRST1_CHIPRST_Msk;
    return 0;                          /* success */
}

void Output_ready(int64_t timestamp, float iaq, uint8_t iaq_accuracy, float temperature, float humidity,
    float pressure, float raw_temperature, float raw_humidity, float gas, bsec_library_return_t bsec_status)
{
	printf("IAQ=%f\n", iaq);
	printf("IAQ accuracy=%d\n", iaq_accuracy);
	printf("temperature=%f\n", temperature);
	printf("humidity=%f\n", humidity);
	printf("pressure=%f\n", pressure);
	printf("raw_temperature=%f\n", raw_temperature);
	printf("gas=%f\n", gas);
}
