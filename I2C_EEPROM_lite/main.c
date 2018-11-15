/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * $Revision: 5 $
 * $Date: 15/12/25 4:01p $
 * @brief    Read/write EEPROM via an I2C interface.
 *
 * @note
 * 
 *
 ******************************************************************************/
/* Standard includes. */
#include <stdio.h>
#include "bsec_integration.h"
#include "Nano_support.h"
#include "Nano103.h"

#define Global_TIMER_FREQ 1000  //Global timer 1000Hz

/*---------------------------------------------------------------------------------------------------------*/
/* Init System Clock                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set HCLK source form HXT and HCLK source divide 1  */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HXT,CLK_HCLK_CLK_DIVIDER(1));

    /* Enable external 12MHz HXT, 32KHz LXT and HIRC */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk | CLK_PWRCTL_HIRC0EN_Msk | CLK_PWRCTL_HIRC1EN_Msk | CLK_PWRCTL_MIRCEN_Msk);

    /* Waiting for clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk | CLK_STATUS_HIRC0STB_Msk | CLK_STATUS_HIRC1STB_Msk | CLK_STATUS_MIRCSTB_Msk);

    /*  Set HCLK frequency 32MHz */
    CLK_SetCoreClock(HCLK_FREQ);
	  
	  /* Enable sys tick counter*/
	  CLK_EnableSysTick(CLK_CLKSEL0_STCLKSEL_HCLK, 0xFFFFFF); /* 24-bit Counter*/
	

    /* Enable IP clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(I2C0_MODULE);  //I2C master
    CLK_EnableModuleClock(I2C1_MODULE);  //I2C slave
		CLK_EnableModuleClock(TMR0_MODULE);  //timer
		
    /* Select IP clock source */
    CLK_SetModuleClock(UART0_MODULE,CLK_CLKSEL1_UART0SEL_HIRC,CLK_UART0_CLK_DIVIDER(1));
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HXT, CLK_TMR0_CLK_DIVIDER(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PA multi-function pins for UART0 RXD and TXD */
    SYS->GPB_MFPL &= ~( SYS_GPB_MFPL_PB0MFP_Msk | SYS_GPB_MFPL_PB1MFP_Msk);
    SYS->GPB_MFPL |= (SYS_GPB_MFPL_PB0MFP_UART0_RXD | SYS_GPB_MFPL_PB1MFP_UART0_TXD );

    /* Set PA multi-function pins for I2C0/I2C1 */
    /* I2C0: GPA8 - SDA, GPA9 - SCL */
    /* I2C1: GPA10 - SDA, GPA11 - SCL */
    /*Nano103 EVT*/
		//SYS->GPA_MFPH = (SYS_GPA_MFPH_PA8MFP_I2C0_SDA | SYS_GPA_MFPH_PA9MFP_I2C0_SCL | SYS_GPA_MFPH_PA10MFP_I2C1_SDA | SYS_GPA_MFPH_PA11MFP_I2C1_SCL);

    /*Advantech board*/
		SYS->GPA_MFPH = (SYS_GPA_MFPH_PA14MFP_I2C1_SDA | SYS_GPA_MFPH_PA15MFP_I2C1_SCL);
    SYS->GPA_MFPL = (SYS_GPA_MFPL_PA4MFP_I2C0_SDA | SYS_GPA_MFPL_PA5MFP_I2C0_SCL);
		
    /* Lock protected registers */
    SYS_LockReg();
}



/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main (void)
{ 
	uint8_t a_data;
	return_values_init ret;   
	
	/* Init System, IP clock and multi-function I/O */
  SYS_Init();
  
  TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, Global_TIMER_FREQ); /* Set timer frequency to 1000Hz */
  TIMER_EnableInt(TIMER0);                                    /* Enable timer interrupt */
  NVIC_EnableIRQ(TMR0_IRQn);
  TIMER_Start(TIMER0);                                        /* Start Timer 0 */
	
  /* Init UART to 115200-8n1 for print message */
  UART_Open(UART0, 115200);
	 
	printf("+-------------------------------------------------------+\n");
  printf("|    Nano103 IAQ sensor hub										          |\n");
  printf("+-------------------------------------------------------+\n");
  
	//SYS_UnlockReg();                   /* Unclok register lock protect */
	//FMC_Open();  											 /* Enable FMC ISP function */
	//set_data_flash_base(DATA_FLASH_BASE);
	
  I2C1_Init(); /* Init I2C1 as I2C master */
	I2C0_Init(); /* Init I2C0 as I2C slave */

	I2C_MasterReadDataFromSlave(0x18, 0x00, &a_data, 1);
	printf("adata=%x\n",a_data);
	
  //Save_State_to_Flash(&test[0],400);
  //Load_State_from_Flash(arr,400);
	
	/* Call to the function which initializes the BSEC library */
	ret = bsec_iot_init(BSEC_SAMPLE_RATE_LP, 0.0f, I2C_MasterWriteDataToSlave, I2C_MasterReadDataFromSlave, delay_ms, Load_State_from_Flash, Load_Config);
	if (ret.bme680_status)
  {
		/* Could not intialize BME680 or BSEC library */
    return (int)ret.bme680_status;
  }
  else if (ret.bsec_status)
  {
		/* Could not intialize BSEC library */    
    return (int)ret.bsec_status;
  }
  /* Call to endless loop function which reads and processes data based on sensor settings */  
  bsec_iot_loop(delay_ms, Get_stamp_us, Output_ready, Save_State_to_Flash, 10000); 

	while(1);
}



