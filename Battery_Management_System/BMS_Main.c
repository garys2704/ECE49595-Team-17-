

#include "BMS.h"



int main(void)
{
	MX_GPIO_Init();
	MX_I2C1_Init();


	if (MAX17055_Init(&hi2c1) != HAL_OK)
 	{
        //Error_Handler();
		print("BMS Init Failed");
 	}

	void Battery_Task(void)
	{
      uint16_t soc_raw;
      float soc;

      if (MAX17055_ReadReg(&hi2c1, REG_REPSOC, &soc_raw) == HAL_OK)
      {
    	  soc = soc_raw / 256.0f;
      }
	}

	//Main Loop
	while (1)
	{
		Battery_Task();
		HAL_Delay(1000);
	}
}
