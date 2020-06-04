#include "print_func.hpp"
#include "PID.h"
#define FLASH_USER_START_ADDR   ADDR_FLASH_SECTOR_11   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR     ADDR_FLASH_SECTOR_11  +  GetSectorSize(ADDR_FLASH_SECTOR_11) -1 /* End @ of user Flash area : sector start address + sector size -1 */

#define DATA_32                 ((uint32_t)0x12345678)

#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) /* Base @ of Sector 0, 16 Kbytes */
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) /* Base @ of Sector 1, 16 Kbytes */
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) /* Base @ of Sector 2, 16 Kbytes */
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) /* Base @ of Sector 3, 16 Kbytes */
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) /* Base @ of Sector 4, 64 Kbytes */
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) /* Base @ of Sector 5, 128 Kbytes */
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) /* Base @ of Sector 6, 128 Kbytes */
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) /* Base @ of Sector 7, 128 Kbytes */
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) /* Base @ of Sector 8, 128 Kbytes */
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) /* Base @ of Sector 9, 128 Kbytes */
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) /* Base @ of Sector 10, 128 Kbytes */
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) /* Base @ of Sector 11, 128 Kbytes */

uint32_t FirstSector = 0, NbOfSectors = 0, Address = 0;
uint32_t SectorError = 0;
__IO uint32_t data32 = 0 , MemoryProgramStatus = 0;

/*Variable used for Erase procedure*/
static FLASH_EraseInitTypeDef EraseInitStruct;

/* Private function prototypes -----------------------------------------------*/
static uint32_t GetSector(uint32_t Address);
static uint32_t GetSectorSize(uint32_t Sector);


void decode(uint8_t * buffer,int length, float * data){
//	char buffer[4*length];
	uint8_t temp[4];
    uint8_t i = 0;

//    int length;
//    length = strlen(buffer);

    for(int count = 0; count <= length - 4; count+=4){
    	temp[0] = buffer[count];
    	temp[1] = buffer[count+1];
    	temp[2] = buffer[count+2];
    	temp[3] = buffer[count+3];

    	data[i] = bytes2Float(temp);
        i++;
    }
//	HAL_UART_Transmit(&huart3,(uint8_t*) buffer, strlen(buffer),1000);
}
void writeFlash(uint16_t num[12]){
	 HAL_FLASH_Unlock();

	  /* Erase the user Flash area
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/

	  /* Get the 1st sector to erase */
	  FirstSector = GetSector(FLASH_USER_START_ADDR);
	  /* Get the number of sector to erase from 1st sector*/
	  NbOfSectors = GetSector(FLASH_USER_END_ADDR) - FirstSector + 1;

	  /* Fill EraseInit structure*/
	  EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
	  EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;
	  EraseInitStruct.Sector = FirstSector;
	  EraseInitStruct.NbSectors = NbOfSectors;
	  if(HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError) != HAL_OK)
	  {
	    /*
	      Error occurred while sector erase.
	      User can add here some code to deal with this error.
	      SectorError will contain the faulty sector and then to know the code error on this sector,
	      user can call function 'HAL_FLASH_GetError()'
	    */
	    /*
	      FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
	    */

	  }

	  /* Note: If an erase operation in Flash memory also concerns data in the data or instruction cache,
	     you have to make sure that these data are rewritten before they are accessed during code
	     execution. If this cannot be done safely, it is recommended to flush the caches by setting the
	     DCRST and ICRST bits in the FLASH_CR register. */
	  __HAL_FLASH_DATA_CACHE_DISABLE();
	  __HAL_FLASH_INSTRUCTION_CACHE_DISABLE();

	  __HAL_FLASH_DATA_CACHE_RESET();
	  __HAL_FLASH_INSTRUCTION_CACHE_RESET();

	  __HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	  __HAL_FLASH_DATA_CACHE_ENABLE();

	  /* Program the user Flash area word by word
	    (area defined by FLASH_USER_START_ADDR and FLASH_USER_END_ADDR) ***********/


//	  float num[32] = {1.686,6.79,10.54,1.64325,8292.02141,675.686,9.79,0.54,1.25,8.5,11.686,96.9,101.54,1.564325,8241,9,
//	     		1.08,6.09,10.34,1.64325,1254.352,675.686,9.79,0.54,1.25,8.5,1.68,6.9,11.54,0.5,8.1,92};



//	  uint8_t temp[4];
	  int i = 0;

	  Address = FLASH_USER_START_ADDR;

	  while (i < 12)
	  {
//		float2Bytes(num[i],temp);
		uint32_t temp_val = (num[i] <<16) | (num[i + 1]);
	    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, Address, temp_val) == HAL_OK)
	    {
	      Address = Address + 4;
	    }
	    else
	    {
	      /* Error occurred while writing data in Flash memory.
	         User can add here some code to deal with this error */
	      /*
	        FLASH_ErrorTypeDef errorcode = HAL_FLASH_GetError();
	      */

	    }
	    i+=2;
	  }

	  /* Lock the Flash to disable the flash control register access (recommended
	     to protect the FLASH memory against possible unwanted operation) *********/
	  HAL_FLASH_Lock();
}

PID_value readFlash(){
	  PID_value pid;
	  Address = FLASH_USER_START_ADDR;
	  MemoryProgramStatus = 0x0;
	  uint16_t temp_arr[12];
	  uint16_t result,result1;

	  uint8_t count = 0;
	  while (Address < FLASH_USER_START_ADDR + 2*12)
	  {
	    data32 = *(__IO uint32_t*)Address;

	    temp_arr[count] = (uint16_t) ((data32 &0xFFFF0000)>>16);
	    temp_arr[count+1] = (uint16_t) (data32 & 0x0000FFFF);


//	    temp_arr[2] = (uint8_t) ((data32 &0x0000FF00)>>8);
//	    temp_arr[3] = (uint8_t) data32 &0x000000FF;
//	    result = bytes2Float(temp_arr);
        count += 2;
	    Address = Address + 4;
	  }
      pid.Kp1 = temp_arr[0];
      pid.Ki1 = temp_arr[1];
      pid.Kd1 = temp_arr[2];

      pid.Kp2 = temp_arr[3];
      pid.Ki2 = temp_arr[4];
      pid.Kd2 = temp_arr[5];

      pid.Kp3 = temp_arr[6];
      pid.Ki3 = temp_arr[7];
      pid.Kd3 = temp_arr[8];

      pid.Kp4 = temp_arr[9];
      pid.Ki4 = temp_arr[10];
      pid.Kd4 = temp_arr[11];
      return pid;
}
/**
  * @brief  Gets the sector of a given address
  * @param  None
  * @retval The sector of a given address
  */
static uint32_t GetSector(uint32_t Address)
{
  uint32_t sector = 0;

  if((Address < ADDR_FLASH_SECTOR_1) && (Address >= ADDR_FLASH_SECTOR_0))
  {
    sector = FLASH_SECTOR_0;
  }
  else if((Address < ADDR_FLASH_SECTOR_2) && (Address >= ADDR_FLASH_SECTOR_1))
  {
    sector = FLASH_SECTOR_1;
  }
  else if((Address < ADDR_FLASH_SECTOR_3) && (Address >= ADDR_FLASH_SECTOR_2))
  {
    sector = FLASH_SECTOR_2;
  }
  else if((Address < ADDR_FLASH_SECTOR_4) && (Address >= ADDR_FLASH_SECTOR_3))
  {
    sector = FLASH_SECTOR_3;
  }
  else if((Address < ADDR_FLASH_SECTOR_5) && (Address >= ADDR_FLASH_SECTOR_4))
  {
    sector = FLASH_SECTOR_4;
  }
  else if((Address < ADDR_FLASH_SECTOR_6) && (Address >= ADDR_FLASH_SECTOR_5))
  {
    sector = FLASH_SECTOR_5;
  }
  else if((Address < ADDR_FLASH_SECTOR_7) && (Address >= ADDR_FLASH_SECTOR_6))
  {
    sector = FLASH_SECTOR_6;
  }
  else if((Address < ADDR_FLASH_SECTOR_8) && (Address >= ADDR_FLASH_SECTOR_7))
  {
    sector = FLASH_SECTOR_7;
  }
  else if((Address < ADDR_FLASH_SECTOR_9) && (Address >= ADDR_FLASH_SECTOR_8))
  {
    sector = FLASH_SECTOR_8;
  }
  else if((Address < ADDR_FLASH_SECTOR_10) && (Address >= ADDR_FLASH_SECTOR_9))
  {
    sector = FLASH_SECTOR_9;
  }
  else if((Address < ADDR_FLASH_SECTOR_11) && (Address >= ADDR_FLASH_SECTOR_10))
  {
    sector = FLASH_SECTOR_10;
  }
  else /* (Address < FLASH_END_ADDR) && (Address >= ADDR_FLASH_SECTOR_11) */
  {
    sector = FLASH_SECTOR_11;
  }

  return sector;
}

/**
  * @brief  Gets sector Size
  * @param  None
  * @retval The size of a given sector
  */
static uint32_t GetSectorSize(uint32_t Sector)
{
  uint32_t sectorsize = 0x00;

  if((Sector == FLASH_SECTOR_0) || (Sector == FLASH_SECTOR_1) || (Sector == FLASH_SECTOR_2) || (Sector == FLASH_SECTOR_3))
  {
    sectorsize = 16 * 1024;
  }
  else if(Sector == FLASH_SECTOR_4)
  {
    sectorsize = 64 * 1024;
  }
  else
  {
    sectorsize = 128 * 1024;
  }
  return sectorsize;
}
