/**
  ******************************************************************************
  * @file    usbd_msc_storage_template.c
  * @author  MCD Application Team
  * @version V2.4.1
  * @date    19-June-2015
  * @brief   Memory management layer
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 


/* Includes ------------------------------------------------------------------*/
#include "usbd_msc_storage_if.h"
#include "spi_flash.h"
#include "uart_debug.h"


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Extern function prototypes ------------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

#define STORAGE_LUN_NBR                  1  
#define STORAGE_BLK_NBR                  0x1000
#define STORAGE_BLK_SIZ                  0x1000

void (*STORAGE_ChipSelect)(void) = &SPI_FLASH_SelectChip1;
void (*STORAGE_ChipDeselect)(void) = &SPI_FLASH_SelectNone;
extern SPI_HandleTypeDef hspi_ms;

int8_t STORAGE_Init (uint8_t lun);

int8_t STORAGE_GetCapacity (uint8_t lun, 
                           uint32_t *block_num, 
                           uint16_t *block_size);

int8_t  STORAGE_IsReady (uint8_t lun);

int8_t  STORAGE_IsWriteProtected (uint8_t lun);

int8_t STORAGE_Read (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,
                        uint16_t blk_len);

int8_t STORAGE_Write (uint8_t lun, 
                        uint8_t *buf, 
                        uint32_t blk_addr,
                        uint16_t blk_len);

int8_t STORAGE_GetMaxLun (void);

/* USB Mass storage Standard Inquiry Data */
int8_t  STORAGE_Inquirydata[] = {//36
  
  /* LUN 0 */
  0x00,        
  0x80,        
  0x02,        
  0x02,
  (STANDARD_INQUIRY_DATA_LEN - 5),
  0x00,
  0x00,    
  0x00,
  'B', 'i', 'n', 'a', 'r', 'y', 'I', 'O', /* Manufacturer : 8 bytes */
  'P', 'r', 'o', 'j', 'e', 'c', 't', ' ', /* Product      : 16 Bytes */
  'P', 'a', 't', 'r', 'o', 'n', 'u', 's',
  '0', '.', '0' ,'1',                     /* Version      : 4 Bytes */
}; 

USBD_StorageTypeDef USBD_MSC_Template_fops =
{
  STORAGE_Init,
  STORAGE_GetCapacity,
  STORAGE_IsReady,
  STORAGE_IsWriteProtected,
  STORAGE_Read,
  STORAGE_Write,
  STORAGE_GetMaxLun,
  STORAGE_Inquirydata,
  
};
/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the microSD card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Init (uint8_t lun)
{
  return (0);
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetCapacity (uint8_t lun, uint32_t *block_num, uint16_t *block_size)
{
  *block_num  = STORAGE_BLK_NBR;
  *block_size = STORAGE_BLK_SIZ;
  return (0);
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsReady (uint8_t lun)
{
    return 1;
    //return 0; //@TODO re-enable this. Disabled to prevent resource contention with Tunnel HID while SPI port is shared.
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t  STORAGE_IsWriteProtected (uint8_t lun)
{
  return  0;
  //@TODO set this up so it is normally write-protected. require a special authorized command to unprotect it (prevents inadvertent changes)
}

/*******************************************************************************
* Function Name  : Read_Memory
* Description    : Handle the Read operation from the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Read (uint8_t lun, 
                 uint8_t *buf, 
                 uint32_t blk_addr,                       
                 uint16_t blk_len)
{
    /*uart_debug_addToBuffer("MSC Storage Read (", 18);
    uart_debug_printuint8(lun);
    uart_debug_addToBuffer(", ", 2);
    uart_debug_hexprint32(blk_addr);
    uart_debug_addToBuffer(", ", 2);
    uart_debug_hexprint32(blk_len);
    uart_debug_addToBuffer(")\n", 2);*/
    uint16_t readLen = blk_len * STORAGE_BLK_SIZ;
    STORAGE_ChipSelect();
    uint16_t rv = SPI_FLASH_ReadData(&hspi_ms, buf, readLen, blk_addr << 12);
    STORAGE_ChipDeselect();
    if(rv == readLen) {
        return 0;
    } else {
        uart_debug_sendline("Error reading sector.\n");
        return 1;
    }
}
/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_Write (uint8_t lun, 
                  uint8_t *buf, 
                  uint32_t blk_addr,
                  uint16_t blk_len)
{
    /*uart_debug_addToBuffer("MSC Storage Write (", 19);
    uart_debug_printuint8(lun);
    uart_debug_addToBuffer(", ", 2);
    uart_debug_hexprint32(blk_addr);
    uart_debug_addToBuffer(", ", 2);
    uart_debug_hexprint32(blk_len);
    uart_debug_addToBuffer(")\n", 2);*/
    STORAGE_ChipSelect();
    for(uint16_t i = 0; i < blk_len; ++i) {
        uint8_t rv = SPI_FLASH_WriteEnableAndWait(&hspi_ms, 100);
        if(!rv) {
            uart_debug_sendline("Error enabling write for erase.\n");
            STORAGE_ChipDeselect();
            return rv;
        }
        rv =  SPI_FLASH_EraseSector(&hspi_ms, (blk_addr + i) << 12);
        if(rv) {
            uart_debug_sendline("Error erasing sector.\n");
            STORAGE_ChipDeselect();
            return rv;
        }
        SPI_FLASH_ReadSR1UntilNotBusy(&hspi_ms); //wait for erase to complete
    }
    uint32_t write_rv = SPI_FLASH_Write(&hspi_ms, buf, blk_len * STORAGE_BLK_SIZ, blk_addr << 12);
    STORAGE_ChipDeselect();
    if(write_rv) {
        return 1;
    }
    return (0);
}
/*******************************************************************************
* Function Name  : Write_Memory
* Description    : Handle the Write operation to the STORAGE card.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
int8_t STORAGE_GetMaxLun (void)
{
  return (STORAGE_LUN_NBR - 1);
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

