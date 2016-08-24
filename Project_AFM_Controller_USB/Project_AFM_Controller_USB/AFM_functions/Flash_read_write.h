//#include "mbed.h"
//
//#include "stm32f7xx_hal_flash.h"
//
//
//uint32_t startAddress = 0x8040000;//starting from 256KB
//                                  //0x8020000 starting 128KB
//void writeFlash(void)
//{
//	uint32_t i, j;
//    HAL_FLASH_Unlock();//unlock flash writing
//FLASH_Unlock();
//FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR | FLASH_FLAG_OPTERR);
//FLASHStatus = FLASH_ErasePage(Page);ePage(startAddress);//erase the entire page before you can write as I //mentioned
//    
//    for(i=0; i<mSize; i++)
//        for(j=0; j<mSize; j++)              
//            FLASH_ProgramHalfWord((startAddress + (i*mSize+j)*2),mazeWalls[i][j]);
//
//    FLASH_Lock();//lock the flash for writing
//
//}
//
//void readFlash(void)
//{
//    u32 i, j;    
//    for(i=0; i<mSize; i++)
//        for(j=0; j<mSize; j++)       
//            mazeWalls[i][j] = *(uint16_t *)(startAddress + (i*mSize+j)*2);
//}