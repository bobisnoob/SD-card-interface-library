/** * ******************************************************************************
  * File Name     : sdcard.c
  * PROJECT       : sdcard library for MMC/SDC cards (NOT compatible with high capacity cards) 
  * PROGRAMMER    : Abdel Alkuor 
  * FIRST VERSION : 2017-4-08
  * Description   : This library can be used along with FATFs library to read/write data
  					to sd card.
  					This library is can be implemented on STM32F3 discovery board with pins connections
  					as follows:
  					PC12 ---> MOSI
  					PC11 ---> MISO
  					PC10 ---> SCK
					PB0  ---> CS 
  *
  *************************************************************************************/

#include "sdcard.h"


uint8_t dummyByte = 0xFF;

uint8_t AA_SDcard_Init(void)
{
	AA_CS_HIGH();
	for(uint8_t counter = 0; counter<=9 ; counter++)
	{
		AA_SD_WriteByte(dummyByte);
	}
	return(AA_SDcard_IdleState());
}

uint8_t AA_SDcard_IdleState(void)
{

	if( AA_SD_SendCmd(SD_CMD_IDLE, 0 , 0x95, SD_IDLE_STATE) != SD_OK)
	{
		return SD_ERROR;
	}
	while(AA_SD_SendCmd(SD_CMD_SEND_OP_COND, 0, 0xFF, SD_RESPONSE_NO_ERROR) != SD_OK);

	return SD_OK;
}


uint8_t AA_SDcard_ReadBlocks(uint8_t *data,uint64_t dataAddress, uint16_t blockSize,uint32_t numberOfBlocks)
{
	uint32_t counter = 0;
	uint32_t offSet = 0;
	uint8_t status = SD_ERROR;

	if( AA_SDcard_WriteCmd(SD_CMD_SET_BLOCKLEN, blockSize, 0xFF, SD_RESPONSE_NO_ERROR) != AA_OK)
	{
		return SD_ERROR;
	}

	while(numberOfBlocks--)
	{
		AA_SD_WriteDummy();

		if(AA_SDcard_WriteCmd(SD_CMD_READ_SINGLE_BLOCK, dataAddress+ offSet, 0xFF, SD_RESPONSE_NO_ERROR) != AA_OK)
		{
			return SD_ERROR;
		}

		if(AA_SD_Wait_Response(SD_START_DATA_SINGLE_BLOCK_READ) == AA_OK)
		{
			for (counter = 0; counter < blockSize; counter++)
			{
				*data = AA_SD_ReadByte();
				data++;
			}
			offSet +=blockSize;
			//get CRC bytes
			AA_SD_ReadByte();
			AA_SD_ReadByte();

			status = SD_OK;
		}
		else
		{
			status = SD_ERROR;
		}
	}

	AA_SD_WriteDummy();

	return status;
}

AA_Status AA_SDcard_WriteCmd(uint8_t cmd, uint32_t argument, uint8_t crc, uint8_t response)
{
	uint8_t cmdFrame[6];
	uint32_t counter = 0;

	cmdFrame[0] = (cmd | 0x40);
	cmdFrame[1] = (uint8_t)(argument>>24);
	cmdFrame[2] = (uint8_t)(argument>>16);
	cmdFrame[3] = (uint8_t)(argument>>8);
	cmdFrame[4] = (uint8_t)(argument);
	cmdFrame[5] = (crc);

	AA_CS_LOW();

	//HAL_SPI_Transmit(&hspi2,cmdFrame,6,1000);
	for (counter = 0; counter < 6; counter++)
	 {
	    AA_SD_WriteByte(cmdFrame[counter]); /* Send the Cmd bytes */
	 }
	if( response != SD_NO_RESPONSE_EXPECTED) // try later without it
	{
		return AA_SD_Wait_Response(response);
	}
	return AA_OK;
}

AA_Status AA_SD_Wait_Response(uint8_t response)
{
	uint32_t timeout = 0xFFF;

	while((AA_SD_ReadByte() != response) && timeout)
	{
		timeout--;
	}

	if(timeout == 0)
	{
		return  AA_TIMEOUT;
	}
	else
	{
		return  AA_OK;
	}

}

uint8_t AA_SD_SendCmd(uint8_t cmd, uint32_t argument, uint8_t crc, uint8_t response)
{
	uint8_t status = SD_ERROR;

	if(AA_SDcard_WriteCmd(cmd, argument, crc, response) == AA_OK)
	{
		status = AA_OK;
	}

	AA_SD_WriteDummy();
	return status;
}

uint8_t AA_SD_WriteBlocks(uint8_t *data,uint64_t dataAddress, uint16_t blockSize,uint32_t numberOfBlocks)
{
	uint32_t counter = 0;
	uint32_t offSet	 = 0;
	uint8_t status = AA_ERROR;

	while(numberOfBlocks--)
	{
		if( AA_SDcard_WriteCmd(SD_CMD_WRITE_SINGLE_BLOCK, dataAddress + offSet, 0xFF, SD_RESPONSE_NO_ERROR) != AA_OK)
		{
			return SD_ERROR;
		}
		AA_SD_WriteByte(dummyByte);

		AA_SD_WriteByte(SD_START_DATA_SIGNEL_BLOCK_WRITE);

		for(counter = 0; counter < blockSize; counter++)
		{
			AA_SD_WriteByte(*data);
			data++;
		}

		offSet +=blockSize;
		AA_SD_ReadByte();
		AA_SD_ReadByte();

		if(AA_SD_GetResponse() == SD_DATA_OK)
		{
			status = AA_OK;
		}
		else
		{
			status = AA_ERROR;
		}
	}
	AA_SD_WriteDummy();

	return status;
}

AA_SD_RESPONSES AA_SD_GetResponse(void)
{
	uint32_t counter = 0 ;
	AA_SD_RESPONSES response;
	AA_SD_RESPONSES returnValue;

	while(counter <=64)
	{
		response = (AA_SD_RESPONSES)AA_SD_ReadByte();
		response &= 0x1F;

		switch (response)
		{
			case SD_DATA_OK:
				returnValue = SD_DATA_OK;
				break;
			case SD_DATA_CRC_ERROR:
				return SD_DATA_CRC_ERROR;
			case SD_DATA_WRTIE_ERROR:
				return SD_DATA_WRTIE_ERROR;
			default:
				returnValue = SD_DATA_OTHER_ERROR;
				break;
		}

		if(returnValue == SD_DATA_OK) break;

		counter++;
	}

	while(AA_SD_ReadByte() == 0);

	return response;
}

uint8_t AA_SD_GetCSDRegister(SD_CSD *Csd)
{
	uint32_t counter = 0;
	uint8_t status = SD_ERROR;
	uint8_t CSD_Tab[16];

	if(AA_SDcard_WriteCmd(SD_CMD_SEND_CSD, 0, 0xFF, SD_RESPONSE_NO_ERROR) == AA_OK)
	{
		if(AA_SD_Wait_Response(SD_START_DATA_SINGLE_BLOCK_READ) == AA_OK)
		{
			for(counter = 0; counter <16; counter++)
			{
				CSD_Tab[counter] = AA_SD_ReadByte();
			}

			AA_SD_WriteByte(dummyByte);
			AA_SD_WriteByte(dummyByte);

			status = AA_OK;
		}
	}
	AA_SD_WriteDummy();

	if(status == SD_RESPONSE_NO_ERROR)
	{
	    /* Byte 0 */
	    Csd->CSDStruct = (CSD_Tab[0] & 0xC0) >> 6;
	    Csd->SysSpecVersion = (CSD_Tab[0] & 0x3C) >> 2;
	    Csd->Reserved1 = CSD_Tab[0] & 0x03;

	    /* Byte 1 */
	    Csd->TAAC = CSD_Tab[1];

	    /* Byte 2 */
	    Csd->NSAC = CSD_Tab[2];

	    /* Byte 3 */
	    Csd->MaxBusClkFrec = CSD_Tab[3];

	    /* Byte 4 */
	    Csd->CardComdClasses = CSD_Tab[4] << 4;

	    /* Byte 5 */
	    Csd->CardComdClasses |= (CSD_Tab[5] & 0xF0) >> 4;
	    Csd->RdBlockLen = CSD_Tab[5] & 0x0F;

	    /* Byte 6 */
	    Csd->PartBlockRead = (CSD_Tab[6] & 0x80) >> 7;
	    Csd->WrBlockMisalign = (CSD_Tab[6] & 0x40) >> 6;
	    Csd->RdBlockMisalign = (CSD_Tab[6] & 0x20) >> 5;
	    Csd->DSRImpl = (CSD_Tab[6] & 0x10) >> 4;
	    Csd->Reserved2 = 0; /*!< Reserved */

	    Csd->DeviceSize = (CSD_Tab[6] & 0x03) << 10;

	    /* Byte 7 */
	    Csd->DeviceSize |= (CSD_Tab[7]) << 2;

	    /* Byte 8 */
	    Csd->DeviceSize |= (CSD_Tab[8] & 0xC0) >> 6;

	    Csd->MaxRdCurrentVDDMin = (CSD_Tab[8] & 0x38) >> 3;
	    Csd->MaxRdCurrentVDDMax = (CSD_Tab[8] & 0x07);

	    /* Byte 9 */
	    Csd->MaxWrCurrentVDDMin = (CSD_Tab[9] & 0xE0) >> 5;
	    Csd->MaxWrCurrentVDDMax = (CSD_Tab[9] & 0x1C) >> 2;
	    Csd->DeviceSizeMul = (CSD_Tab[9] & 0x03) << 1;
	    /* Byte 10 */
	    Csd->DeviceSizeMul |= (CSD_Tab[10] & 0x80) >> 7;

	    Csd->EraseGrSize = (CSD_Tab[10] & 0x40) >> 6;
	    Csd->EraseGrMul = (CSD_Tab[10] & 0x3F) << 1;

	    /* Byte 11 */
	    Csd->EraseGrMul |= (CSD_Tab[11] & 0x80) >> 7;
	    Csd->WrProtectGrSize = (CSD_Tab[11] & 0x7F);

	    /* Byte 12 */
	    Csd->WrProtectGrEnable = (CSD_Tab[12] & 0x80) >> 7;
	    Csd->ManDeflECC = (CSD_Tab[12] & 0x60) >> 5;
	    Csd->WrSpeedFact = (CSD_Tab[12] & 0x1C) >> 2;
	    Csd->MaxWrBlockLen = (CSD_Tab[12] & 0x03) << 2;

	    /* Byte 13 */
	    Csd->MaxWrBlockLen |= (CSD_Tab[13] & 0xC0) >> 6;
	    Csd->WriteBlockPaPartial = (CSD_Tab[13] & 0x20) >> 5;
	    Csd->Reserved3 = 0;
	    Csd->ContentProtectAppli = (CSD_Tab[13] & 0x01);

	    /* Byte 14 */
	    Csd->FileFormatGrouop = (CSD_Tab[14] & 0x80) >> 7;
	    Csd->CopyFlag = (CSD_Tab[14] & 0x40) >> 6;
	    Csd->PermWrProtect = (CSD_Tab[14] & 0x20) >> 5;
	    Csd->TempWrProtect = (CSD_Tab[14] & 0x10) >> 4;
	    Csd->FileFormat = (CSD_Tab[14] & 0x0C) >> 2;
	    Csd->ECC = (CSD_Tab[14] & 0x03);

	    /* Byte 15 */
	    Csd->CSD_CRC = (CSD_Tab[15] & 0xFE) >> 1;
	    Csd->Reserved4 = 1;
	  }

	return status;
}

uint8_t AA_SD_GetCIDRegister(SD_CID *Cid)
{
	uint32_t counter = 0;
	uint8_t status = AA_ERROR;
	uint8_t CID_Tab[16];

	if( AA_SDcard_WriteCmd(SD_CMD_SEND_CID, 0, 0xFF, SD_RESPONSE_NO_ERROR) == AA_OK)
	{
		if(AA_SD_Wait_Response(SD_START_DATA_SINGLE_BLOCK_READ) == AA_OK)
		{
			for(counter = 0; counter < 16 ; counter++)
			{
				CID_Tab[counter] = AA_SD_ReadByte();
			}
			AA_SD_WriteByte(dummyByte);
			AA_SD_WriteByte(dummyByte);
			status = AA_OK;

		}
	}

	AA_SD_WriteDummy();

	if(status == SD_RESPONSE_NO_ERROR)
	{
	    /* Byte 0 */
	    Cid->ManufacturerID = CID_Tab[0];

	    /* Byte 1 */
	    Cid->OEM_AppliID = CID_Tab[1] << 8;

	    /* Byte 2 */
	    Cid->OEM_AppliID |= CID_Tab[2];

	    /* Byte 3 */
	    Cid->ProdName1 = CID_Tab[3] << 24;

	    /* Byte 4 */
	    Cid->ProdName1 |= CID_Tab[4] << 16;

	    /* Byte 5 */
	    Cid->ProdName1 |= CID_Tab[5] << 8;

	    /* Byte 6 */
	    Cid->ProdName1 |= CID_Tab[6];

	    /* Byte 7 */
	    Cid->ProdName2 = CID_Tab[7];

	    /* Byte 8 */
	    Cid->ProdRev = CID_Tab[8];

	    /* Byte 9 */
	    Cid->ProdSN = CID_Tab[9] << 24;

	    /* Byte 10 */
	    Cid->ProdSN |= CID_Tab[10] << 16;

	    /* Byte 11 */
	    Cid->ProdSN |= CID_Tab[11] << 8;

	    /* Byte 12 */
	    Cid->ProdSN |= CID_Tab[12];

	    /* Byte 13 */
	    Cid->Reserved1 |= (CID_Tab[13] & 0xF0) >> 4;
	    Cid->ManufactDate = (CID_Tab[13] & 0x0F) << 8;

	    /* Byte 14 */
	    Cid->ManufactDate |= CID_Tab[14];

	    /* Byte 15 */
	    Cid->CID_CRC = (CID_Tab[15] & 0xFE) >> 1;
	    Cid->Reserved2 = 1;
	}

	return status;
}

uint8_t AA_SD_Erase(uint32_t startAddress, uint32_t endAddress)
{
	uint8_t status = AA_ERROR;

	if(AA_SD_SendCmd(SD_CMD_SD_ERASE_GRP_START, startAddress, 0xFF, SD_RESPONSE_NO_ERROR) == AA_OK)
	{
		if(AA_SD_SendCmd(SD_CMD_SD_ERASE_GRP_END, endAddress, 0xFF, SD_RESPONSE_NO_ERROR) == AA_OK)
		{
			if(AA_SD_SendCmd(SD_CMD_ERASE, 0, 0xFF, SD_RESPONSE_NO_ERROR) == AA_OK)
			{
				if(AA_SD_Wait_Response(SD_RESPONSE_NO_ERROR) == AA_OK)
				{
					status = AA_OK;
				}
			}
		}
	}
	return status;
}

uint8_t AA_SD_GetCardInfo(SD_CardInfo *pCardInfo)
{
	uint8_t status = AA_ERROR;
	 AA_SD_GetCSDRegister(&(pCardInfo->Csd));
	 status = AA_SD_GetCIDRegister(&(pCardInfo->Cid));
	 pCardInfo->CardCapacity = (pCardInfo->Csd.DeviceSize + 1) ;
	 pCardInfo->CardCapacity *= (1 << (pCardInfo->Csd.DeviceSizeMul + 2));
	 pCardInfo->CardBlockSize = 1 << (pCardInfo->Csd.RdBlockLen);
	 pCardInfo->CardCapacity *= pCardInfo->CardBlockSize;
	  /* Returns the reponse */
	 return status;
}
uint8_t AA_SD_GetStatus(void)
{
	return AA_OK;
}
void AA_SD_WriteDummy(void)
{
	AA_CS_HIGH();
	AA_SD_WriteByte(dummyByte);
}

uint8_t AA_SD_ReadByte(void)
{
	uint8_t data = 0;
	 uint32_t writevalue = 0xFFFFFFFF;
	HAL_SPI_TransmitReceive(&hspi3,  (uint8_t*) &writevalue,  (uint8_t*) &data, 1, 1000);

	return data;
}

void AA_SD_WriteByte(uint8_t data)
{
	HAL_SPI_Transmit(&hspi3, (uint8_t*) &data, 1, 1000);
}


