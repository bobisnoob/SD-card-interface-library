/*
 * sdcard.h
 *
 *  Created on: Apr 8, 2017
 *      Author: Abdel Alkuor
 */
#include "stm32f3xx_hal.h"

#ifndef SDCARD_H_
#define SDCARD_H_


extern SPI_HandleTypeDef hspi3;

#define SD_OK					0x00
#define SD_ERROR				0x01
#define SD_NO_RESPONSE_EXPECTED 0x80
typedef enum
{
  AA_OK       = 0x00,
  AA_ERROR    = 0x01,
  AA_BUSY     = 0x02,
  AA_TIMEOUT  = 0x03
} AA_Status;


/*@ brief SD responses and error flags */
typedef enum
{
	SD_RESPONSE_NO_ERROR 	= 0x00,
	SD_IDLE_STATE			= 0x01,
	SD_ERASE_RESET			= 0x02,
	SD_ILLEAGLE_CMD			= 0x04,
	SD_CRC_ERROR			= 0x08,
	SD_ERASE_SEQUENCE_ERROR	= 0x10,
	SD_ADDRESS_ERROR		= 0x20,
	SD_PARAMETER_ERROR		= 0x40,
	SD_RESPONSE_FAILURE		= 0xFF,
	SD_DATA_OK				= 0x05,
	SD_DATA_CRC_ERROR		= 0x0B,
	SD_DATA_WRTIE_ERROR		= 0x0D,
	SD_DATA_OTHER_ERROR		= 0xFF,
	SD_VOLTAGE				= 0xAA
}AA_SD_RESPONSES;


/* @ brief Card specific data: CSD register */
typedef struct
{
  __IO uint8_t  CSDStruct;            /* CSD structure */
  __IO uint8_t  SysSpecVersion;       /* System specification version */
  __IO uint8_t  Reserved1;            /* Reserved */
  __IO uint8_t  TAAC;                 /* Data read access-time 1 */
  __IO uint8_t  NSAC;                 /* Data read access-time 2 in CLK cycles */
  __IO uint8_t  MaxBusClkFrec;        /* Max. bus clock frequency */
  __IO uint16_t CardComdClasses;      /* Card command classes */
  __IO uint8_t  RdBlockLen;           /* Max. read data block length */
  __IO uint8_t  PartBlockRead;        /* Partial blocks for read allowed */
  __IO uint8_t  WrBlockMisalign;      /* Write block misalignment */
  __IO uint8_t  RdBlockMisalign;      /* Read block misalignment */
  __IO uint8_t  DSRImpl;              /* DSR implemented */
  __IO uint8_t  Reserved2;            /* Reserved */
  __IO uint32_t DeviceSize;           /* Device Size */
  __IO uint8_t  MaxRdCurrentVDDMin;   /* Max. read current @ VDD min */
  __IO uint8_t  MaxRdCurrentVDDMax;   /* Max. read current @ VDD max */
  __IO uint8_t  MaxWrCurrentVDDMin;   /* Max. write current @ VDD min */
  __IO uint8_t  MaxWrCurrentVDDMax;   /* Max. write current @ VDD max */
  __IO uint8_t  DeviceSizeMul;        /* Device size multiplier */
  __IO uint8_t  EraseGrSize;          /* Erase group size */
  __IO uint8_t  EraseGrMul;           /* Erase group size multiplier */
  __IO uint8_t  WrProtectGrSize;      /* Write protect group size */
  __IO uint8_t  WrProtectGrEnable;    /* Write protect group enable */
  __IO uint8_t  ManDeflECC;           /* Manufacturer default ECC */
  __IO uint8_t  WrSpeedFact;          /* Write speed factor */
  __IO uint8_t  MaxWrBlockLen;        /* Max. write data block length */
  __IO uint8_t  WriteBlockPaPartial;  /* Partial blocks for write allowed */
  __IO uint8_t  Reserved3;            /* Reserded */
  __IO uint8_t  ContentProtectAppli;  /* Content protection application */
  __IO uint8_t  FileFormatGrouop;     /* File format group */
  __IO uint8_t  CopyFlag;             /* Copy flag (OTP) */
  __IO uint8_t  PermWrProtect;        /* Permanent write protection */
  __IO uint8_t  TempWrProtect;        /* Temporary write protection */
  __IO uint8_t  FileFormat;           /* File Format */
  __IO uint8_t  ECC;                  /* ECC code */
  __IO uint8_t  CSD_CRC;              /* CSD CRC */
  __IO uint8_t  Reserved4;            /* always 1*/
} SD_CSD;


/*   @brief  Card Identification Data: CID Register  */

typedef struct
{
  __IO uint8_t  ManufacturerID;       /* ManufacturerID */
  __IO uint16_t OEM_AppliID;          /* OEM/Application ID */
  __IO uint32_t ProdName1;            /* Product Name part1 */
  __IO uint8_t  ProdName2;            /* Product Name part2*/
  __IO uint8_t  ProdRev;              /* Product Revision */
  __IO uint32_t ProdSN;               /* Product Serial Number */
  __IO uint8_t  Reserved1;            /* Reserved1 */
  __IO uint16_t ManufactDate;         /* Manufacturing Date */
  __IO uint8_t  CID_CRC;              /* CID CRC */
  __IO uint8_t  Reserved2;            /* always 1 */
} SD_CID;

/*  @brief SD Card information */
typedef struct
{
  SD_CSD Csd;
  SD_CID Cid;
  uint32_t CardCapacity;  /* Card Capacity */
  uint32_t CardBlockSize; /* Card Block Size */
} SD_CardInfo;


/* start data tokens */

#define SD_START_DATA_SINGLE_BLOCK_READ		0xFE
#define SD_START_DATA_MULTIPLE_BLOCK_READ	0xFE
#define SD_START_DATA_SIGNEL_BLOCK_WRITE	0xFE
#define SD_START_DATA_MULTIPLE_BLOCK_WRITE	0xFD
#define SD_STOP_DATA_MULTIPLE_BLOCK_WRITE	0xFD


/* define SDcard commands */
/* IMPORTANT NOTE: ALL COMMANDS SHOULD BE ORED WITH 0x40 */

#define SD_CMD_IDLE					  0
#define SD_CMD_SEND_OP_COND			  1
#define SD_CMD_SEND_IF_COND			  8
#define SD_CMD_SEND_CSD				  9
#define SD_CMD_SEND_CID				  10
#define	SD_CMD_STOP_TRANSMISSION	  12
#define SD_CMD_SEND_STATUS			  13
#define SD_CMD_SET_BLOCKLEN           16
#define SD_CMD_READ_SINGLE_BLOCK      17
#define SD_CMD_READ_MULT_BLOCK        18
#define SD_CMD_SET_BLOCK_COUNT        23
#define SD_CMD_WRITE_SINGLE_BLOCK     24
#define SD_CMD_WRITE_MULT_BLOCK       25
#define SD_CMD_PROG_CSD               27
#define SD_CMD_SET_WRITE_PROT         28
#define SD_CMD_CLR_WRITE_PROT         29
#define SD_CMD_SEND_WRITE_PROT        30
#define SD_CMD_SD_ERASE_GRP_START     32
#define SD_CMD_SD_ERASE_GRP_END       33
#define SD_CMD_UNTAG_SECTOR           34
#define SD_CMD_ERASE_GRP_START        35
#define SD_CMD_ERASE_GRP_END          36
#define SD_CMD_UNTAG_ERASE_GROUP      37
#define SD_CMD_ERASE                  38
#define SD_CMD_APP_SEND_OP_COND		  41
#define SD_CMD_APP_CMD 				  55



#define AA_CS_LOW()		HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,RESET);
#define AA_CS_HIGH()	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0,SET);

/* SD card functions prototypes */

uint8_t AA_SDcard_Init(void);
AA_Status AA_SDcard_WriteCmd(uint8_t cmd, uint32_t argument, uint8_t crc, uint8_t response);
AA_Status AA_SD_Wait_Response(uint8_t response);
uint8_t AA_SD_SendCmd(uint8_t cmd, uint32_t argument, uint8_t crc, uint8_t response);
void AA_SD_WriteDummy(void);
uint8_t AA_SDcard_IdleState(void);
uint8_t AA_SD_ReadByte(void);
AA_SD_RESPONSES AA_SD_GetResponse(void);
void AA_SD_WriteByte(uint8_t data);
uint8_t AA_SD_GetCIDRegister(SD_CID *Cid);
uint8_t AA_SD_GetCSDRegister(SD_CSD *Csd);
uint8_t AA_SD_GetStatus(void);
uint8_t AA_SD_GetCardInfo(SD_CardInfo *pCardInfo);
uint8_t AA_SD_WriteBlocks(uint8_t *data,uint64_t dataAddress, uint16_t blockSize,uint32_t numberOfBlocks);
uint8_t AA_SDcard_ReadBlocks(uint8_t *data,uint64_t dataAddress, uint16_t blockSize,uint32_t numberOfBlocks);

#endif /* SDCARD_H_ */
