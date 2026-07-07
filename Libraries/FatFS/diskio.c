/*-----------------------------------------------------------------------*/
/* Low level disk I/O module SKELETON for FatFs     (C)ChaN, 2025        */
/*-----------------------------------------------------------------------*/
/* If a working storage control module is available, it should be        */
/* attached to the FatFs via a glue function rather than modifying it.   */
/* This is an example of glue functions to attach various exsisting      */
/* storage control modules to the FatFs module with a defined API.       */
/*-----------------------------------------------------------------------*/

#include "ff.h"			/* Basic definitions of FatFs */
#include "diskio.h"		/* Declarations FatFs MAI */


#include "TFCARD.h"


/* Example: Mapping of physical drive number for each drive */
#define DEV_FLASH	0	/* Map FTL to physical drive 0 */
#define DEV_MMC		1	/* Map MMC/SD card to physical drive 1 */
#define DEV_USB		2	/* Map USB MSD to physical drive 2 */


/*-----------------------------------------------------------------------*/
/* Get Drive Status                                                      */
/*-----------------------------------------------------------------------*/

DSTATUS disk_status (
	BYTE pdrv		/* Physical drive nmuber to identify the drive */
)
{
	return 0;
}



/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */
/*-----------------------------------------------------------------------*/

DSTATUS disk_initialize (
	BYTE pdrv				/* Physical drive nmuber to identify the drive */
)
{
	if(SD_Init() == 0) return 0;
	return STA_NOINIT;
}



/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */
/*-----------------------------------------------------------------------*/

DRESULT disk_read (
	BYTE pdrv,		/* Physical drive nmuber to identify the drive */
	BYTE *buff,		/* Data buffer to store read data */
	LBA_t sector,	/* Start sector in LBA(Logic Block Address) */
	UINT count		/* Number of sectors(扇区，单位：512Byte) to read */
)
{
	for(UINT i = 0; i < count; i++){
		if(SD_ReadBlock(sector + i, buff + i * 512) != 0)
			return RES_ERROR;
	}
	return RES_OK;
}



/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */
/*-----------------------------------------------------------------------*/

#if FF_FS_READONLY == 0

DRESULT disk_write (
	BYTE pdrv,			/* Physical drive nmuber to identify the drive */
	const BYTE *buff,	/* Data to be written */
	LBA_t sector,		/* Start sector in LBA */
	UINT count			/* Number of sectors to write */
)
{
	for(UINT i = 0; i < count; i++){
		if(SD_WriteBlock(sector + i, buff + i * 512) != 0)
			return RES_ERROR;
	}
	return RES_OK;
}

#endif


/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */
/*-----------------------------------------------------------------------*/

DRESULT disk_ioctl (
	BYTE pdrv,		/* Physical drive nmuber (0..) */
	BYTE cmd,		/* Control code */
	void *buff		/* Buffer to send/receive control data */
)
{
	switch (cmd){
		case CTRL_SYNC:		// CTRL_SYNC:"把缓存数据同步写到卡上";本系统用同步写，无缓存，直接返回OK(0)
			return RES_OK;
		case GET_SECTOR_COUNT:
			*(LBA_t*)buff = 0;	// _mkfs（创建文件系统）才需要，f_mount（挂载已有FAT32）不需要。所以暂时填0。
			return RES_OK;
		case GET_SECTOR_SIZE:
			*(WORD*)buff = 512;		// SD卡固定512，返回512。
			return RES_OK;
		case GET_BLOCK_SIZE:		
			*(DWORD*)buff = 1;		// SD卡SPI模式下不关心，填1即可。
			return RES_OK;
	}
	return RES_PARERR;
}

