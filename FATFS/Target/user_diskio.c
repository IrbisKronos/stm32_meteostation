/* USER CODE BEGIN Header */
/**
 ******************************************************************************
  * @file    user_diskio.c
  * @brief   This file includes a diskio driver skeleton to be completed by the user.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
 /* USER CODE END Header */

#ifdef USE_OBSOLETE_USER_CODE_SECTION_0
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */
#endif

/* USER CODE BEGIN DECL */

#include <string.h>
#include "ff_gen_drv.h"
#include "main.h"

/* SD card type flags */
#define CT_MMC      0x01
#define CT_SD1      0x02
#define CT_SD2      0x04
#define CT_SDC      (CT_SD1 | CT_SD2)
#define CT_BLOCK    0x08

/* SD commands */
#define CMD0    (0)
#define CMD1    (1)
#define ACMD41  (0x80 + 41)
#define CMD8    (8)
#define CMD9    (9)
#define CMD12   (12)
#define CMD16   (16)
#define CMD17   (17)
#define CMD18   (18)
#define ACMD23  (0x80 + 23)
#define CMD24   (24)
#define CMD25   (25)
#define CMD32   (32)
#define CMD33   (33)
#define CMD38   (38)
#define CMD55   (55)
#define CMD58   (58)

static volatile DSTATUS Stat = STA_NOINIT;
static uint8_t CardType;

extern SPI_HandleTypeDef hspi1;

/* ---- CS control ---- */
static inline void CS_Low(void)  { HAL_GPIO_WritePin(CD_CS_GPIO_Port, CD_CS_Pin, GPIO_PIN_RESET); }
static inline void CS_High(void) { HAL_GPIO_WritePin(CD_CS_GPIO_Port, CD_CS_Pin, GPIO_PIN_SET); }

/* ---- SPI helpers ---- */
static uint8_t SPI_TxRx(uint8_t data)
{
    uint8_t rx;
    HAL_SPI_TransmitReceive(&hspi1, &data, &rx, 1, 100);
    return rx;
}

static int wait_ready(uint32_t timeout_ms)
{
    uint32_t start = HAL_GetTick();
    while (SPI_TxRx(0xFF) != 0xFF) {
        if ((HAL_GetTick() - start) >= timeout_ms) return 0;
    }
    return 1;
}

static void sd_deselect(void)
{
    CS_High();
    SPI_TxRx(0xFF);    /* extra clock to release MISO */
}

static int sd_select(void)
{
    CS_Low();
    SPI_TxRx(0xFF);
    if (wait_ready(500)) return 1;
    sd_deselect();
    return 0;
}

/* ---- SPI speed switch ---- */
static void SPI_SetSpeed(uint32_t prescaler)
{
    __HAL_SPI_DISABLE(&hspi1);
    hspi1.Instance->CR1 = (hspi1.Instance->CR1 & ~SPI_CR1_BR) | prescaler;
    __HAL_SPI_ENABLE(&hspi1);
}

/* ---- SD command ---- */
static uint8_t send_cmd(uint8_t cmd, uint32_t arg)
{
    uint8_t crc, res;

    if (cmd & 0x80) {           /* ACMD = CMD55 prefix */
        cmd &= 0x7F;
        res = send_cmd(CMD55, 0);
        if (res > 1) return res;
    }

    sd_deselect();
    if (!sd_select()) return 0xFF;

    SPI_TxRx(0x40 | cmd);
    SPI_TxRx((uint8_t)(arg >> 24));
    SPI_TxRx((uint8_t)(arg >> 16));
    SPI_TxRx((uint8_t)(arg >> 8));
    SPI_TxRx((uint8_t)(arg));

    crc = 0x01;
    if (cmd == CMD0) crc = 0x95;
    if (cmd == CMD8) crc = 0x87;
    SPI_TxRx(crc);

    if (cmd == CMD12) SPI_TxRx(0xFF);  /* skip stuff byte */

    uint8_t n = 10;
    do { res = SPI_TxRx(0xFF); } while ((res & 0x80) && --n);
    return res;
}

/* ---- Read one 512-byte data block ---- */
static int rcvr_datablock(uint8_t *buff, UINT btr)
{
    uint8_t token;
    uint32_t start = HAL_GetTick();
    do {
        token = SPI_TxRx(0xFF);
    } while ((token == 0xFF) && (HAL_GetTick() - start) < 200);

    if (token != 0xFE) return 0;

    HAL_SPI_Receive(&hspi1, buff, btr, 200);
    SPI_TxRx(0xFF);     /* CRC (ignored) */
    SPI_TxRx(0xFF);
    return 1;
}

/* ---- Write one 512-byte data block ---- */
static int xmit_datablock(const uint8_t *buff, uint8_t token)
{
    if (!wait_ready(500)) return 0;

    SPI_TxRx(token);
    if (token != 0xFD) {    /* not Stop token */
        HAL_SPI_Transmit(&hspi1, (uint8_t *)buff, 512, 200);
        SPI_TxRx(0xFF);     /* CRC dummy */
        SPI_TxRx(0xFF);
        uint8_t resp = SPI_TxRx(0xFF);
        if ((resp & 0x1F) != 0x05) return 0;   /* write error */
    }
    return 1;
}

/* USER CODE END DECL */

/* Private function prototypes -----------------------------------------------*/
DSTATUS USER_initialize (BYTE pdrv);
DSTATUS USER_status (BYTE pdrv);
DRESULT USER_read (BYTE pdrv, BYTE *buff, DWORD sector, UINT count);
#if _USE_WRITE == 1
  DRESULT USER_write (BYTE pdrv, const BYTE *buff, DWORD sector, UINT count);
#endif /* _USE_WRITE == 1 */
#if _USE_IOCTL == 1
  DRESULT USER_ioctl (BYTE pdrv, BYTE cmd, void *buff);
#endif /* _USE_IOCTL == 1 */

Diskio_drvTypeDef  USER_Driver =
{
  USER_initialize,
  USER_status,
  USER_read,
#if  _USE_WRITE
  USER_write,
#endif  /* _USE_WRITE == 1 */
#if  _USE_IOCTL == 1
  USER_ioctl,
#endif /* _USE_IOCTL == 1 */
};

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Initializes a Drive
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_initialize (
	BYTE pdrv           /* Physical drive nmuber to identify the drive */
)
{
  /* USER CODE BEGIN INIT */
    uint8_t n, cmd, ty, ocr[4];

    if (pdrv) return STA_NOINIT;

    /* Power-up sequence: ≥74 clocks with CS high */
    CS_High();
    for (n = 10; n; n--) SPI_TxRx(0xFF);

    ty = 0;
    if (send_cmd(CMD0, 0) == 1) {           /* Enter Idle state */
        uint32_t start = HAL_GetTick();

        if (send_cmd(CMD8, 0x1AA) == 1) {   /* SD v2 */
            for (n = 0; n < 4; n++) ocr[n] = SPI_TxRx(0xFF);
            if (ocr[2] == 0x01 && ocr[3] == 0xAA) {
                while ((HAL_GetTick() - start) < 1000 && send_cmd(ACMD41, 1UL << 30));
                if ((HAL_GetTick() - start) < 1000 && send_cmd(CMD58, 0) == 0) {
                    for (n = 0; n < 4; n++) ocr[n] = SPI_TxRx(0xFF);
                    ty = (ocr[0] & 0x40) ? CT_SD2 | CT_BLOCK : CT_SD2;
                }
            }
        } else {                            /* SD v1 or MMC */
            if (send_cmd(ACMD41, 0) <= 1) {
                ty = CT_SD1; cmd = ACMD41;
            } else {
                ty = CT_MMC; cmd = CMD1;
            }
            while ((HAL_GetTick() - start) < 1000 && send_cmd(cmd, 0));
            if ((HAL_GetTick() - start) >= 1000 || send_cmd(CMD16, 512) != 0)
                ty = 0;
        }
    }

    CardType = ty;
    sd_deselect();

    if (ty) {
        /* Switch SPI to full speed: prescaler 4 → 25 MHz */
        SPI_SetSpeed(SPI_CR1_BR_0);
        Stat &= ~STA_NOINIT;
    }

    return Stat;
  /* USER CODE END INIT */
}

/**
  * @brief  Gets Disk Status
  * @param  pdrv: Physical drive number (0..)
  * @retval DSTATUS: Operation status
  */
DSTATUS USER_status (
	BYTE pdrv       /* Physical drive number to identify the drive */
)
{
  /* USER CODE BEGIN STATUS */
    if (pdrv) return STA_NOINIT;
    return Stat;
  /* USER CODE END STATUS */
}

/**
  * @brief  Reads Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data buffer to store read data
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to read (1..128)
  * @retval DRESULT: Operation result
  */
DRESULT USER_read (
	BYTE pdrv,      /* Physical drive nmuber to identify the drive */
	BYTE *buff,     /* Data buffer to store read data */
	DWORD sector,   /* Sector address in LBA */
	UINT count      /* Number of sectors to read */
)
{
  /* USER CODE BEGIN READ */
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    if (!(CardType & CT_BLOCK)) sector *= 512;  /* byte address for non-SDHC */

    if (count == 1) {
        if (send_cmd(CMD17, sector) == 0 && rcvr_datablock(buff, 512))
            count = 0;
    } else {
        if (send_cmd(CMD18, sector) == 0) {
            do {
                if (!rcvr_datablock(buff, 512)) break;
                buff += 512;
            } while (--count);
            send_cmd(CMD12, 0);
        }
    }

    sd_deselect();
    return count ? RES_ERROR : RES_OK;
  /* USER CODE END READ */
}

/**
  * @brief  Writes Sector(s)
  * @param  pdrv: Physical drive number (0..)
  * @param  *buff: Data to be written
  * @param  sector: Sector address (LBA)
  * @param  count: Number of sectors to write (1..128)
  * @retval DRESULT: Operation result
  */
#if _USE_WRITE == 1
DRESULT USER_write (
	BYTE pdrv,          /* Physical drive nmuber to identify the drive */
	const BYTE *buff,   /* Data to be written */
	DWORD sector,       /* Sector address in LBA */
	UINT count          /* Number of sectors to write */
)
{
  /* USER CODE BEGIN WRITE */
    if (pdrv || !count) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;
    if (Stat & STA_PROTECT) return RES_WRPRT;

    if (!(CardType & CT_BLOCK)) sector *= 512;

    if (count == 1) {
        if (send_cmd(CMD24, sector) == 0 && xmit_datablock(buff, 0xFE))
            count = 0;
    } else {
        if (CardType & CT_SDC) send_cmd(ACMD23, count);
        if (send_cmd(CMD25, sector) == 0) {
            do {
                if (!xmit_datablock(buff, 0xFC)) break;
                buff += 512;
            } while (--count);
            if (!xmit_datablock(buff, 0xFD)) count = 1;    /* Stop token */
        }
    }

    sd_deselect();
    return count ? RES_ERROR : RES_OK;
  /* USER CODE END WRITE */
}
#endif /* _USE_WRITE == 1 */

/**
  * @brief  I/O control operation
  * @param  pdrv: Physical drive number (0..)
  * @param  cmd: Control code
  * @param  *buff: Buffer to send/receive control data
  * @retval DRESULT: Operation result
  */
#if _USE_IOCTL == 1
DRESULT USER_ioctl (
	BYTE pdrv,      /* Physical drive nmuber (0..) */
	BYTE cmd,       /* Control code */
	void *buff      /* Buffer to send/receive control data */
)
{
  /* USER CODE BEGIN IOCTL */
    DRESULT res = RES_ERROR;
    uint8_t n, csd[16];
    DWORD csize;

    if (pdrv) return RES_PARERR;
    if (Stat & STA_NOINIT) return RES_NOTRDY;

    switch (cmd) {
    case CTRL_SYNC:
        if (sd_select()) res = RES_OK;
        break;

    case GET_SECTOR_COUNT:
        if (send_cmd(CMD9, 0) == 0 && rcvr_datablock(csd, 16)) {
            if ((csd[0] >> 6) == 1) {   /* SDv2 */
                csize = csd[9] + ((uint16_t)csd[8] << 8)
                      + ((uint32_t)(csd[7] & 63) << 16) + 1;
                *(DWORD *)buff = csize << 10;
            } else {                    /* SDv1 / MMC */
                n = (csd[5] & 15) + ((csd[10] & 128) >> 7)
                  + ((csd[9] & 3) << 1) + 2;
                csize = (csd[8] >> 6) + ((uint16_t)csd[7] << 2)
                      + ((uint16_t)(csd[6] & 3) << 10) + 1;
                *(DWORD *)buff = csize << (n - 9);
            }
            res = RES_OK;
        }
        break;

    case GET_SECTOR_SIZE:
        *(WORD *)buff = 512;
        res = RES_OK;
        break;

    case GET_BLOCK_SIZE:
        *(DWORD *)buff = 128;
        res = RES_OK;
        break;

    default:
        res = RES_PARERR;
    }

    sd_deselect();
    return res;
  /* USER CODE END IOCTL */
}
#endif /* _USE_IOCTL == 1 */
