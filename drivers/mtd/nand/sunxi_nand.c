/*
 * Copyright (c) 2014-2015, Antmicro Ltd <www.antmicro.com>
 * Copyright (c) 2015, AW-SOM Technologies <www.aw-som.com>
 *
 * SPDX-License-Identifier:     GPL-2.0+
 */

#include <common.h>
#include <config.h>
#include <asm/io.h>
#include <nand.h>
#include "sunxi_nand.h"

/* minimal "boot0" style NAND support for Allwinner A20 */

/* temporary buffer in internal ram */
unsigned char temp_buf[CONFIG_SYS_NAND_PAGE_SIZE]
	__aligned(0x10) __section(".text#");

#define MAX_RETRIES 10

static int check_value_inner(int offset, int expected_bits,
				int max_number_of_retries, int negation)
{
	int retries = 0;
	do {
		int val = readl(offset) & expected_bits;
		if (negation ? !val : val)
			return 1;
		mdelay(1);
		retries++;
	} while (retries < max_number_of_retries);

	return 0;
}

static inline int check_value(int offset, int expected_bits,
				int max_number_of_retries)
{
	return check_value_inner(offset, expected_bits,
					max_number_of_retries, 0);
}

static inline int check_value_negated(int offset, int unexpected_bits,
					int max_number_of_retries)
{
	return check_value_inner(offset, unexpected_bits,
					max_number_of_retries, 1);
}

static void nand_set_clocks(void)
{
	uint32_t val;

	writel(PORTC_PC_CFG0_NRB1 |
		PORTC_PC_CFG0_NRB0 |
		PORTC_PC_CFG0_NRE  |
		PORTC_PC_CFG0_NCE0 |
		PORTC_PC_CFG0_NCE1 |
		PORTC_PC_CFG0_NCLE |
		PORTC_PC_CFG0_NALE |
		PORTC_PC_CFG0_NWE, PORTC_BASE + PORTC_PC_CFG0);

	writel(PORTC_PC_CFG1_NDQ7 |
		PORTC_PC_CFG1_NDQ6 |
		PORTC_PC_CFG1_NDQ5 |
		PORTC_PC_CFG1_NDQ4 |
		PORTC_PC_CFG1_NDQ3 |
		PORTC_PC_CFG1_NDQ2 |
		PORTC_PC_CFG1_NDQ1 |
		PORTC_PC_CFG1_NDQ0, PORTC_BASE + PORTC_PC_CFG1);

	writel(PORTC_PC_CFG2_NCE7 |
		PORTC_PC_CFG2_NCE6 |
		PORTC_PC_CFG2_NCE5 |
		PORTC_PC_CFG2_NCE4 |
		PORTC_PC_CFG2_NCE3 |
		PORTC_PC_CFG2_NCE2 |
		PORTC_PC_CFG2_NWP, PORTC_BASE + PORTC_PC_CFG2);

	writel(PORTC_PC_CFG3_NDQS, PORTC_BASE + PORTC_PC_CFG3);

	val = readl(CCU_BASE + CCU_AHB_GATING_REG0);
	writel(CCU_AHB_GATING_REG0_NAND | val, CCU_BASE + CCU_AHB_GATING_REG0);

	val = readl(CCU_BASE + CCU_NAND_SCLK_CFG_REG);
	writel(val | CCU_NAND_SCLK_CFG_REG_SCLK_GATING
		| CCU_NAND_SCLK_CFG_REG_CLK_DIV_RATIO,
		CCU_BASE + CCU_NAND_SCLK_CFG_REG);
}

void nand_init(void)
{
	uint32_t val;

	nand_set_clocks();
	val = readl(NANDFLASHC_BASE + NANDFLASHC_CTL);
	/* enable and reset CTL */
	writel(val | NFC_CTL_EN | NFC_CTL_RESET,
	       NANDFLASHC_BASE + NANDFLASHC_CTL);

	if (!check_value_negated(NANDFLASHC_BASE + NANDFLASHC_CTL,
				 NFC_CTL_RESET, MAX_RETRIES)) {
		printf("Couldn't initialize nand\n");
	}
}

static uint32_t ecc_errors;

void nand_read_page(unsigned int real_addr, int syndrome)
{
	uint32_t val;
	int ecc_off = 0;
	uint16_t ecc_mode = 0;
	uint16_t rand_seed;
	uint32_t page;
	uint16_t column;
	uint32_t oob_offset;

	switch (SUNXI_ECC_STRENGTH) {
	case 16:
		ecc_mode = 0;
		ecc_off = 0x20;
		break;
	case 24:
		ecc_mode = 1;
		ecc_off = 0x2e;
		break;
	case 28:
		ecc_mode = 2;
		ecc_off = 0x32;
		break;
	case 32:
		ecc_mode = 3;
		ecc_off = 0x3c;
		break;
	case 40:
		ecc_mode = 4;
		ecc_off = 0x4a;
		break;
	case 48:
		ecc_mode = 4;
		ecc_off = 0x52;
		break;
	case 56:
		ecc_mode = 4;
		ecc_off = 0x60;
		break;
	case 60:
		ecc_mode = 4;
		ecc_off = 0x0;
		break;
	case 64:
		ecc_mode = 4;
		ecc_off = 0x0;
		break;
	default:
		ecc_mode = 0;
		ecc_off = 0;
	}

	if (ecc_off == 0) {
		printf("Unsupported ECC strength (%d)!\n",
		       SUNXI_ECC_STRENGTH);
		return;
	}

	/* clear temp_buf */
	memset(temp_buf, 0, CONFIG_SYS_NAND_PAGE_SIZE);

	/* set CMD  */
	writel(NFC_SEND_CMD1 | NFC_WAIT_FLAG | 0xff,
	       NANDFLASHC_BASE + NANDFLASHC_CMD);

	if (!check_value(NANDFLASHC_BASE + NANDFLASHC_ST, (1 << 1),
			 MAX_RETRIES)) {
		printf("Error while initilizing command interrupt\n");
		return;
	}

	page = real_addr / CONFIG_SYS_NAND_BLOCK_SIZE;
	column = real_addr % CONFIG_SYS_NAND_BLOCK_SIZE;

	if (syndrome) {
		/* shift every 1kB in syndrome */
		column += (column / CONFIG_SYS_NAND_PAGE_SIZE) * ecc_off;
	}

	/* clear ecc status */
	writel(0, NANDFLASHC_BASE + NANDFLASHC_ECC_ST);

	/* Choose correct seed */
	if (syndrome)
		rand_seed = random_seed_syndrome;
	else
		rand_seed = random_seed[page % 128];

	writel((rand_seed << 16) | NFC_ECC_RANDOM_EN | NFC_ECC_EN
		| NFC_ECC_PIPELINE | (ecc_mode << 12),
		NANDFLASHC_BASE + NANDFLASHC_ECC_CTL);

	val = readl(NANDFLASHC_BASE + NANDFLASHC_CTL);
	writel(val | NFC_CTL_RAM_METHOD, NANDFLASHC_BASE + NANDFLASHC_CTL);

	if (syndrome) {
		writel(CONFIG_SYS_NAND_PAGE_SIZE,
		       NANDFLASHC_BASE + NANDFLASHC_SPARE_AREA);
	} else {
		oob_offset = CONFIG_SYS_NAND_BLOCK_SIZE
			+ (column / CONFIG_SYS_NAND_PAGE_SIZE) * ecc_off;
		writel(oob_offset, NANDFLASHC_BASE + NANDFLASHC_SPARE_AREA);
	}

	/* DMAC */
	writel(0x0, DMAC_BASE + DMAC_CFG_REG0); /* clr dma cmd */
	/* read from REG_IO_DATA */
	writel(NANDFLASHC_BASE + NANDFLASHC_IO_DATA,
	       DMAC_BASE + DMAC_SRC_START_ADDR_REG0);
	writel((uint32_t)temp_buf,
	       DMAC_BASE + DMAC_DEST_START_ADDRR_REG0); /* read to RAM */
	writel(DMAC_DDMA_PARA_REG_SRC_WAIT_CYC
			| DMAC_DDMA_PARA_REG_SRC_BLK_SIZE,
			DMAC_BASE + DMAC_DDMA_PARA_REG0);
	writel(CONFIG_SYS_NAND_PAGE_SIZE,
	       DMAC_BASE + DMAC_DDMA_BC_REG0); /* 1kB */
	writel(DMAC_DDMA_CFG_REG_LOADING
		| DMAC_DDMA_CFG_REG_DMA_DEST_DATA_WIDTH_32
		| DMAC_DDMA_CFG_REG_DMA_SRC_DATA_WIDTH_32
		| DMAC_DDMA_CFG_REG_DMA_SRC_ADDR_MODE_IO
		| DMAC_DDMA_CFG_REG_DDMA_SRC_DRQ_TYPE_NFC,
		DMAC_BASE + DMAC_CFG_REG0);

	writel((0xE0 << NFC_RANDOM_READ_CMD1_OFFSET)
		| (0x05 << NFC_RANDOM_READ_CMD0_OFFSET)
		| (0x30 | NFC_READ_CMD_OFFSET), NANDFLASHC_BASE
			+ NANDFLASHC_RCMD_SET);
	writel(1, NANDFLASHC_BASE + NANDFLASHC_SECTOR_NUM);
	writel(((page & 0xFFFF) << 16) | column,
	       NANDFLASHC_BASE + NANDFLASHC_ADDR_LOW);
	writel((page >> 16) & 0xFF, NANDFLASHC_BASE + NANDFLASHC_ADDR_HIGH);
	writel(NFC_SEND_CMD1 | NFC_SEND_CMD2 | NFC_DATA_TRANS |
		NFC_PAGE_CMD | NFC_WAIT_FLAG | (4 << NFC_ADDR_NUM_OFFSET) |
		NFC_SEND_ADR | NFC_DATA_SWAP_METHOD | (syndrome ? NFC_SEQ : 0),
		NANDFLASHC_BASE + NANDFLASHC_CMD);

	if (!check_value(NANDFLASHC_BASE + NANDFLASHC_ST, (1 << 2),
			 MAX_RETRIES)) {
		printf("Error while initializing dma interrupt\n");
		return;
	}

	if (!check_value_negated(DMAC_BASE + DMAC_CFG_REG0,
				 DMAC_DDMA_CFG_REG_LOADING, MAX_RETRIES)) {
		printf("Error while waiting for dma transfer to finish\n");
		return;
	}

	if (readl(NANDFLASHC_BASE + NANDFLASHC_ECC_ST))
		ecc_errors++;
}

int nand_spl_load_image(uint32_t offs, unsigned int size, void *dest)
{
	void *current_dest;
	uint32_t count;
	uint32_t current_count;

	memset(dest, 0x0, size); /* clean destination memory */
	ecc_errors = 0;
	for (current_dest = dest;
			current_dest < (dest + size);
			current_dest += CONFIG_SYS_NAND_PAGE_SIZE) {
		nand_read_page(offs, offs < SYNDROME_PARTITIONS_END);
		count = current_dest - dest;

		if (size - count > CONFIG_SYS_NAND_PAGE_SIZE)
			current_count = CONFIG_SYS_NAND_PAGE_SIZE;
		else
			current_count = size - count;

		memcpy(current_dest,
		       temp_buf,
		       current_count);
		offs += CONFIG_SYS_NAND_PAGE_SIZE;
	}
	return ecc_errors;
}

void nand_deselect(void) {}
