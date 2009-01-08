/*
 * ingenic JZ Soc
 *
 * Copyright (C) 2009 yajin<yajin@vm-kernel.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */
#ifndef _MIPS_JZ_H_
#define _MIPS_JZ_H_






#define JZ4740_SRAM_SIZE	0x4000
#define JZ4740_SRAM_BASE 0x80000000
#define JZ4740_SDRAM_BASE 0x80004000

#define JZ4740_PHYS_BASE(a) ((a)-0xa0000000)

#define	JZ4740_CPM_BASE	0xB0000000
#define	JZ4740_INTC_BASE	0xB0001000
#define	JZ4740_TCU_BASE	0xB0002000
#define	JZ4740_WDT_BASE	0xB0002000
#define	JZ4740_RTC_BASE	0xB0003000
#define	JZ4740_GPIO_BASE	0xB0010000
#define	JZ4740_AIC_BASE	0xB0020000
#define	JZ4740_ICDC_BASE	0xB0020000
#define	JZ4740_MSC_BASE	0xB0021000
#define	JZ4740_UART0_BASE	0xB0030000
#define	JZ4740_I2C_BASE	0xB0042000
#define	JZ4740_SSI_BASE	0xB0043000
#define	JZ4740_SADC_BASE	0xB0070000
#define	JZ4740_EMC_BASE	0xB3010000
#define	JZ4740_DMAC_BASE	0xB3020000
#define	JZ4740_UHC_BASE	0xB3030000
#define	JZ4740_UDC_BASE	0xB3040000
#define	JZ4740_LCD_BASE	0xB3050000
#define	JZ4740_SLCD_BASE	0xB3050000
#define	JZ4740_CIM_BASE	0xB3060000
#define	JZ4740_ETH_BASE	0xB3100000

/* Clock Control Register */
#define CPM_CPCCR_I2CS		(1 << 31)
#define CPM_CPCCR_CLKOEN	(1 << 30)
#define CPM_CPCCR_UCS		(1 << 29)
#define CPM_CPCCR_UDIV_BIT	23
#define CPM_CPCCR_UDIV_MASK	(0x3f << CPM_CPCCR_UDIV_BIT)
#define CPM_CPCCR_CE		(1 << 22)
#define CPM_CPCCR_PCS		(1 << 21)
#define CPM_CPCCR_LDIV_BIT	16
#define CPM_CPCCR_LDIV_MASK	(0x1f << CPM_CPCCR_LDIV_BIT)
#define CPM_CPCCR_MDIV_BIT	12
#define CPM_CPCCR_MDIV_MASK	(0x0f << CPM_CPCCR_MDIV_BIT)
#define CPM_CPCCR_PDIV_BIT	8
#define CPM_CPCCR_PDIV_MASK	(0x0f << CPM_CPCCR_PDIV_BIT)
#define CPM_CPCCR_HDIV_BIT	4
#define CPM_CPCCR_HDIV_MASK	(0x0f << CPM_CPCCR_HDIV_BIT)
#define CPM_CPCCR_CDIV_BIT	0
#define CPM_CPCCR_CDIV_MASK	(0x0f << CPM_CPCCR_CDIV_BIT)


/* I2S Clock Divider Register */
#define CPM_I2SCDR_I2SDIV_BIT	0
#define CPM_I2SCDR_I2SDIV_MASK	(0x1ff << CPM_I2SCDR_I2SDIV_BIT)

/* LCD Pixel Clock Divider Register */
#define CPM_LPCDR_PIXDIV_BIT	0
#define CPM_LPCDR_PIXDIV_MASK	(0x1ff << CPM_LPCDR_PIXDIV_BIT)

/* MSC Clock Divider Register */
#define CPM_MSCCDR_MSCDIV_BIT	0
#define CPM_MSCCDR_MSCDIV_MASK	(0x1f << CPM_MSCCDR_MSCDIV_BIT)

/* PLL Control Register */
#define CPM_CPPCR_PLLM_BIT	23
#define CPM_CPPCR_PLLM_MASK	(0x1ff << CPM_CPPCR_PLLM_BIT)
#define CPM_CPPCR_PLLN_BIT	18
#define CPM_CPPCR_PLLN_MASK	(0x1f << CPM_CPPCR_PLLN_BIT)
#define CPM_CPPCR_PLLOD_BIT	16
#define CPM_CPPCR_PLLOD_MASK	(0x03 << CPM_CPPCR_PLLOD_BIT)
#define CPM_CPPCR_PLLS		(1 << 10)
#define CPM_CPPCR_PLLBP		(1 << 9)
#define CPM_CPPCR_PLLEN		(1 << 8)
#define CPM_CPPCR_PLLST_BIT	0
#define CPM_CPPCR_PLLST_MASK	(0xff << CPM_CPPCR_PLLST_BIT)





#if TARGET_PHYS_ADDR_BITS == 32
#define JZ_FMT_plx "%#08x"
#elif TARGET_PHYS_ADDR_BITS == 64
#define JZ_FMT_plx "%#08" PRIx64
#else
#error TARGET_PHYS_ADDR_BITS undefined
#endif


#define TCMI_VERBOSE			1

#ifdef TCMI_VERBOSE
#define JZ4740_8B_REG(paddr)		\
        fprintf(stderr, "%s: 8-bit register " JZ_FMT_plx "\n",	\
                        __FUNCTION__, paddr)
#define JZ4740_16B_REG(paddr)		\
        fprintf(stderr, "%s: 16-bit register " JZ_FMT_plx "\n",	\
                        __FUNCTION__, paddr)
#define JZ4740_32B_REG(paddr)		\
        fprintf(stderr, "%s: 32-bit register " JZ_FMT_plx "\n",	\
                        __FUNCTION__, paddr)
# else
#define JZ4740_8B_REG(paddr)
#define JZ4740_16B_REG(paddr)
#define JZ4740_32B_REG(paddr)
#endif



/*forward define*/
struct jz_state_s;


/*mips_jz_clk.c*/
typedef struct clk *jz_clk;
void jz_clk_init(struct jz_state_s *mpu,uint32_t osc_extal_freq);
jz_clk jz_findclk(struct jz_state_s *mpu, const char *name);
void jz_clk_get(jz_clk clk);
void jz_clk_put(jz_clk clk);
void jz_clk_onoff(jz_clk clk, int on);
void jz_clk_canidle(jz_clk clk, int can);
void jz_clk_setrate(jz_clk clk, int divide, int multiply);
int64_t jz_clk_getrate(jz_clk clk);
void jz_clk_reparent(jz_clk clk, jz_clk parent);





enum jz_cpu_model {
        jz4740,
        jz4730,
        jz4750
    };
#define cpu_is_jz4730(cpu)		(cpu->mpu_model == jz4730)
#define cpu_is_jz4740(cpu)		(cpu->mpu_model == jz4740)
#define cpu_is_jz4750(cpu)		(cpu->mpu_model == jz4750)

struct jz_state_s {
 	enum jz_cpu_model mpu_model;
	CPUState *env;
	unsigned long sdram_size;
    unsigned long sram_size;

	
	jz_clk clks;
};

#endif
