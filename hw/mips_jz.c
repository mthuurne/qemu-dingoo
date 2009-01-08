/*
 * QEMU JZ Soc emulation
 *
 * Copyright (c) 2008 yajin (yajin@vm-kernel.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


/*
 * The emulation target is pavo demo board.
 *  http://www.ingenic.cn/eng/productServ/kfyd/Hardware/pffaqQuestionContent.aspx?Category=2&Question=3
 *
 */

#include "hw.h"
#include "arm-misc.h"
#include "mips_jz.h"
#include "sysemu.h"
#include "qemu-timer.h"
#include "qemu-char.h"
#include "flash.h"
#include "soc_dma.h"
#include "audio/audio.h"

#define DEBUG_CPM                     (1<<0x1)

#define  DEBUG_FLAG  (DEBUG_CPM)

#ifdef DEBUG

FILE *fp;
static void debug_init()
{
    fp = fopen("jz4740.txt", "w+");
    if (fp == NULL)
    {
        fprintf(stderr, "can not open jz4740.txt \n");
        exit(-1);
    }
}
static void debug_out(uint32_t flag, const char *format, ...)
{
    va_list ap;
    if (fp)
    {
        if (flag & DEBUG_FLAG)
        {
            va_start(ap, format);
            vfprintf(fp, format, ap);
            fflush(fp);
            va_end(ap);
        }
    }
}
#else
static void debug_init()
{
}
static void debug_out(uint32_t flag, const char *format, ...)
{
}
#endif

uint32_t jz4740_badwidth_read8(void *opaque, target_phys_addr_t addr)
{
    uint8_t ret;

    JZ4740_8B_REG(addr);
    cpu_physical_memory_read(addr, (void *) &ret, 1);
    return ret;
}

void jz4740_badwidth_write8(void *opaque, target_phys_addr_t addr,
                            uint32_t value)
{
    uint8_t val8 = value;

    JZ4740_8B_REG(addr);
    cpu_physical_memory_write(addr, (void *) &val8, 1);
}

uint32_t jz4740_badwidth_read16(void *opaque, target_phys_addr_t addr)
{
    uint16_t ret;
    JZ4740_16B_REG(addr);
    cpu_physical_memory_read(addr, (void *) &ret, 2);
    return ret;
}

void jz4740_badwidth_write16(void *opaque, target_phys_addr_t addr,
                             uint32_t value)
{
    uint16_t val16 = value;

    JZ4740_16B_REG(addr);
    cpu_physical_memory_write(addr, (void *) &val16, 2);
}

uint32_t jz4740_badwidth_read32(void *opaque, target_phys_addr_t addr)
{
    uint32_t ret;

    JZ4740_32B_REG(addr);
    cpu_physical_memory_read(addr, (void *) &ret, 4);
    return ret;
}

void jz4740_badwidth_write32(void *opaque, target_phys_addr_t addr,
                             uint32_t value)
{
    JZ4740_32B_REG(addr);
    cpu_physical_memory_write(addr, (void *) &value, 4);
}


/*clock reset and power control*/
struct jz4740_cpm_s
{
    target_phys_addr_t base;
    struct jz_state_s *cpu;

    uint32_t cpccr;
    uint32_t cppcr;
    uint32_t i2scdr;
    uint32_t lpcdr;
    uint32_t msccdr;
    uint32_t uhccdr;
    uint32_t uhctst;
    uint32_t ssicdr;
};

static inline void jz4740_dump_clocks(struct clk *parent)
{
    struct clk *i = parent;

    debug_out(DEBUG_CPM, "clock %x rate 0x%x \n", i->name, i->rate);
    for (i = clk->child1; i; i = i->sibling)
        jz4740_dump_clocks(i);
}

static inline void jz4740_cpccr_update(struct jz4740_cpm_s *s,
                                       uint32_t new_value)
{
    uint32_t ldiv, mdiv, pdiv, hdiv, cdiv, udiv;
    uint32_t div_table[10] = { 1, 2, 3, 4, 6, 8, 12, 16, 24, 32 };

    if (unlikely(new_value == s->cpccr))
        return;

    if (new_value & CPM_CPCCR_PCS)
        jz_clk_setrate(jz_findclk(s->cpu, "pll_divider"), 1, 1);
    else
        jz_clk_setrate(jz_findclk(s->cpu, "pll_divider"), 2, 1);


    ldiv = (new_value & CPM_CPCCR_LDIV_MASK) >> CPM_CPCCR_LDIV_BIT;
    ldiv++;

    mdiv = div_table[(new_value & CPM_CPCCR_MDIV_MASK) >> CPM_CPCCR_MDIV_BIT];
    pdiv = div_table[(new_value & CPM_CPCCR_PDIV_MASK) >> CPM_CPCCR_PDIV_BIT];
    hdiv = div_table[(new_value & CPM_CPCCR_HDIV_MASK) >> CPM_CPCCR_HDIV_BIT];
    cdiv = div_table[(new_value & CPM_CPCCR_CDIV_MASK) >> CPM_CPCCR_CDIV_BIT];
    udiv = div_table[(new_value & CPM_CPCCR_UDIV_MASK) >> CPM_CPCCR_UDIV_BIT];

    jz_clk_setrate(jz_findclk(s->cpu, "ldclk"), ldiv, 1);
    jz_clk_setrate(jz_findclk(s->cpu, "mclk"), mdiv, 1);
    jz_clk_setrate(jz_findclk(s->cpu, "pclk"), pdiv, 1);
    jz_clk_setrate(jz_findclk(s->cpu, "hclk"), hdiv, 1);
    jz_clk_setrate(jz_findclk(s->cpu, "cclk"), cdiv, 1);
    jz_clk_setrate(jz_findclk(s->cpu, "usbclk"), udiv, 1);

    if (new_value & CPM_CPCCR_UCS)
        jz_clk_reparent(jz_findclk(s->cpu, "usbclk"),
                        jz_findclk(s->cpu, "pll_divider"));
    else
        jz_clk_reparent(jz_findclk(s->cpu, "usbclk"),
                        jz_findclk(s->cpu, "osc_extal"));

    if (new_value & CPM_CPCCR_I2CS)
        jz_clk_reparent(jz_findclk(s->cpu, "i2sclk"),
                        jz_findclk(s->cpu, "pll_divider"));
    else
        jz_clk_reparent(jz_findclk(s->cpu, "i2sclk"),
                        jz_findclk(s->cpu, "osc_extal"));

    s->cpccr = new_value;

    debug_out(DEBUG_CPM, "write to cpccr 0x%x\n", new_value);
    jz4740_dump_clocks(jz_findclk(s->cpu, "osc_extal"));

}

static inline void jz4740_cppcr_update(struct jz4740_cpm_s *s,
                                       uint32_t new_value)
{
    uint32_t pllm, plln, pllod, pllbp, pllen, pllst, pllen, pllbp;
    uint32_t pll0[4] = { 1, 2, 2, 4 };


    pllen = new_value & CPM_CPPCR_PLLEN;
    pllbp = new_value & CPM_CPPCR_PLLBP;
    if ((!pllen) || (pllen && pllbp))
    {
        jz_clk_setrate(jz_findclk(s->cpu, "pll_output"), 1, 1);
        debug_out(DEBUG_CPM, "pll is bypassed \n");
        s->cppcr = new_value | CPM_CPPCR_PLLS;
        return;
    }


    pllm = (new_value & CPM_CPPCR_PLLM_MASK) >> CPM_CPPCR_PLLM_BIT;
    plln = (new_value & CPM_CPPCR_PLLN_MASK) >> CPM_CPPCR_PLLN_BIT;
    pllod = (new_value & CPM_CPPCR_PLLOD_MASK) >> CPM_CPPCR_PLLOD_BIT;
    jz_clk_setrate(jz_findclk(s->cpu, "pll_output"), (plln + 2) * pll0[pllod],
                   pllm + 2);

    s->cppcr = new_value;

    debug_out(DEBUG_CPM, "write to cppcr 0x%x\n", new_value);
    jz4740_dump_clocks(jz_findclk(s->cpu, "osc_extal"));

}

static inline void jz4740_i2scdr_update(struct jz4740_cpm_s *s,
                                        uint32_t new_value)
{
    uint32_t i2scdr;

    i2scdr = new_value & CPM_I2SCDR_I2SDIV_MASK;
    if (unlikely(i2scdr == s->i2scdr))
        return;


    jz_clk_setrate(jz_findclk(s->cpu, "i2sclk"), i2scdr + 1, 1);

    s->i2scdr = i2scdr;

    debug_out(DEBUG_CPM, "write to i2scdr 0x%x\n", new_value);
    jz4740_dump_clocks(jz_findclk(s->cpu, "osc_extal"));

}

static inline void jz4740_lpcdr_update(struct jz4740_cpm_s *s,
                                       uint32_t new_value)
{
    uint32_t ipcdr;

    ipcdr = new_value & CPM_LPCDR_PIXDIV_MASK;
    /*TODO: */
    s->lpcdr = ipcdr;
}

static inline void jz4740_msccdr_update(struct jz4740_cpm_s *s,
                                        uint32_t new_value)
{
    uint32_t msccdr;

    msccdr = new_value & CPM_MSCCDR_MSCDIV_MASK;

    if (unlikely(msccdr == s->msccdr))
        return;


    jz_clk_setrate(jz_findclk(s->cpu, "mscclk"), msccdr + 1, 1);

    s->msccdr = msccdr;

    debug_out(DEBUG_CPM, "write to msccdr 0x%x\n", new_value);
    jz4740_dump_clocks(jz_findclk(s->cpu, "osc_extal"));

}

static inline void jz4740_uhccdr_update(struct jz4740_cpm_s *s,
                                        uint32_t new_value)
{
    uint32_t uhccdr;

    uhccdr = new_value & 0xf;
    /*TODO: */
    s->uhccdr = uhccdr;
}

static void jz4740_cpm_write(void *opaque, target_phys_addr_t addr,
                             uint32_t value)
{
    struct jz4740_cpm_s *s = (struct jz4740_cpm_s *) opaque;
    int offset = addr - s->base;

    switch (offset)
    {
    case 0x0:
        jz4740_cpccr_update(s, value);
        break;
    case 0x10:
        jz4740_cppcr_update(s, value);
        break;
    case 0x60:
        jz4740_i2scdr_update(s, value);
        break;
    case 0x64:
        jz4740_lpcdr_update(s, value);
        break;
    case 0x68:
        jz4740_msccdr_update(s, value);
        break;
    case 0x6c:
        jz4740_uhccdr_update(s, value);
        break;
    case 0x70:
        s->uhctst = value & 0x3f;
        break;
    case 0x74:
        s->ssicdr = value & 0xf;
        break;
    default:
        cpu_abort(s->cpu->env,
                  "jz4740_cpm_write undefined addr " JZ_FMT_plx "  value %x \n",
                  addr, value);
    }

}


static uint32_t jz474_cpm_read(void *opaque, target_phys_addr_t addr)
{
    struct jz4740_cpm_s *s = (struct jz4740_cpm_s *) opaque;
    int offset = addr - s->base;

    switch (offset)
    {
    case 0x0:
        return s->cpccr;
    case 0x10:
        return s->cppcr;
    case 0x60:
        return s->i2scdr;
    case 0x64:
        return s->lpcdr;
    case 0x68:
        return s->msccdr;
    case 0x6c:
        return s->uhccdr;
    case 0x70:
        return s->uhctst;
    case 0x74:
        return s->ssicdr;
    default:
        cpu_abort(s->cpu->env,
                  "jz474_cpm_read undefined addr " JZ_FMT_plx "  \n", addr);
    }

}



static CPUReadMemoryFunc *jz4740_cpm_readfn[] = {
    jz4740_badwidth_read32,
    jz4740_badwidth_read32,
    jz474_cpm_read,
};

static CPUWriteMemoryFunc *jz4740_cpm_writefn[] = {
    jz4740_badwidth_write32,
    jz4740_badwidth_write32,
    jz4740_cpm_write,
};

static void jz4740_cpm_reset(struct jz4740_cpm_s *s)
{
    s->cpccr = 0x42040000;
    s->cppcr = 0x28080011;
    s->i2scdr = 0x00000004;
    s->lpcdr = 0x00000004;
    s->msccdr = 0x00000004;
    s->uhccdr = 0x00000004;
    s->uhctst = 0x0;
    s->ssicdr = 0x00000004;
}

struct jz4740_cpm_s *jz4740_cpm_init(struct jz_state_s *cpu)
{
    int iomemtype;
    struct jz4740_cpm_s *s = (struct jz4740_cpm_s *) qemu_mallocz(sizeof(*s));
    s->base = JZ4740_PHYS_BASE(JZ4740_CPM_BASE);

    jz4740_cpm_reset(s);

    iomemtype =
        cpu_register_io_memory(0, jz4740_cpm_readfn, jz4740_cpm_writefn, s);
    cpu_register_physical_memory(s->base, 0x00001000, iomemtype);
}


struct jz_state_s *jz4740_init(unsigned long sdram_size,
                               DisplayState * ds, const char *core,
                               uint32_t osc_extal_freq)
{
    struct jz_state_s *s = (struct jz_state_s *)
        qemu_mallocz(sizeof(struct jz_state_s));
    ram_addr_t sram_base, sdram_base;

    s->mpu_model = jz4740;
    s->env = cpu_init("jz4740");
    if (!s->env)
    {
        fprintf(stderr, "Unable to find CPU definition\n");
        exit(1);
    }
    qemu_register_reset(main_cpu_reset, env);

    s->sdram_size = sdram_size;
    s->sram_size = JZ4740_SRAM_SIZE;

    /* Clocks */
    jz_clk_init(s, osc_extal_freq);

    /*map sram to 0x80000000 and sdram to 0x80004000 */
    sram_base = qemu_ram_alloc(s->sram_size);
    cpu_register_physical_memory(JZ4740_SRAM_BASE, s->sram_size,
                                 (sram_base | IO_MEM_RAM));
    sdram_base = qemu_ram_alloc(s->sdram_size);
    cpu_register_physical_memory(JZ4740_SDRAM_BASE, s->sdram_size,
                                 (sdram_base | IO_MEM_RAM));


}
