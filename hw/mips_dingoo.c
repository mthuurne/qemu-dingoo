/*
 * QEMU Dingoo A320 emulation
 *
 * Copyright (c) 2009 yajin (yajin@vm-kernel.org)
 * Copyright (c) 2010 Maarten ter Huurne (maarten@treewalker.org)
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


#include "hw.h"
#include "pc.h"
#include "fdc.h"
#include "net.h"
#include "boards.h"
#include "smbus.h"
#include "block.h"
#include "flash.h"
#include "mips.h"
#include "pci.h"
#include "qemu-char.h"
#include "sysemu.h"
#include "audio/audio.h"
#include "boards.h"
#include "qemu-log.h"
#include "mips_jz.h"



#define DINGOO_RAM_SIZE       (0x2000000) /*32M */
#define DINGOO_OSC_EXTAL     (12000000)   /*12MHZ */

struct mips_dingoo_s
{
    struct jz_state_s *soc;

    struct nand_bflash_s *nand;
};

static uint32_t dingoo_nand_read8(void *opaque, target_phys_addr_t addr)
{
    struct mips_dingoo_s *s = (struct mips_dingoo_s *) opaque;

    switch (addr) {
        case 0x8000: /*NAND_COMMAND*/
        case 0x10000: /*NAND_ADDRESS*/
            jz4740_badwidth_read8(s,addr);
            break;
        case 0x0: /*NAND_DATA*/
            return nandb_read_data8(s->nand);
            break;
        default:
            jz4740_badwidth_read8(s,addr);
            break;
    }
    return 0;
}

static void dingoo_nand_write8(void *opaque, target_phys_addr_t addr,
                               uint32_t value)
{
    struct mips_dingoo_s *s = (struct mips_dingoo_s *) opaque;

    //printf("write addr %x value %x \n",addr,value);

    switch (addr) {
        case 0x8000: /*NAND_COMMAND*/
            nandb_write_command(s->nand,value);
            break;
        case 0x10000: /*NAND_ADDRESS*/
            nandb_write_address(s->nand,value);
            break;
        case 0x0: /*NAND_DATA*/
            nandb_write_data8(s->nand,value);
            break;
        default:
            jz4740_badwidth_write8(s,addr,value);
            break;
    }
}


CPUReadMemoryFunc *dingoo_nand_readfn[] = {
        dingoo_nand_read8,
        jz4740_badwidth_read16,
        jz4740_badwidth_read32,
};
CPUWriteMemoryFunc *dingoo_nand_writefn[] = {
        dingoo_nand_write8,
        jz4740_badwidth_write16,
        jz4740_badwidth_write32,
};

static void dingoo_nand_setup(struct mips_dingoo_s *s)
{
    int iomemtype;

    /* K9LBG08U0M */
    s->nand = nandb_init(NAND_MFR_SAMSUNG, 0xd7);

    iomemtype = cpu_register_io_memory(dingoo_nand_readfn,
                                       dingoo_nand_writefn, s);
    cpu_register_physical_memory(0x18000000, 0x20000, iomemtype);
}

static int dingoo_nand_read_page(struct mips_dingoo_s *s, uint8_t *buf,
                                 uint16_t page_addr)
{
    uint8_t *p;
    int i;

    p = (uint8_t *)buf;

    /* send command 0x0 */
    dingoo_nand_write8(s, 0x00008000, 0);
    /* send page address */
    dingoo_nand_write8(s, 0x00010000,  page_addr        & 0xff);
    dingoo_nand_write8(s, 0x00010000, (page_addr >>  8) & 0x07);
    dingoo_nand_write8(s, 0x00010000, (page_addr >> 11) & 0xff);
    dingoo_nand_write8(s, 0x00010000, (page_addr >> 19) & 0xff);
    dingoo_nand_write8(s, 0x00010000, (page_addr >> 27) & 0xff);
    /* send command 0x30 */
    dingoo_nand_write8(s, 0x00008000, 0x30);

    for (i = 0; i < 0x800; i++) {
        *p++ = dingoo_nand_read8(s, 0x00000000);
    }
    return 1;
}

/*read the u-boot from NAND Flash into internal RAM*/
static int dingoo_boot_from_nand(struct mips_dingoo_s *s)
{
    uint32_t len;
    uint8_t nand_page[0x800], *load_dest;
    uint32_t nand_pages, i;

    //int fd;

    len = 0x2000; /*8K*/

    /*put the first page into internal ram*/
    load_dest = qemu_get_ram_ptr(0);

    nand_pages = len/0x800;
    //fd = open("u-boot.bin", O_RDWR | O_CREAT);
    for (i = 0; i < nand_pages; i++) {
        dingoo_nand_read_page(s, nand_page, i * 0x800);
        memcpy(load_dest, nand_page, 0x800);
        //write(fd,nand_page,0x800);
        load_dest += 0x800;
    }
    s->soc->env->active_tc.PC = 0x80000004;

    //close(fd);
    return 0;
}


static int dingoo_rom_emu(struct mips_dingoo_s *s)
{
    return dingoo_boot_from_nand(s);
}

static void mips_dingoo_init(ram_addr_t ram_size,
                             const char *boot_device,
                             const char *kernel_filename,
                             const char *kernel_cmdline,
                             const char *initrd_filename,
                             const char *cpu_model)
{
    struct mips_dingoo_s *s = (struct mips_dingoo_s *) qemu_mallocz(sizeof(*s));

    if (ram_size < DINGOO_RAM_SIZE + JZ4740_SRAM_SIZE) {
        fprintf(stderr, "This architecture uses %d bytes of memory\n",
                DINGOO_RAM_SIZE  + JZ4740_SRAM_SIZE);
        exit(1);
    }
    s->soc = jz4740_init(DINGOO_RAM_SIZE, DINGOO_OSC_EXTAL);
    dingoo_nand_setup(s);
    if (dingoo_rom_emu(s) < 0) {
        fprintf(stderr, "boot from nand failed\n");
        exit(-1);
    }

}




QEMUMachine mips_dingoo_machine = {
    .name = "a320",
    .desc = "Dingoo A320 portable game/media player",
    .init = mips_dingoo_init,
};

static void mips_dingoo_machine_init(void)
{
    qemu_register_machine(&mips_dingoo_machine);
}

machine_init(mips_dingoo_machine_init);
