/*
 * QEMU pavo demo board emulation
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

#include "mips_jz.h"
#include "qemu-common.h"
#include "sysemu.h"
#include "arm-misc.h"
#include "irq.h"
#include "console.h"
#include "boards.h"
#include "i2c.h"
#include "devices.h"
#include "flash.h"
#include "hw.h"



#define PAVO_RAM_SIZE       (0x4000000) /*64M */


/* pavo board support */
struct mips_pavo_s
{
    struct jz_state_s *cpu;
};





static
    void mips_pavo_init(ram_addr_t ram_size, int vga_ram_size,
                        const char *boot_device, DisplayState * ds,
                        const char *kernel_filename, const char *kernel_cmdline,
                        const char *initrd_filename, const char *cpu_model)
{
    struct mips_pavo_s *s = (struct mips_pavo_s *) qemu_mallocz(sizeof(*s));

    if (ram_size < PAVO_RAM_SIZE + JZ4740_SRAM_SIZE)
    {
        fprintf(stderr, "This architecture uses %i bytes of memory\n",
                PAVO_RAM_SIZE + JZ4740_SRAM_SIZE);
        exit(1);
    }
    s->cpu = jz4740_init(PAVO_RAM_SIZE, NULL, NULL);

}




QEMUMachine mips_pavo_machine = {
    .name = "pavo",
    .desc = "JZ Pavo demo board",
    .init = mips_pavo_init,
    .ram_require = (JZ4740_SRAM_SIZE + PAVO_RAM_SIZE) | RAMSIZE_FIXED,
    .nodisk_ok = 1,
};


