/*
 * byt_plat_clock.c - VLV2 platform clock driver
 * Copyright (C) 2013 Intel Corporation
 *
 * Author: Asutosh Pathak <asutosh.pathak@intel.com>
 * Author: Chandra Sekhar Anagani <chandra.sekhar.anagani@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#include <linux/err.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/atomisp_platform.h>


/* Helper macros to manipulate bitfields */
#define REG_MASK(n)		(((1 << (n##_BIT_LEN)) - 1) << (n##_BIT_POS))
#define REG_SET_FIELD(r, n, v)	(((r) & ~REG_MASK(n)) | \
				 (((v) << (n##_BIT_POS)) & REG_MASK(n)))
#define REG_GET_FIELD(r, n)	(((r) & REG_MASK(n)) >> n##_BIT_POS)
/*
 * vlv2 platform has 6 platform clocks, controlled by 4 byte registers
 * Total size required for mapping is 6*4 = 24 bytes
 */
#define PMC_MAP_SIZE			24

static DEFINE_MUTEX(clk_mutex);
static void __iomem *pmc_base;

/*
 * byt_plat_set_clock_freq - Set clock frequency to a specified platform clock
 * @clk_num: Platform clock number (i.e. 0, 1, 2, ...,5)
 * @freq_type: Clock frequency (0-25 MHz(XTAL), 1-19.2 MHz(PLL) )
 */
int byt_plat_set_clock_freq(int clk_num, int freq_type)
{
	void __iomem *addr;

	if (clk_num < 0 && clk_num > MAX_CLK_COUNT) {
		pr_err("Clock number out of range (%d)\n", clk_num);
		return -EINVAL;
	}

	if (freq_type != CLK_FREQ_TYPE_XTAL &&
	    freq_type != CLK_FREQ_TYPE_PLL) {
		pr_err("wrong clock type\n");
		return -EINVAL;
	}

	if (!pmc_base) {
		pr_err("memio map is not set\n");
		return -EINVAL;
	}

	addr = pmc_base + PLT_CLK_CTL_OFFSET(clk_num);

	mutex_lock(&clk_mutex);
	writel(REG_SET_FIELD(readl(addr), CLK_FREQ_TYPE, freq_type), addr);
	mutex_unlock(&clk_mutex);

	return 0;
}
EXPORT_SYMBOL_GPL(byt_plat_set_clock_freq);

/*
 * byt_plat_get_clock_freq - Get the status of specified platform clock
 * @clk_num: Platform clock number (i.e. 0, 1, 2, ...,5)
 *
 * Returns 0 for 25 MHz(XTAL) and 1 for 19.2 MHz(PLL)
 */
int byt_plat_get_clock_freq(int clk_num)
{
	u32 ret;

	if (clk_num < 0 && clk_num > MAX_CLK_COUNT) {
		pr_err("Clock number out of range (%d)\n", clk_num);
		return -EINVAL;
	}

	if (!pmc_base) {
		pr_err("memio map is not set\n");
		return -EINVAL;
	}

	mutex_lock(&clk_mutex);
	ret = REG_GET_FIELD(readl(pmc_base + PLT_CLK_CTL_OFFSET(clk_num)),
			    CLK_FREQ_TYPE);
	mutex_unlock(&clk_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(byt_plat_get_clock_freq);

/*
 * byt_plat_configure_clock - Configure the specified platform clock
 * @clk_num: Platform clock number (i.e. 0, 1, 2, ...,5)
 * @conf:      Clock gating:
 *		0   - Clock gated on D3 state
 *		1   - Force on
 *		2,3 - Force off
 */
int byt_plat_configure_clock(int clk_num, u32 conf)
{
	void __iomem *addr;

	if (clk_num < 0 && clk_num > MAX_CLK_COUNT) {
		pr_err("Clock number out of range (%d)\n", clk_num);
		return -EINVAL;
	}

	if (conf != CLK_CONFG_D3_GATED &&
	    conf != CLK_CONFG_FORCE_ON &&
	    conf != CLK_CONFG_FORCE_OFF) {
		pr_err("Invalid clock configuration requested\n");
		return -EINVAL;
	}

	if (!pmc_base) {
		pr_err("memio map is not set\n");
		return -EINVAL;
	}

	addr = pmc_base + PLT_CLK_CTL_OFFSET(clk_num);

	mutex_lock(&clk_mutex);
	writel(REG_SET_FIELD(readl(addr), CLK_CONFG, conf), addr);
	mutex_unlock(&clk_mutex);
	return 0;
}
EXPORT_SYMBOL_GPL(byt_plat_configure_clock);

/*
 * byt_plat_get_clock_status - Get the status of specified platform clock
 * @clk_num: Platform clock number (i.e. 0, 1, 2, ...,5)
 *
 * Returns 1 - On, 0 - Off
 */
int byt_plat_get_clock_status(int clk_num)
{
	int ret;

	if (clk_num < 0 && clk_num > MAX_CLK_COUNT) {
		pr_err("Clock number out of range (%d)\n", clk_num);
		return -EINVAL;
	}

	if (!pmc_base) {
		pr_err("memio map is not set\n");
		return -EINVAL;
	}

	mutex_lock(&clk_mutex);
	ret = (int)REG_GET_FIELD(readl(pmc_base + PLT_CLK_CTL_OFFSET(clk_num)),
				 CLK_CONFG);
	mutex_unlock(&clk_mutex);
	return ret;
}
EXPORT_SYMBOL_GPL(byt_plat_get_clock_status);

int byt_plat_clk_init(void)
{
        pmc_base = ioremap_nocache(VLV2_PMC_CLK_BASE_ADDRESS, PMC_MAP_SIZE);
        if (!pmc_base) {
                return -ENOMEM;
        }

	return 0;
}
EXPORT_SYMBOL_GPL(byt_plat_clk_init);

