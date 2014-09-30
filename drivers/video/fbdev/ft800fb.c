#include <linux/err.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/random.h>
#include <linux/fb.h>
#include <linux/timer.h>
#include <linux/spinlock.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <video/of_display_timing.h>
#include <video/videomode.h>

#define FT800_WRITE 0x800000

#define FT800_HCMD_ACTIVE  0
#define FT800_HCMD_CORERST 0x680000
#define FT800_HCMD_CLKEXT  0x440000

#define FT800_MEM_RAM_G    0x0
#define FT800_MEM_RAM_DL   0x100000

#define FT800_REG_BASE 0x102400
#define FT800_REG_ID       (FT800_REG_BASE + 0x0)
#define FT800_REG_PWM_DUTY (FT800_REG_BASE + 0xc4)
#define FT800_REG_PWM_HZ   (FT800_REG_BASE + 0xc0)
#define FT800_REG_GPIO     (FT800_REG_BASE + 0x90)
#define FT800_REG_PCLK     (FT800_REG_BASE + 0x6c)

#define FT800_REG_HCYCLE  (FT800_REG_BASE + 0x28)
#define FT800_REG_HOFFSET  (FT800_REG_BASE + 0x2c)
#define FT800_REG_HSIZE  (FT800_REG_BASE + 0x30)
#define FT800_REG_HSYNC0  (FT800_REG_BASE + 0x34)
#define FT800_REG_HSYNC1  (FT800_REG_BASE + 0x38)

#define FT800_REG_VCYCLE  (FT800_REG_BASE + 0x3c)
#define FT800_REG_VOFFSET  (FT800_REG_BASE + 0x40)
#define FT800_REG_VSIZE  (FT800_REG_BASE + 0x44)
#define FT800_REG_VSYNC0  (FT800_REG_BASE + 0x48)
#define FT800_REG_VSYNC1  (FT800_REG_BASE + 0x4c)

#define FT800_REG_SWIZZLE (FT800_REG_BASE + 0x60)
#define FT800_REG_CSPREAD (FT800_REG_BASE + 0x64)
#define FT800_REG_PCLK_POL (FT800_REG_BASE + 0x68)
#define FT800_REG_DLSWAP (FT800_REG_BASE + 0x50)

#define FT800_REG_TOUCH_DIRECT_XY (FT800_REG_BASE + 0x174)
#define FT800_REG_TOUCH_DIRECT_Z1Z2 (FT800_REG_BASE + 0x178)

#define FT800_REG_TOUCH_RAW_XY (FT800_REG_BASE + 0x108)
#define FT800_REG_TOUCH_RZ (FT800_REG_BASE + 0x10c)
#define FT800_REG_TOUCH_MODE (FT800_REG_BASE + 0xf0)

#define FT800_REG_ID_VALUE 0x7c

#define FT800_DL_BITMAP_HANDLE(i) ((0x5 << 24) | (i))
#define FT800_DL_BITMAP_SOURCE(i) ((0x1 << 24) | (i))
#define BM(v, bits, lsb) (((v) & ((1 << (bits)) - 1)) << (lsb))
#define FT800_DL_BITMAP_LAYOUT(fmt, stride, height) \
	(BM(7, 8, 24) | BM(fmt, 5, 19) | BM(stride, 10, 9) | BM(height, 9, 0))

#define FT800_DL_BITMAP_SIZE(bilinear, wrapx, wrapy, width, height) \
	(BM(0x8, 8, 24) | BM(bilinear, 1, 20) | BM(wrapx, 1, 19) | \
	    BM(wrapy, 1, 18) | BM(width, 9, 9) | BM(height, 9, 0))

#define FT800_DL_BEGIN(primitive) (BM(0x1f, 8, 24) | BM(primitive, 4, 0))
#define FT800_DL_VERTEX2II(x, y, handle, cell) \
	(BM(2, 2, 30) | BM(x, 9, 21) | BM(y, 9, 12) | BM(handle, 5, 7) | \
		BM(cell, 7, 0))
#define FT800_DL_BITMAP_TRANSFORM_A(v) (BM(0x15, 8, 24) | BM(v, 17, 0))
#define FT800_DL_BITMAP_TRANSFORM_B(v) (BM(0x16, 8, 24) | BM(v, 17, 0))
#define FT800_DL_BITMAP_TRANSFORM_C(v) (BM(0x17, 8, 24) | BM(v, 24, 0))
#define FT800_DL_BITMAP_TRANSFORM_D(v) (BM(0x18, 8, 24) | BM(v, 17, 0))
#define FT800_DL_BITMAP_TRANSFORM_E(v) (BM(0x19, 8, 24) | BM(v, 17, 0))
#define FT800_DL_BITMAP_TRANSFORM_F(v) (BM(0x1A, 8, 24) | BM(v, 24, 0))
#define FT800_DL_END 0

#define FT800_BITMAP_FORMAT_RGB565 0x7
#define FT800_DL_BEGIN_BITMAPS 0x1

#define BYTES_PER_PIXEL 2
#define REDRAW_CHUNK_SIZE 0x2000
#define REDRAW_DELAY_MSEC 5

#define FT800_SPI_FREQ_STARTUP 10000000 /* 11 MHz per spec */
#define FT800_SPI_FREQ_NOMINAL 30000000 /* 30 MHz per spec */
#define FT800_PALETTE_SIZE 256

#define FT800_TOUCHSCREEN_XY_MAX 0x3ff
#define TOUCHSCREEN_BUFFER_SIZE 8
#define TOUCHSCREEN_POLL_DELAY_MS 10

/* Structure that holds control data for FT800 spi r/w transfers */
struct ft800_spi_buffer {
	u32 cmd;
	struct spi_message message;
	struct spi_transfer transfers[2];
};

struct ft800_data {
	int reset_gpio;
	bool reset_active_low;

	int hw_width, hw_height; /* physical display width/height in pixels */
	int virt_width, virt_height; /* framebuffer width/height in pixels */
	int stride; /* bytes per line */
	uint32_t rotation; /* clockwise rotation index, 0..3 */
	bool flip_x, flip_y; /* clockwise rotation index, 0..3 */

	struct videomode videomode;

	uint32_t touchscreen_rotation;
	bool touchscreen_flip_x;
	bool touchscreen_flip_y;

	uint32_t spi_max_freq;

	spinlock_t lock;

	struct fb_info *fb_info;
	u32 pseudo_palette[FT800_PALETTE_SIZE];

	/* struct page **pages; */
	u8 *vmem;
	size_t vmem_size;

	struct input_dev *input_dev;

	/* variables used by framebuffer transfer process */
	/* state vars */
	int  fb_users; /* number of active open() */
	bool fb_dirty; /* buffer was written to using blit/write/copy funcs */
	bool fb_mmapped; /* buffer is mmaped -- difficult to know is it dirty */
	bool fb_blanked; /* non-zero blank mode was set */
	bool fb_shutdown; /* shutdown was requested */
	bool fb_redraw_in_progress; /* is there an ongoing redraw */

	int  fb_redraw_bytes_copied; /* number of bytes already transferred */
	u32 fb_prev_jiffie;

	struct timer_list fb_timer;
	struct ft800_spi_buffer fb_refresh_buffer;

	/* variables used by touchscreen input device */
	bool input_busy;
	bool input_shutdown;
	struct timer_list input_timer;
	struct ft800_spi_buffer input_poll_buffer;
	u8 input_poll_data[TOUCHSCREEN_BUFFER_SIZE];

};

/* ---------------------------------------------------------------- Generic */

static int ft800_xfer(struct spi_device *spi, uint32_t cmd,
			bool is_write, void *buf, int n)
{
	struct spi_message message;
	struct spi_transfer transfers[2];
	uint32_t cmd_buf = cpu_to_be32(cmd << 8);

	spi_message_init(&message);
	memset(transfers, 0, sizeof(transfers));
	transfers[0].tx_buf = &cmd_buf;
	transfers[0].len = is_write ? 3 : 4;
	spi_message_add_tail(&transfers[0], &message);

	if (n) {
		if (is_write)
			transfers[1].tx_buf = buf;
		else
			transfers[1].rx_buf = buf;
		transfers[1].len = n;
		spi_message_add_tail(&transfers[1], &message);
	}

	return spi_sync(spi, &message);
}

/* Writes buffer into FT800 memory and calls specified callback.
 * It uses buffers statically allocated in ft800_data stucture, so
 * callers must ensure that only one transfer is queued at a time.
 */
static int ft800_write_async(struct spi_device *spi,
			struct ft800_spi_buffer *sbuf,
			uint32_t addr, void *buf, int n,
			void (*complete), void *context)
{
	sbuf->cmd = cpu_to_be32((addr | FT800_WRITE) << 8);

	spi_message_init(&sbuf->message);
	sbuf->message.complete = complete;
	sbuf->message.context = context;

	memset(sbuf->transfers, 0, sizeof(sbuf->transfers));
	sbuf->transfers[0].tx_buf = &sbuf->cmd;
	sbuf->transfers[0].len = 3;
	sbuf->transfers[1].tx_buf = buf;
	sbuf->transfers[1].len = n;

	spi_message_add_tail(&sbuf->transfers[0], &sbuf->message);
	spi_message_add_tail(&sbuf->transfers[1], &sbuf->message);

	return spi_async(spi, &sbuf->message);

}

static int ft800_read_async(struct spi_device *spi,
			struct ft800_spi_buffer *sbuf,
			uint32_t addr, void *buf, int n,
			void (*complete), void *context)
{
	sbuf->cmd = cpu_to_be32(addr << 8);

	spi_message_init(&sbuf->message);
	sbuf->message.complete = complete;
	sbuf->message.context = context;

	memset(sbuf->transfers, 0, sizeof(sbuf->transfers));
	sbuf->transfers[0].tx_buf = &sbuf->cmd;
	sbuf->transfers[0].len = 4;
	sbuf->transfers[1].rx_buf = buf;
	sbuf->transfers[1].len = n;

	spi_message_add_tail(&sbuf->transfers[0], &sbuf->message);
	spi_message_add_tail(&sbuf->transfers[1], &sbuf->message);

	return spi_async(spi, &sbuf->message);

}

/*
static int ft800_write(struct spi_device *spi, uint32_t addr, void *buf, int n)
{
	return ft800_xfer(spi, addr | FT800_WRITE, 1, buf, n);
}
*/

static int ft800_w_u32(struct spi_device *spi, uint32_t reg, uint32_t val)
{
	uint32_t tmp = cpu_to_le32(val);
	return ft800_xfer(spi, reg | FT800_WRITE, 1, &tmp, 4);
}

static uint32_t ft800_r_u32(struct spi_device *spi, uint32_t reg, uint32_t *val)
{
	int rc;
	uint32_t tmp;

	rc = ft800_xfer(spi, reg, 0, &tmp, 4);
	if (rc)
		return rc;
	*val = le32_to_cpu(tmp);
	return 0;
}

static int ft800_hcmd(struct spi_device *spi, uint32_t cmd)
{
	return ft800_xfer(spi, cmd, 1, NULL, 0);
}

static int ft800_reset(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	int rc;

	spi->max_speed_hz = FT800_SPI_FREQ_STARTUP;
	/* FIXME: try to find timing in datasheet */

	/* If hardware reset is available, use it */
	if (gpio_is_valid(fdata->reset_gpio)) {
		dev_info(&spi->dev, "reset: 0");
		gpio_set_value_cansleep(fdata->reset_gpio,
					fdata->reset_active_low ? 0 : 1);
		msleep(50);
		gpio_set_value_cansleep(fdata->reset_gpio,
					fdata->reset_active_low ? 1 : 0);
		dev_info(&spi->dev, "reset: 1");
	}

	rc = ft800_hcmd(spi, FT800_HCMD_CORERST);
	if (rc)
		return rc;

	msleep(20); /* 5ms should be enough */
	rc = ft800_hcmd(spi, FT800_HCMD_ACTIVE);
	if (rc)
		return rc;

	msleep(50);
	rc = ft800_hcmd(spi, FT800_HCMD_CLKEXT);
	if (rc)
		return rc;

	msleep(150);

	spi->max_speed_hz = fdata->spi_max_freq;

	/* reset TFT panel */
	rc = ft800_w_u32(spi, FT800_REG_GPIO, 0x0);
	if (rc)
		return rc;

	msleep(100);
	rc = ft800_w_u32(spi, FT800_REG_GPIO, 0x80);
	if (rc)
		return rc;

	msleep(50);

	return 0;
}

/* ---------------------------------------------------------------- FB */

/*static void ft800fb_update(struct fb_info *info)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);

	ft800_write(spi, FT800_MEM_RAM_G, fdata->vmem,
		fdata->stride * fdata->height);
}*/

static bool start_new_redraw(struct ft800_data *fdata)
{
	if (fdata->fb_shutdown || fdata->fb_blanked)
		return false;

	if (fdata->fb_redraw_in_progress)
		return false;

	if (fdata->fb_mmapped || fdata->fb_dirty)
		return true;

	return false;
}

static bool continue_ongoing_redraw(struct ft800_data *fdata)
{
	if (fdata->fb_shutdown || fdata->fb_blanked)
		return false;

	return fdata->fb_redraw_in_progress;
}

static void ft800_schedule_redraw(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	bool start_redraw = false;
	unsigned long flags;

	spin_lock_irqsave(&fdata->lock, flags);
	start_redraw = start_new_redraw(fdata);
	if (start_redraw)
		fdata->fb_redraw_in_progress = true;
	spin_unlock_irqrestore(&fdata->lock, flags);

	if (start_redraw)
		mod_timer(&fdata->fb_timer, jiffies +
			msecs_to_jiffies(REDRAW_DELAY_MSEC));
}

void ft800_redraw_complete(void *data)
{
	struct spi_device *spi = data;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	bool continue_redraw = false;
	unsigned long flags;
	int n;

	spin_lock_irqsave(&fdata->lock, flags);
	if (fdata->fb_redraw_bytes_copied == fdata->vmem_size)
		fdata->fb_redraw_in_progress = false;

	continue_redraw = continue_ongoing_redraw(fdata);
	fdata->fb_redraw_in_progress = continue_redraw;
	spin_unlock_irqrestore(&fdata->lock, flags);

	if (continue_redraw) {
		n = fdata->vmem_size - fdata->fb_redraw_bytes_copied;
		n = min(n, REDRAW_CHUNK_SIZE);
		ft800_write_async(spi, &fdata->fb_refresh_buffer,
			FT800_MEM_RAM_G + fdata->fb_redraw_bytes_copied,
			fdata->vmem + fdata->fb_redraw_bytes_copied, n,
			ft800_redraw_complete, spi);
		fdata->fb_redraw_bytes_copied += n;
	} else {
		ft800_schedule_redraw(spi);
	}
}

void ft800_redraw_timer(unsigned long data)
{
	struct spi_device *spi = (struct spi_device *) data;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	unsigned long flags;
	int n;

	spin_lock_irqsave(&fdata->lock, flags); /* context is hwirq or swirq */
	fdata->fb_dirty = false;
	spin_unlock_irqrestore(&fdata->lock, flags);

	if (fdata->fb_prev_jiffie)
		dev_dbg(&spi->dev, "redraw at %lu/1000 fps",
			1000*HZ/(jiffies - fdata->fb_prev_jiffie));
	fdata->fb_prev_jiffie = jiffies;

	n = min_t(size_t, fdata->vmem_size, REDRAW_CHUNK_SIZE);
	fdata->fb_redraw_bytes_copied = n;
	ft800_write_async(spi, &fdata->fb_refresh_buffer,
			FT800_MEM_RAM_G, fdata->vmem, n,
			ft800_redraw_complete, spi);
}

static void ft800_redraw_shutdown(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);

	bool busy;
	unsigned long flags;

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->fb_shutdown = true;
	spin_unlock_irqrestore(&fdata->lock, flags);

	while (1) {
		spin_lock_irqsave(&fdata->lock, flags);
		busy = fdata->fb_redraw_in_progress;
		spin_unlock_irqrestore(&fdata->lock, flags);

		if (!busy)
			break;

		msleep(20);
	}
}


static void ft800fb_mark_dirty(struct fb_info *info)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	unsigned long flags;

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->fb_dirty = true;
	spin_unlock_irqrestore(&fdata->lock, flags);

	ft800_schedule_redraw(spi);
}


static ssize_t ft800fb_write(struct fb_info *info, const char __user *buf,
		size_t count, loff_t *ppos)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);

	dev_dbg(&spi->dev, "write %d", count);

	if ((*ppos >= fdata->vmem_size) || (count > fdata->vmem_size))
		return -EFBIG;

	if (count + *ppos > fdata->vmem_size)
		return -ENOSPC;

	if (copy_from_user(fdata->vmem + *ppos, buf, count))
		return -EFAULT;

	*ppos += count;
	ft800fb_mark_dirty(info);
	return count;
}

static void ft800fb_fillrect(struct fb_info *info,
			     const struct fb_fillrect *rect)
{
	struct spi_device *spi = info->par;
	dev_dbg(&spi->dev, "fillrect");
	sys_fillrect(info, rect);
	ft800fb_mark_dirty(info);
}

static void ft800fb_copyarea(struct fb_info *info,
			     const struct fb_copyarea *area)
{
	struct spi_device *spi = info->par;
	dev_dbg(&spi->dev, "copyarea");
	sys_copyarea(info, area);
	ft800fb_mark_dirty(info);
}

static void ft800fb_imageblit(struct fb_info *info,
			      const struct fb_image *image)
{
	sys_imageblit(info, image);
	ft800fb_mark_dirty(info);
}

static inline unsigned int chan_to_field(unsigned int chan,
					const struct fb_bitfield *bf)
{
	chan &= 0xffff;
	chan >>= 16 - bf->length;
	return chan << bf->offset;
}

static int ft800fb_setcolreg(unsigned int regno, unsigned int red,
			     unsigned int green, unsigned int blue,
			     unsigned int transp, struct fb_info *info)
{
	unsigned int val;
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);

	if (regno < ARRAY_SIZE(fdata->pseudo_palette)) {
		val  = chan_to_field(red, &info->var.red);
		val |= chan_to_field(green, &info->var.green);
		val |= chan_to_field(blue, &info->var.blue);
		((u32 *)info->pseudo_palette)[regno] = val;
		/* printk("set palette %d = %x %x %x %x -> %x\n", regno, */
		/* red, green, blue, transp, val); */
		return 0;
	} else {
		return -EINVAL;
	}
}

static int ft800fb_blank(int blank_mode, struct fb_info *info)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	unsigned long flags;
	bool fb_blanked, try_to_redraw;

	dev_info(&spi->dev, "blank mode=%d", blank_mode);

	spin_lock_irqsave(&fdata->lock, flags);
	fb_blanked = blank_mode != FB_BLANK_UNBLANK;
	try_to_redraw = !fb_blanked && fdata->fb_blanked;
	fdata->fb_blanked = fb_blanked;
	spin_unlock_irqrestore(&fdata->lock, flags);

	if (try_to_redraw)
		ft800_schedule_redraw(spi);

	return 0;
}

static int ft800fb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	unsigned long flags;
	unsigned long mmio_pgoff;
	unsigned long start;
	u32 len;
	int ret;

	dev_info(&spi->dev, "mmap()");

	start = info->fix.smem_start;
	len = info->fix.smem_len;
	mmio_pgoff = PAGE_ALIGN((start & ~PAGE_MASK) + len) >> PAGE_SHIFT;
	if (vma->vm_pgoff >= mmio_pgoff)
		return -EINVAL;

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);

	ret = vm_iomap_memory(vma, start, len);
	if (ret)
		return ret;

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->fb_mmapped = true;
	spin_unlock_irqrestore(&fdata->lock, flags);
	ft800_schedule_redraw(spi);
	return 0;
}

/*static int ft800_test(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	int i; //, j;
	//int addr;
	//spi->max_speed_hz = 30000000;

	dev_info(&spi->dev, "start");
	for (i=0; i < 20; i+=1) {
		prandom_bytes(fdata->vmem, fdata->stride * fdata->height);
		ft800fb_update(fdata->fb_info);
	}
	dev_info(&spi->dev, "end");
	return 0;
	}*/

int ft800fb_open(struct fb_info *info, int user)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	unsigned long flags;

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->fb_users++;
	spin_unlock_irqrestore(&fdata->lock, flags);

	dev_info(&spi->dev, "open(), users=%d", fdata->fb_users);
	return 0;
}

int ft800fb_release(struct fb_info *info, int user)
{
	struct spi_device *spi = info->par;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);

	unsigned long flags;

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->fb_users--;
	if (fdata->fb_users == 0)
		fdata->fb_mmapped = false;
	spin_unlock_irqrestore(&fdata->lock, flags);

	dev_info(&spi->dev, "release(), users=%d", fdata->fb_users);
	return 0;
}

static struct fb_ops ft800fb_ops = {
	.owner		= THIS_MODULE,
	.fb_open	= ft800fb_open,
	.fb_release	= ft800fb_release,
	.fb_fillrect	= ft800fb_fillrect,
	.fb_copyarea	= ft800fb_copyarea,
	.fb_imageblit	= ft800fb_imageblit,
	.fb_setcolreg	= ft800fb_setcolreg,
	.fb_mmap	= ft800fb_mmap,
	.fb_write	= ft800fb_write,
	.fb_blank	= ft800fb_blank,
};

static struct fb_fix_screeninfo ft800fb_fix = {
	.id		= "FT800",
	.type		= FB_TYPE_PACKED_PIXELS,
	.visual		= FB_VISUAL_TRUECOLOR,
	.xpanstep	= 0,
	.ypanstep	= 0,
	.ywrapstep	= 0,
	.accel		= FB_ACCEL_NONE,
};

static struct fb_var_screeninfo ft800fb_var = {
	.bits_per_pixel	= 16,
	.red.offset    =  0, .red.length    = 5,
	.green.offset  =  5, .green.length  = 6,
	.blue.offset   = 11, .blue.length   = 5,
	.transp.offset =  0, .transp.length = 5,
};


static int ft800_configure(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	int a00, a01, a10, a11, b0, b1;
	int hw_w, hw_h, w, h, i;
	int dcos[] = {1, 0, -1, 0};
	int dsin[] = {0, 1, 0, -1};
	int sync0, sync1, offset, cycle, size;
	struct videomode *vm;
	int ci = 0;

	/* Image coordinates (x', y') are calculated from
	 * screen coordinates (x, y) as follows:
	 * x' = a00 * x + a01 * y + b0
	 * y' = a10 * x + a11 * y + b1
	 */
	i = fdata->rotation;
	a00 =  dcos[i];
	a01 =  dsin[i];
	a10 = -dsin[i];
	a11 =  dcos[i];
	hw_w = fdata->videomode.hactive - 1;
	hw_h = fdata->videomode.vactive - 1;
	w = fdata->virt_width - 1;
	h = fdata->virt_height - 1;
	b0 = (w - hw_w * dcos[i] - hw_h * dsin[i])/2;
	b1 = (h + hw_w * dsin[i] - hw_h * dcos[i])/2;
	if (fdata->flip_x) {
		a00 *= -1;
		a01 *= -1;
		b0 = w - b0;
	}
	if (fdata->flip_y) {
		a10 *= -1;
		a11 *= -1;
		b1 = h - b1;
	}

	dev_info(&spi->dev, "xx [%d %d %d %d] [%d %d]",
			a00, a01, a10, a11, b0, b1);

	vm = &fdata->videomode;

	sync0 = vm->hback_porch;
	sync1 = sync0 + vm->hsync_len;
	offset = sync1 + vm->hfront_porch;
	size = vm->hactive;
	cycle = offset + size;
	ft800_w_u32(spi, FT800_REG_HCYCLE, cycle);
	ft800_w_u32(spi, FT800_REG_HOFFSET, offset);
	ft800_w_u32(spi, FT800_REG_HSYNC0, sync0);
	ft800_w_u32(spi, FT800_REG_HSYNC1, sync1);
	ft800_w_u32(spi, FT800_REG_HSIZE, size);

	sync0 = vm->vback_porch;
	sync1 = sync0 + vm->vsync_len;
	offset = sync1 + vm->vfront_porch;
	size = vm->vactive;
	cycle = offset + size;
	ft800_w_u32(spi, FT800_REG_VCYCLE, cycle);
	ft800_w_u32(spi, FT800_REG_VOFFSET, offset);
	ft800_w_u32(spi, FT800_REG_VSYNC0, sync0);
	ft800_w_u32(spi, FT800_REG_VSYNC1, sync1);
	ft800_w_u32(spi, FT800_REG_VSIZE, size);

	ft800_w_u32(spi, FT800_REG_SWIZZLE, 0);
	ft800_w_u32(spi, FT800_REG_PCLK_POL, 1);
	ft800_w_u32(spi, FT800_REG_CSPREAD, 0);

	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_HANDLE(0));
	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_SOURCE(0));

	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4, FT800_DL_BITMAP_LAYOUT(
		FT800_BITMAP_FORMAT_RGB565, fdata->stride, fdata->virt_height));

	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_TRANSFORM_A((a00*0x100) & 0x1ff00));
	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_TRANSFORM_B((a01*0x100) & 0x1ff00));
	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_TRANSFORM_C(b0*0x100));
	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_TRANSFORM_D((a10*0x100) & 0x1ff00));
	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_TRANSFORM_E((a11*0x100) & 0x1ff00));
	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BITMAP_TRANSFORM_F(b1*0x100));

	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_BEGIN(FT800_DL_BEGIN_BITMAPS));

	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4,
		FT800_DL_VERTEX2II(0, 0, 0, 0));

	ft800_w_u32(spi, FT800_MEM_RAM_DL + (ci++)*4, FT800_DL_END);

	ft800_w_u32(spi, FT800_REG_DLSWAP, 1);
	ft800_w_u32(spi, FT800_REG_PCLK, 8); /* PCLK divisor */

	/* ft800_w_u32(spi, FT800_REG_PWM_DUTY, 0x7f); */ /* FIXME */
	ft800_w_u32(spi, FT800_REG_PWM_DUTY, 0x2f);

	msleep(400); /* screen becomes garbled if removed */
	return 0;
}

static int ft800_init_fb(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	struct device_node *np = spi->dev.of_node;
	struct display_timings *timings;
	struct fb_info *fb;
	int rc;

	fdata->fb_users = 0;
	fdata->fb_dirty = false;
	fdata->fb_mmapped = false;
	fdata->fb_blanked = false;
	fdata->fb_shutdown = false;
	fdata->fb_redraw_in_progress = false;

	if (of_property_read_u32(np, "rotation", &fdata->rotation))
		fdata->rotation = 0;

	if (fdata->rotation >= 4) {
		dev_err(&spi->dev, "'rotation' dt value is invalid");
		return -EINVAL;
	}
	/* fdata->rotation = 1; */

	fdata->flip_x = of_property_read_bool(np, "flip-x");
	fdata->flip_y = of_property_read_bool(np, "flip-y");

	timings = of_get_display_timings(np);
	if (!timings) {
		dev_err(&spi->dev, "failed to get display timings\n");
		return -EINVAL;
	}

	rc = videomode_from_timings(timings, &fdata->videomode,
				    timings->native_mode);
	display_timings_release(timings);
	if (rc)
		return rc;

	fdata->virt_width = fdata->videomode.hactive;
	fdata->virt_height = fdata->videomode.vactive;
	if (fdata->rotation & 1)
		swap(fdata->virt_width, fdata->virt_height);

	fdata->stride = fdata->virt_width * BYTES_PER_PIXEL;
	fdata->vmem_size = fdata->stride * fdata->virt_height;

	fdata->fb_redraw_bytes_copied = 0;

	rc = ft800_configure(spi);
	if (rc)
		return rc;


	/* FIXME: align to page */
	fdata->vmem = devm_kzalloc(&spi->dev, fdata->vmem_size,
				GFP_KERNEL | GFP_DMA);

	if (!fdata->vmem)
		return -ENOMEM;


	fb = framebuffer_alloc(sizeof(void *), &spi->dev);
	if (!fb) {
		dev_err(&spi->dev, "Cannot allocate framebuffer");
		return -ENOMEM;
	}
	fdata->fb_info = fb;
	fb->par = spi;
	fb->fbops = &ft800fb_ops;
	fb->pseudo_palette = &fdata->pseudo_palette;

	fb->fix = ft800fb_fix;
	fb->fix.line_length = fdata->stride;

	fb->var = ft800fb_var;
	fb->var.xres = fdata->virt_width;
	fb->var.xres_virtual = fdata->virt_width;
	fb->var.yres = fdata->virt_height;
	fb->var.yres_virtual = fdata->virt_height;

	fb->screen_base = (u8 __force __iomem *) fdata->vmem;
	fb->fix.smem_start = (unsigned long) __pa(fdata->vmem);
	fb->fix.smem_len = fdata->stride * fdata->virt_height;

	/* ft800_test(spi); */
	rc = register_framebuffer(fb);
	if (rc) {
		dev_err(&spi->dev, "Cannot register framebuffer");
		return rc;
	}


	setup_timer(&fdata->fb_timer, ft800_redraw_timer, (unsigned long) spi);
	dev_info(&spi->dev, "fb dev init success");
	return 0;
}


/* ---------------------------------------------------------------- Input */

static void ft800_input_xfer_complete(void *data)
{
	struct spi_device *spi = data;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	bool start_transfer = false;
	u8 *buf = fdata->input_poll_data;
	unsigned long flags;
	int x, y, xr, yr, r, p;

	y = buf[0] + ((int) buf[1] << 8);
	x = buf[2] + ((int) buf[3] << 8);
	r = buf[4] + ((int) buf[5] << 8);

	if ((x <= FT800_TOUCHSCREEN_XY_MAX) &&
	    (y <= FT800_TOUCHSCREEN_XY_MAX)) {
		int dcos[] = {1, 0, -1, 0}; /* FIXME */
		int dsin[] = {0, 1, 0, -1};
		int i = fdata->touchscreen_rotation;
		xr =  dcos[i] * x + dsin[i] * y +
			(1 - dcos[i] - dsin[i]) * FT800_TOUCHSCREEN_XY_MAX / 2;
		yr = -dsin[i] * x + dcos[i] * y +
			(1 + dsin[i] - dcos[i]) * FT800_TOUCHSCREEN_XY_MAX / 2;
		if (fdata->touchscreen_flip_x)
			xr = FT800_TOUCHSCREEN_XY_MAX - xr;
		if (fdata->touchscreen_flip_y)
			yr = FT800_TOUCHSCREEN_XY_MAX - yr;

		p = r ? min(0xff, 3 * 0x7fff / r) : 0xff;
		dev_dbg(&spi->dev, "(%d, %d) raw=(%d, %d) r=%d, p=%d",
				xr, yr, x, y, r, p);

		input_report_key(fdata->input_dev, BTN_TOUCH, 1);
		input_report_abs(fdata->input_dev, ABS_X, xr);
		input_report_abs(fdata->input_dev, ABS_Y, yr);
		input_report_abs(fdata->input_dev, ABS_PRESSURE, p);
		input_sync(fdata->input_dev);
	} else {
		input_report_key(fdata->input_dev, BTN_TOUCH, 0);
		input_report_abs(fdata->input_dev, ABS_PRESSURE, 0);
		input_sync(fdata->input_dev);
	}



	spin_lock_irqsave(&fdata->lock, flags);
	if (!fdata->input_shutdown)
		start_transfer = true;
	else
		fdata->input_busy = false;
	spin_unlock_irqrestore(&fdata->lock, flags);

	if (start_transfer)
		mod_timer(&fdata->input_timer, jiffies +
			  msecs_to_jiffies(TOUCHSCREEN_POLL_DELAY_MS));
}

#if (FT800_REG_TOUCH_RZ + 4 - FT800_REG_TOUCH_RAW_XY) != TOUCHSCREEN_BUFFER_SIZE
#error "Wrong register offsets"
#endif

static void ft800_input_timer(unsigned long data)
{
	struct spi_device *spi = (struct spi_device *) data;
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	/*if (!ft800_r_u32(spi, FT800_REG_TOUCH_RAW_XY, &xy_reg) &&
		!ft800_r_u32(spi, FT800_REG_TOUCH_RZ, &r_reg)) {
	*/
	ft800_read_async(spi, &fdata->input_poll_buffer,
			FT800_REG_TOUCH_RAW_XY,
			fdata->input_poll_data, TOUCHSCREEN_BUFFER_SIZE,
			ft800_input_xfer_complete, spi);
}

static void ft800_input_shutdown(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	unsigned long flags;
	bool busy;

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->input_shutdown = true;
	spin_unlock_irqrestore(&fdata->lock, flags);

	while (1) {
		spin_lock_irqsave(&fdata->lock, flags);
		busy = fdata->input_busy;
		spin_unlock_irqrestore(&fdata->lock, flags);

		if (!busy)
			break;

		msleep(20);
	}
}

static int ft800_init_input(struct spi_device *spi)
{
	struct ft800_data *fdata = dev_get_drvdata(&spi->dev);
	struct device_node *np = spi->dev.of_node;
	unsigned long flags;
	int rc;

	fdata->input_dev = NULL;
	if (!of_property_read_bool(np, "touchscreen-enable"))
		return 0;

	fdata->input_dev = devm_input_allocate_device(&spi->dev);
	if (!fdata->input_dev)
		return PTR_ERR(fdata->input_dev);

	fdata->input_dev->name = "FT800 Touchscreen";
	fdata->input_dev->open = NULL;
	fdata->input_dev->close = NULL;

	fdata->input_dev->evbit[0] |= BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	fdata->input_dev->keybit[BIT_WORD(BTN_TOUCH)] |= BIT_MASK(BTN_TOUCH);

	input_set_abs_params(fdata->input_dev, ABS_X, 0,
			     FT800_TOUCHSCREEN_XY_MAX, 4, 0);
	input_set_abs_params(fdata->input_dev, ABS_Y, 0,
			     FT800_TOUCHSCREEN_XY_MAX, 4, 0);
	input_set_abs_params(fdata->input_dev, ABS_PRESSURE, 0, 0xff, 0, 0);

	if (of_property_read_u32(np, "touchscreen-rotation",
			&fdata->touchscreen_rotation))
		fdata->touchscreen_rotation = 0;

	if (fdata->touchscreen_rotation >= 4) {
		dev_err(&spi->dev,
			"'touchscreen-rotation' dt value is invalid");
		return -EINVAL;
	}

	fdata->touchscreen_flip_x = of_property_read_bool(np,
			"touchscreen-flip-x");
	fdata->touchscreen_flip_y = of_property_read_bool(np,
			"touchscreen-flip-y");

	rc = input_register_device(fdata->input_dev);
	if (rc) {
		dev_err(&spi->dev, "Error registering input dev");
		return rc;
	}
	dev_info(&spi->dev, "input dev init success");

	setup_timer(&fdata->input_timer, ft800_input_timer,
		    (unsigned long) spi);

	spin_lock_irqsave(&fdata->lock, flags);
	fdata->input_busy = true;
	spin_unlock_irqrestore(&fdata->lock, flags);
	mod_timer(&fdata->input_timer, jiffies + msecs_to_jiffies(2500));
	return 0;
}

/* ---------------------------------------------------------------- Probe/Rm */

static int ft800_probe(struct spi_device *spi)
{
	u32 reg;

	struct ft800_data *fdata;
	enum of_gpio_flags flags;
	struct device_node *np;
	unsigned long gpio_conf;
	int rc;

	fdata = devm_kzalloc(&spi->dev, sizeof(*fdata),
			GFP_KERNEL | GFP_DMA);
	if (!fdata)
		return -ENOMEM;

	dev_set_drvdata(&spi->dev, fdata);

	init_timer(&fdata->fb_timer);
	spin_lock_init(&fdata->lock);

	fdata->spi_max_freq = min_t(uint32_t,
			spi->max_speed_hz, FT800_SPI_FREQ_NOMINAL);

	fdata->input_busy = false;
	fdata->input_shutdown = false;

	np = spi->dev.of_node;
	fdata->reset_gpio = -1;

	fdata->reset_gpio = of_get_named_gpio_flags(np, "reset-gpios", 0,
						    &flags);
	if (!gpio_is_valid(fdata->reset_gpio)) {
		dev_warn(&spi->dev, "no reset GPIO specified, will use SW reset");
	} else {
		fdata->reset_active_low = flags & OF_GPIO_ACTIVE_LOW;
		gpio_conf = fdata->reset_active_low ?
				GPIOF_OUT_INIT_HIGH : GPIOF_OUT_INIT_LOW;
		rc = devm_gpio_request_one(&spi->dev, fdata->reset_gpio,
					   gpio_conf, dev_name(&spi->dev));
		if (rc)
			return rc;
	}

	rc = ft800_reset(spi);
	if (rc)
		return rc;

	rc = ft800_r_u32(spi, FT800_REG_ID, &reg);
	if (rc)
		return rc;

	if (reg != FT800_REG_ID_VALUE) {
		dev_err(&spi->dev, "Wrong device id, got 0x%x, expected 0x%x",
			reg, FT800_REG_ID_VALUE);
		return rc;
	}

	rc = ft800_init_fb(spi);
	if (rc)
		return rc;

	rc = ft800_init_input(spi);
	if (rc)
		return rc;

	/* FIXME -- do proper release*/
	return 0;
}

static int ft800_remove(struct spi_device *spi)
{
	struct ft800_data *fdata = spi_get_drvdata(spi);

	if (fdata->input_dev)
		input_unregister_device(fdata->input_dev);

	ft800_redraw_shutdown(spi);
	ft800_input_shutdown(spi);
	unregister_framebuffer(fdata->fb_info);
	framebuffer_release(fdata->fb_info);
	return 0;
}


static const struct spi_device_id ft800_ids[] = {
	{ "ft800",  0},
	{}
};

static struct of_device_id ft800_spi_of_match[] = {
	{.compatible = "ftdi,ft800"},
	{}
};

static struct spi_driver ft800_driver = {
	.probe		= ft800_probe,
	.remove		= ft800_remove,
	.id_table	= ft800_ids,
	.driver = {
		.name	= "ft800",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(ft800_spi_of_match),
	},
};

module_spi_driver(ft800_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Alexander Morozov <linux@meltdown.ru>");
MODULE_DESCRIPTION("SPI driver for FT800 LCD device");
