#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/device.h>

#include <linux/mm.h>
#include <linux/dma-mapping.h>
#include <linux/uaccess.h>
#include <linux/sched/mm.h>
#include <linux/pagemap.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/slab.h>

#include <asm/current.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "dmacalc.h"

#include <asm/dma-mapping.h>




// ------------ ここから ------------

//  以下の 3つの キャッシュ操作のコードは
//  https://github.com/ikwzm/uiomem/blob/develop/uiomem.c
//  より転用させて頂いております。

static inline u64  arm64_read_dcache_line_size(void)
{
    u64       ctr;
    u64       dcache_line_size;
    const u64 bytes_per_word = 4;
    asm volatile ("mrs %0, ctr_el0" : "=r"(ctr) : : );
    asm volatile ("nop" : : : );
    dcache_line_size = (ctr >> 16) & 0xF;
    return (bytes_per_word << dcache_line_size);
}

static inline void arm64_inval_dcache_area(void* start, size_t size)
{
    u64   vaddr           = (u64)start;
    u64   __end           = (u64)start + size;
    u64   cache_line_size = arm64_read_dcache_line_size();
    u64   cache_line_mask = cache_line_size - 1;
    if ((__end & cache_line_mask) != 0) {
        __end &= ~cache_line_mask;
        asm volatile ("dc civac, %0" :  : "r"(__end) : );
    }
    if ((vaddr & cache_line_mask) != 0) {
        vaddr &= ~cache_line_mask;
        asm volatile ("dc civac, %0" :  : "r"(vaddr) : );
    }
    while (vaddr < __end) {
        asm volatile ("dc ivac, %0"  :  : "r"(vaddr) : );
        vaddr += cache_line_size;
    }
    asm volatile ("dsb  sy"  :  :  : );
}

static inline void arm64_clean_dcache_area(void* start, size_t size)
{
    u64   vaddr           = (u64)start;
    u64   __end           = (u64)start + size;
    u64   cache_line_size = arm64_read_dcache_line_size();
    u64   cache_line_mask = cache_line_size - 1;
    vaddr &= ~cache_line_mask;
    while (vaddr < __end) {
        asm volatile ("dc cvac, %0"  :  : "r"(vaddr) : );
        vaddr += cache_line_size;
    }
    asm volatile ("dsb  sy"  :  :  : );
}

// ------------ ここまで ------------






// PL に構成した DMAなど のベースアドレス
#define PL_BASE                     0xa0000000
#define DMAR_BASE                   (PL_BASE + 0x00000000)
#define DMAW_BASE                   (PL_BASE + 0x00100000)

// DMAの制御レジスタのオフセットアドレス
#define REG_DMAR_CORE_ID            0x00
#define REG_DMAR_CFG_DATA_BITS      0x04
#define REG_DMAR_CTL_STATUS         0x08
#define REG_DMAR_PARAM_ARID         0x10
#define REG_DMAR_PARAM_ARADDR       0x11
#define REG_DMAR_PARAM_ARLEN        0x12
#define REG_DMAR_PARAM_ARSIZE       0x13
#define REG_DMAR_PARAM_ARBURST      0x14
#define REG_DMAR_PARAM_ARLOCK       0x15
#define REG_DMAR_PARAM_ARCACHE      0x16
#define REG_DMAR_PARAM_ARPROT       0x17
#define REG_DMAR_PARAM_ARQOS        0x18
#define REG_DMAR_PARAM_ARREGION     0x19

#define REG_DMAW_CORE_ID            0x00
#define REG_DMAW_CFG_DATA_BITS      0x04
#define REG_DMAW_CTL_STATUS         0x08
#define REG_DMAW_CTL_ISSUE_CNT      0x09
#define REG_DMAW_PARAM_AWID         0x10
#define REG_DMAW_PARAM_AWADDR       0x11
#define REG_DMAW_PARAM_AWLEN        0x12
#define REG_DMAW_PARAM_AWSIZE       0x13
#define REG_DMAW_PARAM_AWBURST      0x14
#define REG_DMAW_PARAM_AWLOCK       0x15
#define REG_DMAW_PARAM_AWCACHE      0x16
#define REG_DMAW_PARAM_AWPROT       0x17
#define REG_DMAW_PARAM_AWQOS        0x18
#define REG_DMAW_PARAM_AWREGION     0x19

// メモリマップドレジスタへのアクセス用ポインタ
static volatile unsigned long *dmar;
static volatile unsigned long *dmaw;


static DEFINE_MUTEX(dmacalc_mutex);


MODULE_LICENSE("Dual MIT/GPL");

#define DRIVER_NAME "fpga_dmacalc"

static const unsigned int MINOR_BASE = 0;
static const unsigned int MINOR_NUM  = 1;

static unsigned int dmacalc_major;
static struct cdev  dmacalc_cdev;

static int dmacalc_open(struct inode *inode, struct file *file)
{
    printk("dmacalc_open");

    mutex_lock(&dmacalc_mutex);
    mutex_unlock(&dmacalc_mutex);

    return 0;
}

static int dmacalc_close(struct inode *inode, struct file *file)
{
    printk("dmacalc_close");

    mutex_lock(&dmacalc_mutex);
    mutex_unlock(&dmacalc_mutex);

    return 0;
}

static ssize_t dmacalc_read(struct file *filp, char __user *buf, size_t count, loff_t *f_pos)
{
    printk("dmacalc_read");
    mutex_lock(&dmacalc_mutex);
    mutex_unlock(&dmacalc_mutex);
    return 0;
}

static ssize_t dmacalc_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
    printk("dmacalc_write");
    mutex_lock(&dmacalc_mutex);
    mutex_unlock(&dmacalc_mutex);
    return 0;
}



#define MAX_PAGES   (512*1024+1)

static struct page  *src_pages[MAX_PAGES];
static struct page  *dst_pages[MAX_PAGES];

static long dmacalc_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
    unsigned long i;
    int ret;

    printk("dmacalc_ioctl\n");

    switch (cmd) {
    case DMACALC_CALC:
        {
            struct dmacalc_calc calc;

//          printk("DMACALC_CALC\n");
            if ( copy_from_user(&calc, (void __user *)arg, sizeof(calc))) {
                return -EFAULT;
            }
//          printk("src  : 0x%016lx\n", (unsigned long)calc.src);
//          printk("dst  : 0x%016lx\n", (unsigned long)calc.dst);
//          printk("size : %lx\n", calc.size);

            if ( calc.size > (MAX_PAGES-1)*PAGE_SIZE ) {
                return -EFAULT;
            }

            unsigned long r_unit = (dmar[REG_DMAR_CFG_DATA_BITS] / 8);
            unsigned long w_unit = (dmar[REG_DMAW_CFG_DATA_BITS] / 8);
            if ( r_unit == 0 ) { r_unit = 128/8; }
            if ( w_unit == 0 ) { w_unit = 128/8; }
            if ( r_unit != w_unit ) {
                return -EFAULT;
            }

            unsigned long src_addr   = (unsigned long)calc.src;
            unsigned long src_base   = src_addr & PAGE_MASK;
            unsigned long src_offset = src_addr & ~PAGE_MASK;
            unsigned long src_size   = (calc.size + src_offset + PAGE_SIZE - 1) / PAGE_SIZE;

            unsigned long dst_addr   = (unsigned long)calc.dst;
            unsigned long dst_base   = dst_addr & PAGE_MASK;
            unsigned long dst_offset = dst_addr & ~PAGE_MASK;
            unsigned long dst_size   = (calc.size + dst_offset + PAGE_SIZE - 1) / PAGE_SIZE;

            if ( src_addr % r_unit != 0 || dst_addr % w_unit != 0 || calc.size % r_unit != 0 ) {
                printk(KERN_ERR "Alignmant error\n");
                printk(KERN_ERR "src_addr : %016lx\n", (unsigned long)src_addr); 
                printk(KERN_ERR "dst_addr : %016lx\n", (unsigned long)dst_addr); 
                printk(KERN_ERR "size     : %016lx\n", (unsigned long)calc.size); 
                return -EFAULT;
            }

            mutex_lock(&dmacalc_mutex);

            if ( src_size > MAX_PAGES ) { src_size = MAX_PAGES; }
            ret = get_user_pages(src_base, src_size, FOLL_FORCE, src_pages, NULL);
            if (ret < 0) {
                printk(KERN_ERR "Failed to get_user_pages: %d\n", ret);
                mutex_unlock(&dmacalc_mutex);
                return ret;
            }
//          dma_addr_t src_dma_addr = page_to_phys(src_pages[0]);
//          printk("src_addr     : %016lx\n", (unsigned long)src_addr); 
//          printk("src_base     : %016lx\n", (unsigned long)src_base);
//          printk("src_offset   : %016lx\n", (unsigned long)src_offset); 
//          printk("src_size     : %016lx\n", (unsigned long)src_size); 
//          printk("src_pages[0] : %016lx\n", (unsigned long)src_pages[0]); 
//          printk("src_dma_addr : %016lx\n", (unsigned long)src_dma_addr); 

            if ( dst_size > MAX_PAGES ) { dst_size = MAX_PAGES; }
            ret = get_user_pages(dst_base, dst_size, FOLL_WRITE, dst_pages, NULL);
            if (ret < 0) {
                printk(KERN_ERR "Failed to get_user_pages: %d\n", ret);
                mutex_unlock(&dmacalc_mutex);
                return ret;
            }
//          dma_addr_t dst_dma_addr = page_to_phys(dst_pages[0]);
//          printk("dst_addr    : %016lx\n", (unsigned long)dst_addr); 
//          printk("dst_base    : %016lx\n", (unsigned long)dst_base);
//          printk("dst_offset  : %016lx\n", (unsigned long)dst_offset); 
//          printk("dst_size    : %016lx\n", (unsigned long)dst_size); 
//          printk("dst_pages[0] : %016lx\n", (unsigned long)dst_pages[0]); 
//          printk("dst_dma_addr : %016lx\n", (unsigned long)dst_dma_addr);

            {
                unsigned long src_n    = 0;
                unsigned long src_size = calc.size;

                unsigned long dst_n    = 0;
                unsigned long dst_size = calc.size;
                
                dmar[REG_DMAR_PARAM_ARCACHE] = 0x0f;
                dmar[REG_DMAR_PARAM_ARPROT]  = 0x02;
                dmaw[REG_DMAW_PARAM_AWCACHE] = 0x0f;
                dmaw[REG_DMAW_PARAM_AWPROT]  = 0x02;

                while ( src_size > 0 || dst_size > 0 ) {
                    if ( src_size > 0 && dmar[REG_DMAR_CTL_STATUS] == 0 ) {
                        dma_addr_t    addr = page_to_phys(src_pages[src_n]);
                        void         *kbuf = kmap(src_pages[src_n]);
                        void         *ptr  = (void *)((unsigned long)kbuf + src_offset);
                        unsigned long len  = PAGE_SIZE - src_offset;
                        if ( len > src_size ) { len = src_size; }
                        arm64_clean_dcache_area(ptr, len);
                        
                        unsigned long arlen = len / r_unit;
                        while ( arlen > 0 ) {
                            while ( dmar[REG_DMAR_CTL_STATUS] != 0) {
                                udelay(1);
                            }
                            if ( arlen >= 256 ) {
                                dmar[REG_DMAR_PARAM_ARLEN]  = 0xff;
                                dmar[REG_DMAR_PARAM_ARADDR] = addr + src_offset;
                                arlen -= 256;
                                addr  += 256 * r_unit;
                            }
                            else {
                                dmar[REG_DMAR_PARAM_ARLEN]  = arlen - 1;
                                dmar[REG_DMAR_PARAM_ARADDR] = addr + src_offset;
                                arlen = 0;
                            }
                        }
//                      dmar[REG_DMAR_PARAM_ARLEN]   = (len / 16) - 1;
//                      dmar[REG_DMAR_PARAM_ARADDR]  = addr + src_offset;

                        kunmap(kbuf);
                        src_offset  = 0;
                        src_n      += 1;
                        src_size   -= len;
                    }

                    if ( dst_size > 0 && dmaw[REG_DMAW_CTL_STATUS] == 0 ) {
                        dma_addr_t    addr = page_to_phys(dst_pages[dst_n]);
                        void         *kbuf = kmap(dst_pages[dst_n]);
                        void         *ptr  = (void *)((unsigned long)kbuf + dst_offset);
                        unsigned long len  = PAGE_SIZE - dst_offset;
                        if ( len > dst_size ) { len = dst_size; }
                        arm64_inval_dcache_area(ptr, len);

                        unsigned long awlen = len / w_unit;
                        while ( awlen > 0 ) {
                            while ( dmaw[REG_DMAW_CTL_STATUS] != 0) {
                                udelay(1);
                            }
                            if ( awlen >= 256 ) {
                                dmaw[REG_DMAW_PARAM_AWLEN]  = 0xff;
                                dmaw[REG_DMAW_PARAM_AWADDR] = addr + dst_offset;
                                awlen -= 256;
                                addr  += 256 * w_unit;
                            }
                            else {
                                dmaw[REG_DMAW_PARAM_AWLEN]  = awlen - 1;
                                dmaw[REG_DMAW_PARAM_AWADDR] = addr + dst_offset;
                                awlen = 0;
                            }
                        }
//                      dmaw[REG_DMAW_PARAM_AWLEN]   = (len / 16) - 1;
//                      dmaw[REG_DMAW_PARAM_AWADDR]  = addr + dst_offset;

                        kunmap(kbuf);
                        dst_offset  = 0;
                        dst_n      += 1;
                        dst_size   -= len;
                    }

                    if ( dmar[REG_DMAR_CTL_STATUS] != 0 && dmaw[REG_DMAW_CTL_STATUS] != 0  ) {
                        udelay(10);
                    }
                }
            }

            while ( dmaw[REG_DMAW_CTL_ISSUE_CNT] != 0  ) {
                udelay(1);
            }

            // ページテーブルを返却
            for ( i = 0; i < src_size; i++ ) {
                put_page(src_pages[i]);
            }
            for ( i = 0; i < dst_size; i++ ) {
                put_page(dst_pages[i]);
            }

            mutex_unlock(&dmacalc_mutex);
        }
        break;
    
    default:
        printk(KERN_WARNING "unsupported command %d\n", cmd);
        return -EFAULT;
    }

    return 0;
}


struct file_operations dmacalc_fops = {
    .open    = dmacalc_open,
    .release = dmacalc_close,
    .read    = dmacalc_read,
    .write   = dmacalc_write,
    .unlocked_ioctl = dmacalc_ioctl,
};

static struct class *dmacalc_class = NULL;


static int dmacalc_init(void)
{
    printk("dmacalc_init\n");

    mutex_init(&dmacalc_mutex);

    int alloc_ret = 0;
    int cdev_err = 0;
    dev_t dev;

    alloc_ret = alloc_chrdev_region(&dev, MINOR_BASE, MINOR_NUM, DRIVER_NAME);
    if (alloc_ret != 0) {
        printk(KERN_ERR  "alloc_chrdev_region = %d\n", alloc_ret);
        return -1;
    }

    dmacalc_major = MAJOR(dev);
    dev = MKDEV(dmacalc_major, MINOR_BASE);

    cdev_init(&dmacalc_cdev, &dmacalc_fops);
    dmacalc_cdev.owner = THIS_MODULE;

    cdev_err = cdev_add(&dmacalc_cdev, dev, MINOR_NUM);
    if (cdev_err != 0) {
        printk(KERN_ERR  "cdev_add = %d\n", cdev_err);
        unregister_chrdev_region(dev, MINOR_NUM);
        return -1;
    }

    dmacalc_class = class_create(THIS_MODULE, "fpga-dmacalc");
    if (IS_ERR(dmacalc_class)) {
        printk(KERN_ERR  "class_create\n");
        cdev_del(&dmacalc_cdev);
        unregister_chrdev_region(dev, MINOR_NUM);
        return -1;
    }

    for (int minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) {
        device_create(dmacalc_class, NULL, MKDEV(dmacalc_major, minor), NULL, "fpga-dmacalc%d", minor);
    }

    // メモリマップドレジスタへのアクセス用ページを作りポインタを取得
    dmar    = (unsigned long *)ioremap(DMAR_BASE,  4096);
    dmaw    = (unsigned long *)ioremap(DMAW_BASE,  4096);
//  printk("dmar    : %016lx\n", (unsigned long)dmar);
//  printk("dmaw    : %016lx\n", (unsigned long)dmaw);
//  printk("dmar ID : %08lx\n", dmar[REG_DMAR_CORE_ID]);
//  printk("dmaw ID : %08lx\n", dmaw[REG_DMAW_CORE_ID]);
//  printk("dmar bis : %ld\n", dmar[REG_DMAR_CFG_DATA_BITS]);
//  printk("dmaw bis : %ld\n", dmaw[REG_DMAW_CFG_DATA_BITS]);
//  printk("arm64_read_dcache_line_size : %08lx\n", (unsigned long)arm64_read_dcache_line_size());

    return 0;
}

static void dmacalc_exit(void)
{
    printk("dmacalc_exit\n");

    // メモリマップドレジスタへのアクセス用ページを解放
    iounmap(dmar);
    iounmap(dmaw);

    dev_t dev = MKDEV(dmacalc_major, MINOR_BASE);

    for (int minor = MINOR_BASE; minor < MINOR_BASE + MINOR_NUM; minor++) {
        device_destroy(dmacalc_class, MKDEV(dmacalc_major, minor));
    }

    class_destroy(dmacalc_class);

    cdev_del(&dmacalc_cdev);

    unregister_chrdev_region(dev, MINOR_NUM);
}

module_init(dmacalc_init);
module_exit(dmacalc_exit);

