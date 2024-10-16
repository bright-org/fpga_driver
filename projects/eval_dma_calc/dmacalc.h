
#ifndef __FPGA__DMACALC_H__
#define __FPGA__DMACALC_H__

#include <linux/ioctl.h>

struct dmacalc_calc {
    const float     *src;
    float           *dst;
    unsigned long   size;
};

#define DMACALC_IOC_TYPE    'E'

#define DMACALC_CALC  _IOW(DMACALC_IOC_TYPE, 1, struct dmacalc_calc)


#endif /* __FPGA__DMACALC_H__ */
