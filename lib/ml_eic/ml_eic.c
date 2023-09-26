/*
 * Author: Ben Westcott
 * Date created: 7/31/23
 */
#include <ml_eic.h>
#include <ml_clocks.h>

void eic_init(void)
{
    ML_SET_GCLK7_PCHCTRL(EIC_GCLK_ID);

    EIC->CTRLA.reg &= ~EIC_CTRLA_ENABLE;
    EIC->CTRLA.reg |= EIC_CTRLA_SWRST;
    while(EIC->SYNCBUSY.bit.ENABLE & EIC->SYNCBUSY.bit.SWRST);

    EIC->CTRLA.reg &= ~EIC_CTRLA_CKSEL;

    // Set all EXTINTs to trigger syncrhonously
    // I believe this is default, i.e., this register is 
    // initialized to a value of 0x0
    // EIC->ASYNCH.reg &= ~EIC_ASYNCH_ASYNCH(0);
}

void eic_enable(void)
{
    EIC->CTRLA.bit.ENABLE = 1;
    while(EIC->SYNCBUSY.bit.ENABLE);
}