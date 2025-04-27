/*
 * xintf.c
 *
 *  Created on: 2024年1月16日
 *      Author: Suzkfly
 */
#include "xintf.h"

#pragma CODE_SECTION(xintf_zone6and7_timing,"ramfuncs");
void xintf_zone6and7_timing()
{
    // All Zones---------------------------------
    // Timing for all zones based on XTIMCLK = SYSCLKOUT / 2
    XintfRegs.XINTCNF2.bit.XTIMCLK = 1;
    // Buffer up to 3 writes
    XintfRegs.XINTCNF2.bit.WRBUFF = 3;
    // XCLKOUT is enabled
    XintfRegs.XINTCNF2.bit.CLKOFF = 1;      /* 为1时关闭XCLKOUT信号 */
    // XCLKOUT = XTIMCLK
    XintfRegs.XINTCNF2.bit.CLKMODE = 0;


    // Zone 6------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING6.bit.XWRLEAD = 1;
    XintfRegs.XTIMING6.bit.XWRACTIVE = 1;
    XintfRegs.XTIMING6.bit.XWRTRAIL = 1;
    // Zone read timing
    XintfRegs.XTIMING6.bit.XRDLEAD = 1;
    XintfRegs.XTIMING6.bit.XRDACTIVE = 2;
    XintfRegs.XTIMING6.bit.XRDTRAIL = 0;

    // do not double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING6.bit.X2TIMING = 0;

    // Zone will not sample READY
    XintfRegs.XTIMING6.bit.USEREADY = 0;
    XintfRegs.XTIMING6.bit.READYMODE = 0;

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING6.bit.XSIZE = 3;


    // Zone 7------------------------------------
    // When using ready, ACTIVE must be 1 or greater
    // Lead must always be 1 or greater
    // Zone write timing
    XintfRegs.XTIMING7.bit.XWRLEAD = 1;
    XintfRegs.XTIMING7.bit.XWRACTIVE = 1;
    XintfRegs.XTIMING7.bit.XWRTRAIL = 1;
    // Zone read timing
    XintfRegs.XTIMING7.bit.XRDLEAD = 1;
    XintfRegs.XTIMING7.bit.XRDACTIVE = 2;
    XintfRegs.XTIMING7.bit.XRDTRAIL = 0;

    // don't double all Zone read/write lead/active/trail timing
    XintfRegs.XTIMING7.bit.X2TIMING = 0;

    // Zone will not sample XREADY signal
    XintfRegs.XTIMING7.bit.USEREADY = 0;
    XintfRegs.XTIMING7.bit.READYMODE = 0;

    // Size must be 1,1 - other values are reserved
    XintfRegs.XTIMING7.bit.XSIZE = 3;

   //Force a pipeline flush to ensure that the write to
   //the last register configured occurs before returning.
   asm(" RPT #7 || NOP");

   /***********************  0~ 20  *****************************/
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");


   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   /***********************  20~ 40  *****************************/
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");


   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   /***********************  40~ 60  *****************************/
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   /***********************  60~ 80  *****************************/
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   /***********************  80~ 100  *****************************/
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");

   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
   asm("   nop");
}
