/*
 * QEMU PowerPC MPC8560 communication processor module
 *
 * Author : noyecube@gmail.com
 *
 * This is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * *****************************************************************
 *
 * The documentation for this device is noted in the MPC8560 documentation,
 * file name "MPC8560RM.pdf". You can easily find it on the web.
 *
 */

#if !defined (__MPC8560_CPM__)
#define __MPC8560_CPM__
/* ******************** HEADER (INCLUDE) SECTION ******************* */
#include "sysemu/char.h"
#include "qemu/fifo8.h"
/* ************* MACROS, CONSTANTS, COMPILATION FLAGS ************** */
#define BASE_CCSR_TO_CPM(addr)	((addr) - (0x80000))
#define BCTC(addr)	BASE_CCSR_TO_CPM(addr)

#define BASE_CPM_TO_DEFAULT(addr)	((addr) - (0x90000))
#define BCTD(addr)	BASE_CPM_TO_DEFAULT(addr)

#define BASE_CPM_TO_INT(addr)	((addr) - (0x90c00))
#define BCTI(addr)	BASE_CPM_TO_INT(addr)

#define BASE_CPM_TO_SCCX(addr)	((addr) - (0x91a00))
#define BCTS(addr)	BASE_CPM_TO_SCCX(addr)

#define BASE_CPM_TO_CP(addr)	((addr) - (0x919c0))
#define BCTCP(addr)	BASE_CPM_TO_CP(addr)

/* ********** CPM Memory Map ********** */
/* ****** CPM Dual-Port RAM ****** */
#define CPM_BASE_DPRAM1 (0x80000ULL)    /* Dual-port RAM / RW / - */
#define CPM_BASE_DPRAM2 (0x88000ULL)    /* Dual-port RAM / RW / - */
#define CPM_SIZE_DPRAM (0x4000)

/* ****** e500 Core Interface ****** */
#define CPM_REG_CEAR    (0x90000ULL)    /* CPM error address register / R / 0x0000_0000 */
#define CPM_REG_CEER    (0x90004ULL)    /* CPM error event register / RW / 0x0000 */
#define CPM_REG_CEMR    (0x90006ULL)    /* CPM error mask register / RW / 0x0000 */

/* ****** SDMA ****** */
#define CPM_REG_SMAER   (0x90050ULL)    /* System bus address error register / R / 0x0000_0000 */
#define CPM_REG_SMEVR   (0x90058ULL)    /* System bus event register / RW / 0x0000_0000 */
#define CPM_REG_SMCTR   (0x9005cULL)    /* System bus control register / RW / 0x3800_0000 */
#define CPM_REG_LMAER   (0x90060ULL)    /* Local bus address error register / R / 0x0000_0000 */
#define CPM_REG_LMEVR   (0x90068ULL)    /* Local bus event register / RW / 0x0000_0000 */
#define CPM_REG_LMCTR   (0x9006cULL)    /* Local bus control register / RW / 0x3800_0000 */

/* ****** Interrupt Controller ****** */
#define CPM_REG_SICR    (0x90c00ULL)    /* CPM interrupt configuration register / RW / 0x0000_0000 */
#define CPM_REG_SIVEC   (0x90c04ULL)    /* CPM interrupt vector register / RW / 0x0000_0000 */
#define CPM_REG_SIPNR_H (0x90c08ULL)    /* CPM interrupt pending register(high) / RW / 0x0000_0000 */
#define CPM_REG_SIPNR_L (0x90c0cULL)    /* CPM interrupt pending register(low) / RW / 0x0000_0000 */
#define CPM_REG_SCPRR_H (0x90c14ULL)    /* CPM interrupt priority register(high) / RW / 0x0530_9770 */
#define CPM_REG_SCPRR_L (0x90c18ULL)    /* CPM interrupt priority register(low) / RW / 0x0530_9770 */
#define CPM_REG_SIMR_H  (0x90c1cULL)    /* CPM interrupt mask register(high) / RW / 0x0000_0000 */
#define CPM_REG_SIMR_L  (0x90c20ULL)    /* CPM interrupt mask register(low) / RW / 0x0000_0000 */
#define CPM_REG_SIEXR   (0x90c24ULL)    /* CPM external interrupt control register / RW / 0x0000_0000 */

/* ****** Clock ****** */
#define CPM_REG_SCCR    (0x90c80ULL)    /* System clock control register / RW / 0x0000_0001 */

/* ****** Input/Output Port ****** */
#define CPM_REG_PDIRA   (0x90d00ULL)    /* Port A data direction register / RW / 0x0000_0000 */
#define CPM_REG_PPARA   (0x90d04ULL)    /* Port A pin assignment register / RW / 0x0000_0000 */
#define CPM_REG_PSORA   (0x90d08ULL)    /* Port A special options register / RW / 0x0000_0000 */
#define CPM_REG_PODRA   (0x90d0cULL)    /* Port A open drain register / RW / 0x0000_0000 */
#define CPM_REG_PDATA   (0x90d10ULL)    /* Port A data register / RW / 0x0000_0000 */
#define CPM_REG_PDIRB   (0x90d20ULL)    /* Port B data direction register / RW / 0x0000_0000 */
#define CPM_REG_PPARB   (0x90d24ULL)    /* Port B pin assignment register / RW / 0x0000_0000 */
#define CPM_REG_PSORB   (0x90d28ULL)    /* Port B special options register / RW / 0x0000_0000 */
#define CPM_REG_PODRB   (0x90d2cULL)    /* Port B open drain register / RW / 0x0000_0000 */
#define CPM_REG_PDATB   (0x90d30ULL)    /* Port B data register / RW / 0x0000_0000 */
#define CPM_REG_PDIRC   (0x90d40ULL)    /* Port C data direction register / RW / 0x0000_0000 */
#define CPM_REG_PPARC   (0x90d44ULL)    /* Port C pin assignment register / RW / 0x0000_0000 */
#define CPM_REG_PSORC   (0x90d48ULL)    /* Port C special options register / RW / 0x0000_0000 */
#define CPM_REG_PODRC   (0x90d4cULL)    /* Port C open drain register / RW / 0x0000_0000 */
#define CPM_REG_PDATC   (0x90d50ULL)    /* Port C data register / RW / 0x0000_0000 */
#define CPM_REG_PDIRD   (0x90d60ULL)    /* Port D data direction register / RW / 0x0000_0000 */
#define CPM_REG_PPARD   (0x90d64ULL)    /* Port D pin assignment register / RW / 0x0000_0000 */
#define CPM_REG_PSORD   (0x90d68ULL)    /* Port D special options register / RW / 0x0000_0000 */
#define CPM_REG_PODRD   (0x90d6cULL)    /* Port D open drain register / RW / 0x0000_0000 */
#define CPM_REG_PDATD   (0x90d70ULL)    /* Port D data register / RW / 0x0000_0000 */

/* ****** CPM Timers ****** */
#define CPM_REG_TGCR1   (0x90d80ULL)    /* Timer1 and timer2 global configuration register / RW / 0x00 */
#define CPM_REG_TGCR2   (0x90d84ULL)    /* Timer3 and timer4 global configuration register / RW / 0x00 */
#define CPM_REG_TMR1    (0x90d90ULL)    /* Timer 1 mode register / RW / 0x0000 */
#define CPM_REG_TMR2    (0x90d92ULL)    /* Timer 2 mode register / RW / 0x0000 */
#define CPM_REG_TRR1    (0x90d94ULL)    /* Timer 1 reference register / RW / 0x0000 */
#define CPM_REG_TRR2    (0x90d96ULL)    /* Timer 2 reference register / RW / 0x0000 */
#define CPM_REG_TCR1    (0x90d98ULL)    /* Timer 1 capture register / RW / 0x0000 */
#define CPM_REG_TCR2    (0x90d9aULL)    /* Timer 2 capture register / RW / 0x0000 */
#define CPM_REG_TCN1    (0x90d9cULL)    /* Timer 1 counter / RW / 0x0000 */
#define CPM_REG_TCN2    (0x90d9eULL)    /* Timer 2 counter / RW / 0x0000 */
#define CPM_REG_TMR3    (0x90da0ULL)    /* Timer 3 mode register / RW / 0x0000 */
#define CPM_REG_TMR4    (0x90da2ULL)    /* Timer 4 mode register / RW / 0x0000 */
#define CPM_REG_TRR3    (0x90da4ULL)    /* Timer 3 reference register / RW / 0x0000 */
#define CPM_REG_TRR4    (0x90da6ULL)    /* Timer 4 reference register / RW / 0x0000 */
#define CPM_REG_TCR3    (0x90da8ULL)    /* Timer 3 capture register / RW / 0x0000 */
#define CPM_REG_TCR4    (0x90daaULL)    /* Timer 4 capture register / RW / 0x0000 */
#define CPM_REG_TCN3    (0x90dacULL)    /* Timer 3 counter / RW / 0x0000 */
#define CPM_REG_TCN4    (0x90daeULL)    /* Timer 4 counter / RW / 0x0000 */
#define CPM_REG_TER1    (0x90db0ULL)    /* Timer 1 event register / RW / 0x0000 */
#define CPM_REG_TER2    (0x90db2ULL)    /* Timer 2 event register / RW / 0x0000 */
#define CPM_REG_TER3    (0x90db4ULL)    /* Timer 3 event register / RW / 0x0000 */
#define CPM_REG_TER4    (0x90db6ULL)    /* Timer 4 event register / RW / 0x0000 */

/* ****** FCC1 ****** */
#define CPM_REG_GFMR1   (0x91300ULL)    /* FCC1 general mode register / RW / 0x0000_0000 */
#define CPM_REG_FPSMR1  (0x91304ULL)    /* FCC1 protocol-specific mode register / RW / 0x0000_0000 */
#define CPM_REG_FTODR1  (0x91308ULL)    /* FCC1 transmit on demand register / RW / 0x0000 */
#define CPM_REG_FDSR1   (0x9130cULL)    /* FCC1 data synchronization register / RW / 0x7e7e */
#define CPM_REG_FCCE1   (0x91310ULL)    /* FCC1 event register / RW / 0x0000_0000 */
#define CPM_REG_FCCM1   (0x91314ULL)    /* FCC1 mask register / RW / 0x0000_0000 */
#define CPM_REG_FCCS1   (0x91318ULL)    /* FCC1 status register / R / 0x00 */
#define CPM_REG_FTIRR1_PHY0 (0x9131cULL)/* FCC1 transmit internal rate registers / RW / 0x00 */
#define CPM_REG_FTIRR1_PHY1 (0x9131dULL)/* FCC1 transmit internal rate registers / RW / 0x00 */
#define CPM_REG_FTIRR1_PHY2 (0x9131eULL)/* FCC1 transmit internal rate registers / RW / 0x00 */
#define CPM_REG_FTIRR1_PHY3 (0x9131fULL)/* FCC1 transmit internal rate registers / RW / 0x00 */

/* ****** FCC2 ****** */
#define CPM_REG_GFMR2   (0x91320ULL)    /* FCC2 general mode register / RW / 0x0000_0000 */
#define CPM_REG_FPSMR2  (0x91324ULL)    /* FCC2 protocol-specific mode register / RW / 0x0000_0000 */
#define CPM_REG_FTODR2  (0x91328ULL)    /* FCC2 transmit on demand register / RW / 0x0000 */
#define CPM_REG_FDSR2   (0x9132cULL)    /* FCC2 data synchronization register / RW / 0x7e7e */
#define CPM_REG_FCCE2   (0x91330ULL)    /* FCC2 event register / RW / 0x0000_0000 */
#define CPM_REG_FCCM2   (0x91334ULL)    /* FCC2 mask register / RW / 0x0000_0000 */
#define CPM_REG_FCCS2   (0x91338ULL)    /* FCC2 status register / R / 0x00 */
#define CPM_REG_FTIRR2_PHY0 (0x9133cULL)/* FCC2 transmit internal rate registers / RW / 0x00 */
#define CPM_REG_FTIRR2_PHY1 (0x9133dULL)/* FCC2 transmit internal rate registers / RW / 0x00 */
#define CPM_REG_FTIRR2_PHY2 (0x9133eULL)/* FCC2 transmit internal rate registers / RW / 0x00 */
#define CPM_REG_FTIRR2_PHY3 (0x9133fULL)/* FCC2 transmit internal rate registers / RW / 0x00 */

/* ****** FCC3 ****** */
#define CPM_REG_GFMR3   (0x91340ULL)    /* FCC3 general mode register / RW / 0x0000_0000 */
#define CPM_REG_FPSMR3  (0x91344ULL)    /* FCC3 protocol-specific mode register / RW / 0x0000_0000 */
#define CPM_REG_FTODR3  (0x91348ULL)    /* FCC3 transmit on demand register / RW / 0x0000 */
#define CPM_REG_FDSR3   (0x9134cULL)    /* FCC3 data synchronization register / RW / 0x7e7e */
#define CPM_REG_FCCE3   (0x91350ULL)    /* FCC3 event register / RW / 0x0000_0000 */
#define CPM_REG_FCCM3   (0x91354ULL)    /* FCC3 mask register / RW / 0x0000_0000 */
#define CPM_REG_FCCS3   (0x91358ULL)    /* FCC3 status register / R / 0x00 */

/* ****** FCC1(continued) ****** */
#define CPM_REG_FIRPER1 (0x91380ULL)    /* FCC1 internal rate port enable register / RW / 0x0000_0000 */
#define CPM_REG_FIRER1  (0x91384ULL)    /* FCC1 internal rate event register / RW / 0x0000_0000 */
#define CPM_REG_FIRSR1_HI   (0x91388ULL)/* FCC1 internal rate selection register:HI / RW / 0x0000_0000 */
#define CPM_REG_FIRSR1_LO   (0x9138cULL)/* FCC1 internal rate selection register:LO / RW / 0x0000_0000 */
#define CPM_REG_GFEMR1  (0x91390ULL)    /* General FCC1 expansion mode register / RW / 0x00 */

/* ****** FCC2(continued) ****** */
#define CPM_REG_FIRPER2 (0x913a0ULL)    /* FCC2 internal rate port enable register / RW / 0x0000_0000 */
#define CPM_REG_FIRER2  (0x913a4ULL)    /* FCC2 internal rate event register / RW / 0x0000_0000 */
#define CPM_REG_FIRSR2_HI   (0x913a8ULL)/* FCC2 internal rate selection register:HI / RW / 0x0000_0000 */
#define CPM_REG_FIRSR2_LO   (0x913acULL)/* FCC2 internal rate selection register:LO / RW / 0x0000_0000 */
#define CPM_REG_GFEMR2  (0x913b0ULL)    /* General FCC2 expansion mode register / RW / 0x00 */

/* ****** FCC3(continued) ****** */
#define CPM_REG_GFEMR3  (0x913d0ULL)    /* General FCC3 expansion mode register / RW / 0x00 */

/* ****** TC Layer 1 ****** */
#define CPM_REG_TCMODE1 (0x91400ULL)    /* TC1 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR1  (0x91402ULL)    /* TC1 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER1   (0x91404ULL)    /* TC1 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC1 (0x91406ULL)    /* TC1 received cell counter */
#define CPM_REG_TCMR1   (0x91408ULL)    /* TC1 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC1 (0x9140aULL)    /* TC1 filtered cell counter */
#define CPM_REG_TC_CCC1 (0x9140cULL)    /* TC1 corrected cell counter */
#define CPM_REG_TC_ICC1 (0x9140eULL)    /* TC1 idle cell counter */
#define CPM_REG_TC_TCC1 (0x91410ULL)    /* TC1 transmitted cell counter */
#define CPM_REG_TC_ECC1 (0x91412ULL)    /* TC1 error cell counter */

/* ****** TC Layer 2 ****** */
#define CPM_REG_TCMODE2 (0x91420ULL)    /* TC2 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR2  (0x91422ULL)    /* TC2 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER2   (0x91424ULL)    /* TC2 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC2 (0x91426ULL)    /* TC2 received cell counter */
#define CPM_REG_TCMR2   (0x91428ULL)    /* TC2 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC2 (0x9142aULL)    /* TC2 filtered cell counter */
#define CPM_REG_TC_CCC2 (0x9142cULL)    /* TC2 corrected cell counter */
#define CPM_REG_TC_ICC2 (0x9142eULL)    /* TC2 idle cell counter */
#define CPM_REG_TC_TCC2 (0x91430ULL)    /* TC2 transmitted cell counter */
#define CPM_REG_TC_ECC2 (0x91432ULL)    /* TC2 error cell counter */

/* ****** TC Layer 3 ****** */
#define CPM_REG_TCMODE3 (0x91440ULL)    /* TC3 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR3  (0x91442ULL)    /* TC3 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER3   (0x91444ULL)    /* TC3 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC3 (0x91446ULL)    /* TC3 received cell counter */
#define CPM_REG_TCMR3   (0x91448ULL)    /* TC3 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC3 (0x9144aULL)    /* TC3 filtered cell counter */
#define CPM_REG_TC_CCC3 (0x9144cULL)    /* TC3 corrected cell counter */
#define CPM_REG_TC_ICC3 (0x9144eULL)    /* TC3 idle cell counter */
#define CPM_REG_TC_TCC3 (0x91450ULL)    /* TC3 transmitted cell counter */
#define CPM_REG_TC_ECC3 (0x91452ULL)    /* TC3 error cell counter */

/* ****** TC Layer 4 ****** */
#define CPM_REG_TCMODE4 (0x91460ULL)    /* TC4 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR4  (0x91462ULL)    /* TC4 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER4   (0x91464ULL)    /* TC4 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC4 (0x91466ULL)    /* TC4 received cell counter */
#define CPM_REG_TCMR4   (0x91468ULL)    /* TC4 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC4 (0x9146aULL)    /* TC4 filtered cell counter */
#define CPM_REG_TC_CCC4 (0x9146cULL)    /* TC4 corrected cell counter */
#define CPM_REG_TC_ICC4 (0x9146eULL)    /* TC4 idle cell counter */
#define CPM_REG_TC_TCC4 (0x91470ULL)    /* TC4 transmitted cell counter */
#define CPM_REG_TC_ECC4 (0x91472ULL)    /* TC4 error cell counter */

/* ****** TC Layer 5 ****** */
#define CPM_REG_TCMODE5 (0x91480ULL)    /* TC5 mode register */
#define CPM_REG_CDSMR5  (0x91482ULL)    /* TC5 cell delineation state machine register */
#define CPM_REG_TCER5   (0x91484ULL)    /* TC5 event register */
#define CPM_REG_TC_RCC5 (0x91486ULL)    /* TC5 received cell counter */
#define CPM_REG_TCMR5   (0x91488ULL)    /* TC5 mask register */
#define CPM_REG_TC_FCC5 (0x9148aULL)    /* TC5 filtered cell counter */
#define CPM_REG_TC_CCC5 (0x9148cULL)    /* TC5 corrected cell counter */
#define CPM_REG_TC_ICC5 (0x9148eULL)    /* TC5 idle cell counter */
#define CPM_REG_TC_TCC5 (0x91490ULL)    /* TC5 transmitted cell counter */
#define CPM_REG_TC_ECC5 (0x91492ULL)    /* TC5 error cell counter */

/* ****** TC Layer 6 ****** */
#define CPM_REG_TCMODE6 (0x914a0ULL)    /* TC6 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR6  (0x914a2ULL)    /* TC6 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER6   (0x914a4ULL)    /* TC6 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC6 (0x914a6ULL)    /* TC6 received cell counter */
#define CPM_REG_TCMR6   (0x914a8ULL)    /* TC6 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC6 (0x914aaULL)    /* TC6 filtered cell counter */
#define CPM_REG_TC_CCC6 (0x914acULL)    /* TC6 corrected cell counter */
#define CPM_REG_TC_ICC6 (0x914aeULL)    /* TC6 idle cell counter */
#define CPM_REG_TC_TCC6 (0x914b0ULL)    /* TC6 transmitted cell counter */
#define CPM_REG_TC_ECC6 (0x914b2ULL)    /* TC6 error cell counter */

/* ****** TC Layer 7 ****** */
#define CPM_REG_TCMODE7 (0x914c0ULL)    /* TC7 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR7  (0x914c2ULL)    /* TC7 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER7   (0x914c4ULL)    /* TC7 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC7 (0x914c6ULL)    /* TC7 received cell counter */
#define CPM_REG_TCMR7   (0x914c8ULL)    /* TC7 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC7 (0x914caULL)    /* TC7 filtered cell counter */
#define CPM_REG_TC_CCC7 (0x914ccULL)    /* TC7 corrected cell counter */
#define CPM_REG_TC_ICC7 (0x914ceULL)    /* TC7 idle cell counter */
#define CPM_REG_TC_TCC7 (0x914d0ULL)    /* TC7 transmitted cell counter */
#define CPM_REG_TC_ECC7 (0x914d2ULL)    /* TC7 error cell counter */

/* ****** TC Layer 8 ****** */
#define CPM_REG_TCMODE8 (0x914e0ULL)    /* TC8 mode register / 0x0000 / 0x0000 */
#define CPM_REG_CDSMR8  (0x914e2ULL)    /* TC8 cell delineation state machine register / 0x0000 / 0x0000 */
#define CPM_REG_TCER8   (0x914e4ULL)    /* TC8 event register / 0x0000 / 0x0000 */
#define CPM_REG_TC_RCC8 (0x914e6ULL)    /* TC8 received cell counter */
#define CPM_REG_TCMR8   (0x914e8ULL)    /* TC8 mask register / 0x0000 / 0x0000 */
#define CPM_REG_TC_FCC8 (0x914eaULL)    /* TC8 filtered cell counter */
#define CPM_REG_TC_CCC8 (0x914ecULL)    /* TC8 corrected cell counter */
#define CPM_REG_TC_ICC8 (0x914eeULL)    /* TC8 idle cell counter */
#define CPM_REG_TC_TCC8 (0x914f0ULL)    /* TC8 transmitted cell counter */
#define CPM_REG_TC_ECC8 (0x914f2ULL)    /* TC8 error cell counter */

/* ****** TC Layer - General ****** */
#define CPM_REG_TCGSR   (0x91500ULL)    /* TC general status register / RW / 0x0000 */
#define CPM_REG_TCGER   (0x91502ULL)    /* TC general event register / RW / 0x0000 */

/* ****** BRGs 5-8 ****** */
#define CPM_REG_BRGC5   (0x915f0ULL)    /* BRG5 configuration register / RW / 0x0000_0000 */
#define CPM_REG_BRGC6   (0x915f4ULL)    /* BRG6 configuration register / RW / 0x0000_0000 */
#define CPM_REG_BRGC7   (0x915f8ULL)    /* BRG7 configuration register / RW / 0x0000_0000 */
#define CPM_REG_BRGC8   (0x915fcULL)    /* BRG8 configuration register / RW / 0x0000_0000 */

/* ****** I2C ****** */
#define CPM_REG_I2MOD   (0x91860ULL)    /* I2C mode register / RW / 0x00 */
#define CPM_REG_I2ADD   (0x91864ULL)    /* I2C address register / RW / 0x00 */
#define CPM_REG_II2BRG  (0x91868ULL)    /* I2C BRG register / RW / 0x00 */
#define CPM_REG_I2COM   (0x9186cULL)    /* I2C command register / RW / 0x00 */
#define CPM_REG_I2CER   (0x91870ULL)    /* I2C event register / RW / 0x00 */
#define CPM_REG_II2CMR  (0x91874ULL)    /* I2C mask register / RW / 0x00 */

/* ****** Communications Processor ****** */
#define CPM_REG_CPCR    (0x919c0ULL)    /* Communications processor command register / RW / 0x0000_0000 */
#define CPM_REG_RCCR    (0x919c4ULL)    /* CP configuration register / RW / 0x0000_0000 */
#define CPM_REG_RTER    (0x919d6ULL)    /* CP timers event register / RW / 0x0000 */
#define CPM_REG_RTMR    (0x919daULL)    /* CP timers mask register / RW / 0x0000 */
#define CPM_REG_RTSCR   (0x919dcULL)    /* CP time-stamp timer control register / RW / 0x0000 */
#define CPM_REG_RTSR    (0x919e0ULL)    /* CP time-stamp register / RW / 0x0000_0000 */

/* ****** BRGs 1-4 ****** */
#define CPM_REG_BRGC1   (0x919f0ULL)    /* BRG1 configuration register / RW / 0x0000_0000 */
#define CPM_REG_BRGC2   (0x919f4ULL)    /* BRG2 configuration register / RW / 0x0000_0000 */
#define CPM_REG_BRGC3   (0x919f8ULL)    /* BRG3 configuration register / RW / 0x0000_0000 */
#define CPM_REG_BRGC4   (0x919fcULL)    /* BRG4 configuration register / RW / 0x0000_0000 */

/* ****** SCC1 ****** */
#define CPM_REG_GSMR_L1 (0x91a00ULL)    /* SCC1 general mode register / RW / 0x0000_0000 */
#define CPM_REG_GSMR_H1 (0x91a04ULL)    /* SCC1 general mode register / RW / 0x0000_0000 */
#define CPM_REG_PSMR1   (0x91a08ULL)    /* SCC1 protocol-specific mode register / RW / 0x0000 */
#define CPM_REG_TODR1   (0x91a0cULL)    /* SCC1 transmit-on-demand register / W / 0x0000 */
#define CPM_REG_DSR1    (0x91a0eULL)    /* SCC1 data synchronization register / RW / 0x7e7e */
#define CPM_REG_SCCE1   (0x91a10ULL)    /* SCC1 event register / RW / 0x0000 */
#define CPM_REG_RESV1	(0x91a12ULL)	/* SCC1 errata reserved */
#define CPM_REG_SCCM1   (0x91a14ULL)    /* SCC1 mask register / RW / 0x0000 */
#define CPM_REG_SCCS1   (0x91a17ULL)    /* SCC1 status register / RW / 0x00 */

/* ****** SCC2 ****** */
#define CPM_REG_GSMR_L2 (0x91a20ULL)    /* SCC2 general mode register / RW / 0x0000_0000 */
#define CPM_REG_GSMR_H2 (0x91a24ULL)    /* SCC2 general mode register / RW / 0x0000_0000 */
#define CPM_REG_PSMR2   (0x91a28ULL)    /* SCC2 protocol-specific mode register / RW / 0x0000 */
#define CPM_REG_TODR2   (0x91a2cULL)    /* SCC2 transmit-on-demand register / W / 0x0000 */
#define CPM_REG_DSR2    (0x91a2eULL)    /* SCC2 data synchronization register / RW / 0x7e7e */
#define CPM_REG_SCCE2   (0x91a30ULL)    /* SCC2 event register / RW / 0x0000 */
#define CPM_REG_SCCM2   (0x91a34ULL)    /* SCC2 mask register / RW / 0x0000 */
#define CPM_REG_SCCS2   (0x91a37ULL)    /* SCC2 status register / RW / 0x00 */

/* ****** SCC3 ****** */
#define CPM_REG_GSMR_L3 (0x91a40ULL)    /* SCC3 general mode register / RW / 0x0000_0000 */
#define CPM_REG_GSMR_H3 (0x91a44ULL)    /* SCC3 general mode register / RW / 0x0000_0000 */
#define CPM_REG_PSMR3   (0x91a48ULL)    /* SCC3 protocol-specific mode register / RW / 0x0000 */
#define CPM_REG_TODR3   (0x91a4cULL)    /* SCC3 transmit-on-demand register / W / 0x0000 */
#define CPM_REG_DSR3    (0x91a4eULL)    /* SCC3 data synchronization register / RW / 0x7e7e */
#define CPM_REG_SCCE3   (0x91a50ULL)    /* SCC3 event register / RW */
#define CPM_REG_SCCM3   (0x91a54ULL)    /* SCC3 mask register / RW / 0x0000 */
#define CPM_REG_SCCS3   (0x91a57ULL)    /* SCC3 status register / RW / 0x00 */

/* ****** SCC4 ****** */
#define CPM_REG_GSMR_L4 (0x91a60ULL)    /* SCC4 general mode register / RW / 0x0000_0000 */
#define CPM_REG_GSMR_H4 (0x91a64ULL)    /* SCC4 general mode register / RW / 0x0000_0000 */
#define CPM_REG_PSMR4   (0x91a68ULL)    /* SCC4 protocol-specific mode register / RW / 0x0000 */
#define CPM_REG_TODR4   (0x91a6cULL)    /* SCC4 transmit-on-demand register / W / 0x0000 */
#define CPM_REG_DSR4    (0x91a6eULL)    /* SCC4 data synchronization register / RW / 0x7e7e */
#define CPM_REG_SCCE4   (0x91a70ULL)    /* SCC4 event register / RW */
#define CPM_REG_SCCM4   (0x91a74ULL)    /* SCC4 mask register / RW / 0x0000 */
#define CPM_REG_SCCS4   (0x91a77ULL)    /* SCC4 status register / RW / 0x00 */

/* ****** SPI ****** */
#define CPM_REG_SPMODE  (0x91aa0ULL)    /* SPI mode register / RW / 0x0000 */
#define CPM_REG_SPIE    (0x91aa6ULL)    /* SPI event register / RW / 0x00 */
#define CPM_REG_SPIM    (0x91aaaULL)    /* SPI mask register / RW / 0x00 */
#define CPM_REG_SPCOM   (0x91aadULL)    /* SPI command register / W / 0x00 */

/* ****** CPM Mux ****** */
#define CPM_REG_CMXSI1CR (0x91b00ULL)   /* CPM mux SI1 clock route register / RW / 0x00 */
#define CPM_REG_CMXSI2CR (0x91b02ULL)   /* CPM mux SI2 clock route register / RW / 0x00 */
#define CPM_REG_CMXFCR  (0x91b04ULL)    /* CPM mux FCC clock route register / RW / 0x0000_0000 */
#define CPM_REG_CMXSCR  (0x91b08ULL)    /* CPM mux SCC clock route register / RW / 0x0000_0000 */
#define CPM_REG_CMXUAR  (0x91b0eULL)    /* CPM mux UTOPIA address register / RW / 0x0000 */

/* ****** SI1 Registers ****** */
#define CPM_REG_SI1AMR  (0x91b20ULL)    /* SI1 TDMA1 mode register / RW / 0x0000 */
#define CPM_REG_SI1BMR  (0x91b22ULL)    /* SI1 TDMB1 mode register / RW / 0x0000 */
#define CPM_REG_SI1CMR  (0x91b24ULL)    /* SI1 TDMC1 mode register / RW / 0x0000 */
#define CPM_REG_SI1DMR  (0x91b26ULL)    /* SI1 TDMD1 mode register / RW / 0x0000 */
#define CPM_REG_SI1GMR  (0x91b28ULL)    /* SI1 global mode register / RW / 0x00 */
#define CPM_REG_SI1CMDR (0x91b2aULL)    /* SI1 command register / RW / 0x00 */
#define CPM_REG_SI1STR  (0x91b2cULL)    /* SI1 status register / RW / 0x00 */
#define CPM_REG_SI1RSR  (0x91b2eULL)    /* SI1 RAM shadow address register / RW / 0x0000 */

/* ****** MCC1 Registers ****** */
#define CPM_REG_MCCE1   (0x91b30ULL)    /* MCC1 event register / RW / 0x0000 */
#define CPM_REG_MCCM1   (0x91b34ULL)    /* MCC1 mask register / RW / 0x0000 */
#define CPM_REG_MCCF1   (0x91b38ULL)    /* MCC1 configuration register / RW / 0x00 */

/* ****** SI2 Registers ****** */
#define CPM_REG_SI2AMR  (0x91b40ULL)    /* SI2 TDMA1 mode register / RW / 0x0000 */
#define CPM_REG_SI2BMR  (0x91b42ULL)    /* SI2 TDMB1 mode register / RW / 0x0000 */
#define CPM_REG_SI2CMR  (0x91b44ULL)    /* SI2 TDMC1 mode register / RW / 0x0000 */
#define CPM_REG_SI2DMR  (0x91b46ULL)    /* SI2 TDMD1 mode register / RW / 0x0000 */
#define CPM_REG_SI2GMR  (0x91b48ULL)    /* SI2 global mode register / RW / 0x00 */
#define CPM_REG_SI2CMDR (0x91b4aULL)    /* SI2 command register / RW / 0x00 */
#define CPM_REG_SI2STR  (0x91b4cULL)    /* SI2 status register / RW / 0x00 */
#define CPM_REG_SI2RSR  (0x91b4eULL)    /* SI2 RAM shadow address register / RW / 0x0000 */

/* ****** MCC2 Registers ****** */
#define CPM_REG_MCCE2   (0x91b50ULL)    /* MCC2 event register / RW / 0x0000 */
#define CPM_REG_MCCM2   (0x91b54ULL)    /* MCC2 mask register / RW / 0x0000 */
#define CPM_REG_MCCF2   (0x91b58ULL)    /* MCC2 configuration register / RW / 0x00 */

/* ****** SI1 RAM ****** */
#define CPM_BASE_SI1TxRAM (0x92000ULL)  /* SI1 transmit routing RAM */
#define CPM_SIZE_SI1TxRAM (0x200)
#define CPM_BASE_SI1RxRAM (0x92400ULL)  /* SI1 receive routing RAM */
#define CPM_SIZE_SI1RxRAM (0x200)

/* ****** SI2 RAM ****** */
#define CPM_BASE_SI2TxRAM (0x92800ULL)  /* SI2 transmit routing RAM */
#define CPM_SIZE_SI2TxRAM (0x200)
#define CPM_BASE_SI2RxRAM (0x92c00ULL)  /* SI2 receive routing RAM */
#define CPM_SIZE_SI2RxRAM (0x200)

/* ****** Instruction RAM ****** */
#define CPM_BASE_DPINSTRAM  (0xa0000ULL)    /* Dual-port RAM(instruction RAM only) / / undef */
#define CPM_SIZE_DPINSTRAM  (0x8000)

#define SCC_TXRX_FIFO_LEN	(32)

/* ***************** STRUCTURES, TYPE DEFINITIONS ****************** */
typedef enum
{
	PL_HIGHEST =	1,
	PL_XCC_1	=	2,
	PL_XCC_2	=	3,
	PL_XCC_3	=	4,
	PL_XCC_4	=	5,
	PL_XCC_5	=	6,
	PL_XCC_6	=	7,
	PL_XCC_7	=	8,
	PL_XCC_8	=	9,
	PL_YCC_G1	=	10,
	PL_YCC_G2	=	11,
	PL_YCC_G3	=	12,
	PL_YCC_G4	=	13,
	PL_YCC_G5	=	14,
	PL_YCC_G6	=	15,
	PL_YCC_G7	=	16,
	PL_YCC_G8	=	17,
	PL_IOPC_15	=	18,
	PL_TIMER_1	=	19,
	PL_IOPC_14	=	20,
	PL_YCC_S1	=	21,
	PL_IOPC_13	=	22,

	PL_YCC_S2	=	25,
	PL_IOPC_12	=	26,
	PL_IOPC_11	=	27,

	PL_TIMER_2	=	29,
	PL_IOPC_10	=	30,
	PL_YCC_S3	=	31,
	PL_RISC_TB	=	32,
	PL_I2C		=	33,
	PL_YCC_S4	=	34,
	PL_IOPC_9	=	35,
	PL_IOPC_8	=	36,

	PL_TIMER_3	=	38,
	PL_YCC_S5	=	39,
	PL_IOPC_7	=	40,
	PL_IOPC_6	=	41,
	PL_IOPC_5	=	42,
	PL_TIMER_4	=	43,

	PL_YCC_S6	=	44,
	PL_IOPC_4	=	45,

	PL_SPI		=	47,
	PL_IOPC_3	=	48,
	PL_IOPC_2	=	49,
	
	PL_YCC_S7	=	51,
	
	PL_IOPC_1	=	53,
	PL_IOPC_0	=	54,
	PL_YCC_S8	=	55

} TYPE_INT_SRC_PRI;	/* Interrupt Source Priority Levels */

typedef enum
{
	SC_SCC_1	=	0x04,
	SC_SCC_2	=	0x05,
	SC_SCC_3	=	0x06,
	SC_SCC_4	=	0x07,
	SC_SPI		=	0x0a,
	SC_I2C		=	0x0b,
	SC_RAND		=	0x0e,
	SC_TIMER	=	0x0f,
	SC_FCC_1	=	0x10,
	SC_FCC_2	=	0x11,
	SC_FCC_3	=	0x12,
	SC_MCC_1	=	0x1c,
	SC_MCC_2	=	0x1d

} TYPE_CPCR_SBC_CODE; /* CPCR SBC Code */

/** CP Command Opcodes **/
typedef enum
{
	OP_INIT_RX_TX_PARAMS	=	0x00,
	OP_INIT_RX_PARAMS		=	0x01,
	OP_INIT_TX_PARAMS		=	0x02,
	OP_ENTER_HUNT_MODE		=	0x03,
	OP_STOP_TX				=	0x04,
	OP_GRACEFUL_STOP_TX		=	0x05,
	OP_RESTART_TX			=	0x06,
	OP_SET_GROUP_ADDRESS	=	0x08,
	OP_RESET_BCS			=	0x0a

} TYPE_CP_CMD_SCC;


/* ********** Parameter RAM Offset ********** */
typedef enum
{
	PG_SCC_1	=	0x88000,
	PG_SCC_2	=	0x88100,
	PG_SCC_3	=	0x88200,
	PG_SCC_4	=	0x88300,
	PG_FCC_1	=	0x88400,
	PG_FCC_2	=	0x88500,
	PG_FCC_3	=	0x88600
	/* TODO : will be continued */
} TYPE_PRAM_PAGE; /* Parameter RAM Page Address Offset */

typedef enum
{
	SR_RBASE	=	0x00,	/* The DPRAM location of the first RBD */
	SR_TBASE	=	0x02,	
	SR_RFCR		=	0x04,	/* Function Code for Receive DMA */
	SR_TFCR		=	0x05,
	SR_MRBLR	=	0x06,	/* Max Receive Buffer Length */
	SR_RSTATE	=	0x08,	/* RCV State information about channel */
	SR_R_PTR	=	0x0c,	/* Pointer to next memory write location */
	SR_RBPTR	=	0x10,	/* Pointer to current/next BD location */
	SR_R_CNT	=	0x12,	/* Down count to end of frame or Buffer */
	SR_RTEMP	=	0x14,
	SR_TSTATE	=	0x18,
	SR_T_PTR	=	0x1c,
	SR_TBPTR	=	0x20,
	SR_T_CNT	=	0x22,
	SR_TTEMP	=	0x24,
	SR_R_CRC	=	0x28,	/* Current Receive CRC */
	SR_T_CRC	=	0x2c

} TYPE_SCC_PRAM_OFFSET; /* SCC Parameter RAM Offset */

typedef struct CPMState CPMState;
struct CPMState
{
	/*< private >*/
	SysBusDevice parent_obj;

	/*< public >*/
	qemu_irq irq;	/* pin out to openpic */
	
	CharDriverState* serial_scc1;
	Fifo8 recv_fifo_scc1;
	uint8 recv_fifo_itl_scc1;	/* Interrupt trigger level for recv_fifo */
	QEMUTimer* fifo_timeout_timer;
	QEMUTimer* scc_status_poll;

	MemoryRegion cpm_space;

	/* Dual-Port RAM */
	MemoryRegion dpram1;
	MemoryRegion dpram2;

	/* SIx RAM */

	/* Instruction RAM */
	MemoryRegion dpinstram;

	/* Regions for MMIO */
	MemoryRegion whole_mmio;
	MemoryRegion intctrl; 	/* Interrupt Controller */
	MemoryRegion cp_mmio;
	MemoryRegion sccx;		/* SCCx */

	/* FIXME START: temporary, remove later this region */
	MemoryRegion cpm_dpram_temp;
	MemoryRegion cpinstemp;
	
	hwaddr cur_rxbd_addr;
	uint16_t bd_status;
	uint16_t bd_dlen;
//	uint32_t bd_pbuf;
//	uint32_t bd_buffer;

	/* FIXME END */

	/* Parameter RAM variables */

	/* e500 Core Interface */
	uint32_t cear;
	uint16_t ceer;
	uint16_t cemr;

	/* SDMA */
	/* Interrupt Controller */
	uint32_t sicr;
	uint32_t sivec;
	uint32_t sipnr_h;
	uint32_t sipnr_l;
	uint32_t scprr_h;
	uint32_t scprr_l;
	uint32_t simr_h;
	uint32_t simr_l;
	uint32_t siexr;

	/* Clock */
	uint32_t sccr;

	/* Input/Output Port */
	uint32_t pdirc;	/*	(0x90d40ULL) Port C data direction register / RW / 0x0000_0000 */
	uint32_t pparc; /*	(0x90d44ULL) Port C pin assignment register / RW / 0x0000_0000 */
	uint32_t psorc; /*	(0x90d48ULL) Port C special options register / RW / 0x0000_0000 */
	uint32_t podrc; /*	(0x90d4cULL) Port C open drain register / RW / 0x0000_0000 */
	uint32_t pdatc; /*	(0x90d50ULL) Port C data register / RW / 0x0000_0000 */
	uint32_t pdird;	/*  (0x90d60ULL) Port D data direction register / RW / 0x0000_0000 */
	uint32_t ppard; /*  (0x90d64ULL) Port D pin assignment register / RW / 0x0000_0000 */
	uint32_t psord; /*  (0x90d68ULL) Port D special options register / RW / 0x0000_0000 */
	uint32_t podrd; /*  (0x90d6cULL) Port D open drain register / RW / 0x0000_0000 */
	uint32_t pdatd; /*  (0x90d70ULL) Port D data register / RW / 0x0000_0000 */

	/* Timers */
	/* FCC 1 ~ 3 */
	/* TC Layer 1 ~ 8, General */
	/* I2C */
	/* BRGs 1 ~ 8 */
	uint32_t brgc1;	/*	(0x919f0ULL) BRG1 configuration register / RW / 0x0000_0000 */
	uint32_t brgc2; /*  (0x919f4ULL) BRG2 configuration register / RW / 0x0000_0000 */
	uint32_t brgc3; /*  (0x919f8ULL) BRG3 configuration register / RW / 0x0000_0000 */
	uint32_t brgc4; /*	(0x919fcULL) BRG4 configuration register / RW / 0x0000_0000 */

	/* Communications Processor */
	uint32_t cpcr;
	uint32_t rccr;
	uint16_t rter;
	uint16_t rtmr;
	uint16_t rtscr;
	uint16_t rtsr;

	/* SCC1 */
	uint32_t gsmr_l1;
	uint32_t gsmr_h1;
	uint16_t psmr1;
	uint16_t todr1;
	uint16_t dsr1;
	uint16_t scce1;
	uint16_t scc_resv1; /* Errata Rev. 1 */
	uint16_t sccm1;
	uint8_t sccs1;

	/* SCC2 */
	uint32_t gsmr_l2;
	uint32_t gsmr_h2;
	uint16_t psmr2;
	uint16_t todr2;
	uint16_t dsr2;
	uint16_t scce2;
	uint16_t sccm2;
	uint8_t sccs2;

	/* SCC3 */
	/* SCC4 */

	/* ****** CPM Mux ****** */
	uint8_t cmxsi1cr; /* (0x91b00ULL) CPM mux SI1 clock route register / RW / 0x00 */
	uint8_t cmxsi2cr; /* (0x91b02ULL) CPM mux SI2 clock route register / RW / 0x00 */
	uint32_t cmxfcr;  /* (0x91b04ULL) CPM mux FCC clock route register / RW / 0x0000_0000 */
	uint32_t cmxscr;  /* (0x91b08ULL) CPM mux SCC clock route register / RW / 0x0000_0000 */
	uint32_t cmxuar;  /* (0x91b0eULL) CPM mux UTOPIA address register / RW / 0x0000 */
};
#define TYPE_MPC8560_CPM    ("mpc8560-cpm")
#define MPC8560_CPM(obj)    OBJECT_CHECK(CPMState, (obj), TYPE_MPC8560_CPM)

void mpc8560_cpm_init_serial(CPMState* s, CharDriverState* chr);

#endif /* !defined (__MPC8560_CPM__) */
