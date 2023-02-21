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

/* ******************** HEADER (INCLUDE) SECTION ******************* */
#include "qemu-common.h"
#include "qemu/timer.h"
#include "hw/hw.h"
#include "sysemu/sysemu.h"
#include "e500-ccsr.h"
#include "hw/sysbus.h"
#include "hw/ppc/mpc8560_cpm.h"

/* ************* MACROS, CONSTANTS, COMPILATION FLAGS ************** */
//#define DEBUG_CPM
#ifdef DEBUG_CPM
#define DPRINTF( fmt, args... )	\
	fprintf( stderr, "MPC8560-cpm(%s:%d) " fmt ".\n", \
			__func__, __LINE__, ##args)
#else
#define DPRINTF( fmt, args... )
#endif /* DEBUG_CPM */

/**
 * @fn PARTIAL_MASKING(base, var, addr, size, umask, shifter)
 * @brief
 *
 */
#define PARTIAL_MASKING(base, var, addr, size, umask, shifter) do {		\
	if( (addr == base) && (sizeof(var) == size) )	\
		break;	\
	shifter = ((sizeof(umask) - sizeof(var)) << 3);	\
	umask = umask >> shifter;	\
	shifter = ((addr - base) << 3);	\
	umask = umask >> shifter;	\
	shifter = ((sizeof(var) - size) - (addr - base)) << 3;	\
	} while(0)

/**
 * @fn PARTIAL_READ(base, var, addr, size, ret)
 * @brief
 *
 */
#define PARTIAL_READ(base, var, addr, size, ret) do {	\
	uint32_t umask = 0xffffffff;	\
	uint8_t shifter = 0;	\
	PARTIAL_MASKING(base, var, addr, size, umask, shifter);	\
	ret = (var & (umask << shifter)) >> shifter;	\
	} while(0)

/**
 * @fn PARTIAL_WRITE(base, var, addr, size, value)
 * @brief
 *
 */
#define PARTIAL_WRITE(base, var, addr, size, value) do {		\
	uint32_t umask = 0xffffffff;	\
	uint8_t	shifter = 0;	\
	PARTIAL_MASKING(base, var, addr, size, umask, shifter);	\
	value = value & umask;	\
	var = var & ~(umask << shifter);	\
	var = var | (value << shifter);	\
	} while(0)

/**
 * @fn PARTIAL_WRITE_REVERSE(base, var, addr, size, value)
 * @brief
 *
 */
#define PARTIAL_WRITE_REVERSE(base, var, addr, size, value) do {	\
	uint32_t umask = 0xffffffff;	\
	uint8_t shifter = 0;	\
	PARTIAL_MASKING(base, var, addr, size, umask, shifter);	\
	umask = umask & value;	\
	var = var & ~(umask << shifter);	\
	} while(0)

#define MPC8560_CCSRBAR_BASE	(0xE1000000ULL)
#define IVPR_MASK_SHIFT		(31)
#define IVPR_MASK_MASK		(1 << IVPR_MASK_SHIFT)

#define SCCE_TX_SHIFT		(1)
#define SCCE_TX_MASK		(1 << SCCE_TX_SHIFT)
#define SCCE_RX_MASK		(1)

#define CPCR_RST_SHIFT		(31)
#define CPCR_RST_MASK		((1) << (CPCR_RST_SHIFT))
#define CPCR_PAGE_SHIFT		(26)
#define CPCR_PAGE_MASK		((0x1f) << (CPCR_PAGE_SHIFT))
#define CPCR_SBC_SHIFT		(21)
#define CPCR_SBC_MASK		((0x1f) << (CPCR_SBC_SHIFT))
#define CPCR_FLG_SHIFT		(16)
#define CPCR_FLG_MASK		((1) << (CPCR_FLG_SHIFT))
#define CPCR_OPCODE_MASK	(0xf)
/* ***************** STRUCTURES, TYPE DEFINITIONS ****************** */

/* ********************* FUNCTION PROTOTYPES *********************** */

/* ************************* CODE SECTION ************************** */
static void mpc8560_cpm_reset_direct(void* opaque)
{
	CPMState* s = (CPMState*)opaque;
	uint32_t ret;

	/* recv_fifo trigger level */
	s->recv_fifo_itl_scc1 = 1;
	
	/* Interrupt controller */
	s->sicr	= 0x0;
	s->sivec = 0x0;
	s->sipnr_h = 0x0;
	s->sipnr_l = 0x0;
	s->scprr_h = 0x05309770;
	s->scprr_l = 0x05309770;
	s->simr_h = 0x0;
	s->simr_l = 0x0;
	s->siexr = 0x0;

	/* Clock */

	/* ** Input/Output Port ** */
	/* Port C */
	s->pdirc = 0x0;
	s->pparc = 0x0;
	s->psorc = 0x0;
	s->podrc = 0x0;
	s->pdatc = 0x0;

	/* Port D */
	s->pdird = 0x0;
	s->ppard = 0x0;
	s->psord = 0x0;
	s->podrd = 0x0;
	s->pdatd = 0x0;

	/* BRGs 1 ~ 8 */
	s->brgc1 = 0x0; /*  (0x919f0ULL) BRG1 configuration register / RW / 0x0000_0000 */
	s->brgc2 = 0x0; /*  (0x919f4ULL) BRG2 configuration register / RW / 0x0000_0000 */
	s->brgc3 = 0x0; /*  (0x919f8ULL) BRG3 configuration register / RW / 0x0000_0000 */
	s->brgc4 = 0x0; /*  (0x919fcULL) BRG4 configuration register / RW / 0x0000_0000 */

	/* Communications processor */
	s->cpcr = 0x0;
	s->rccr = 0x0;
	s->rter = 0x0;
	s->rtmr = 0x0;
	s->rtscr = 0x0;
	s->rtsr = 0x0;

	/* ** SCCx ** */
	s->gsmr_l1 = 0x0;
	s->gsmr_h1 = 0x0;
	s->psmr1 = 0x0;
	s->todr1 = 0x0;
	s->dsr1 = 0x7e7e;
	s->scce1 = 0x0;
	s->scc_resv1 = 0x0; /* Errata Rev. 1 */
	s->sccm1 = 0x0;
	s->sccs1 = 0x0;

	s->gsmr_l2 = 0x0;
	s->gsmr_h2 = 0x0;
	s->psmr2 = 0x0;
	s->todr2 = 0x0;
	s->dsr2 = 0x7e7e;
	s->scce2 = 0x0;
	s->sccm2 = 0x0;
	s->sccs2 = 0x0;

	/* ****** CPM Mux ****** */
	s->cmxsi1cr = 0x0;	/* (0x91b00ULL) CPM mux SI1 clock route register / RW / 0x00 */
	s->cmxsi2cr = 0x0;	/* (0x91b02ULL) CPM mux SI2 clock route register / RW / 0x00 */
	s->cmxfcr = 0x0;	/* (0x91b04ULL) CPM mux FCC clock route register / RW / 0x0000_0000 */
	s->cmxscr = 0x0;	/* (0x91b08ULL) CPM mux SCC clock route register / RW / 0x0000_0000 */
	s->cmxuar = 0x0;	/* (0x91b0eULL) CPM mux UTOPIA address register / RW / 0x0000 */
	
	/** Enable IRQ_30 from OpenPIC */
	/* Reading IIVPR30(Internal interrupt 30 vector/priority register) */
	ret = ldl_phys(MPC8560_CCSRBAR_BASE + 0x505c0);
	/* Mask interrupts */
	ret = ret & ~IVPR_MASK_MASK;
	DPRINTF("writing value %d to IIVPR30", ret);
	stl_phys(MPC8560_CCSRBAR_BASE + 0x505c0, ret);
	
	/* IIDR30(Internal interrupt 30 destination register) */
	ret = ldl_phys(MPC8560_CCSRBAR_BASE + 0x505d0);
	ret = ret | (1); /* ( 1 << cpu ) -> number of CPU? */
	DPRINTF("writing value %d to IIDR30", ret);
	stl_phys(MPC8560_CCSRBAR_BASE + 0x505d0, ret);
	
	return ;	
}

static void mpc8560_cpm_reset(DeviceState* d)
{
	CPMState* s = MPC8560_CPM(d);
	DPRINTF("HIT");
	mpc8560_cpm_reset_direct(s);

	return ;
}

/**
 *
 *
 *
 */
static uint64_t mpc8560_cpm_default_read(void* opaque, hwaddr addr, unsigned size)
{
	uint32_t ret = 0;
	CPMState* s = (CPMState*)opaque;
	
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;

	switch( addr ) {
	/* Port C */
	case BCTD(CPM_REG_PDIRC):	
		ret = s->pdirc;	
		break;
	case BCTD(CPM_REG_PPARC):
		ret = s->pparc;
		break;
	case BCTD(CPM_REG_PSORC):
		ret = s->psorc;
		break;
	case BCTD(CPM_REG_PODRC):
		ret = s->podrc;
		break;
	case BCTD(CPM_REG_PDATC):
		ret = s->pdatc;
		break;
	
	/* Port D */
	case BCTD(CPM_REG_PDIRD):
		ret = s->pdird;
		break;
	case BCTD(CPM_REG_PPARD):
		ret = s->ppard;
		break;
	case BCTD(CPM_REG_PSORD):
		ret = s->psord;
		break;
	case BCTD(CPM_REG_PODRD):
		ret = s->podrd;
		break;
	case BCTD(CPM_REG_PDATD):
		ret = s->pdatd;
		break;
		/* BRGs 1 - 4 */	
	case BCTD(CPM_REG_BRGC1):
		ret = s->brgc1;
		break;
	case BCTD(CPM_REG_BRGC2):
		ret = s->brgc2;
		break;
	case BCTD(CPM_REG_BRGC3):
		ret = s->brgc3;
		break;
	case BCTD(CPM_REG_BRGC4):
		ret = s->brgc4;
		break;
	
	/* ****** CPM Mux ****** */	
	case BCTD(CPM_REG_CMXSI1CR): /* (0x91b00ULL) CPM mux SI1 clock route register / RW / 0x00 */
		ret = s->cmxsi1cr;
		break;
	case BCTD(CPM_REG_CMXSI2CR): /* (0x91b02ULL) CPM mux SI2 clock route register / RW / 0x00 */
		ret = s->cmxsi2cr;
		break;
	case BCTD(CPM_REG_CMXFCR):	/* (0x91b04ULL) CPM mux FCC clock route register / RW / 0x0000_0000 */
		ret = s->cmxfcr;
		break;
	case BCTD(CPM_REG_CMXSCR):	/* (0x91b08ULL) CPM mux SCC clock route register / RW / 0x0000_0000 */
		ret = s->cmxscr;
		break;
	case BCTD(CPM_REG_CMXUAR):	/* (0x91b0eULL) CPM mux UTOPIA address register / RW / 0x0000 */
		ret = s->cmxuar;
		break;
	
	default :
		fprintf( stderr, "%s:%d] Unknown register raed: %x\n",
				__func__, __LINE__, (int)addr);
	}
	
	DPRINTF("opaque= %p, addr= %x, size= %x ret=%x", opaque, (uint32_t)addr, size, ret);
	
	return ret;
}

/**
 *
 *
 *
 */
static void mpc8560_cpm_default_write(void* opaque, hwaddr addr, uint64_t value, unsigned size)
{
	CPMState* s = (CPMState*)opaque;
	
	DPRINTF("opaque= %p, addr= %x, value= %lx size=%x", opaque, (uint32_t)addr, value, size);
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;
	/* TODO : scale 'value' according to size?? */	
	switch( addr ) {
	/* Port C */
	case BCTD(CPM_REG_PDIRC):
		s->pdirc = value;
		DPRINTF("pdirc -> %x", s->pdirc);
		break;
	case BCTD(CPM_REG_PPARC):
		s->pparc = value;
		DPRINTF("pparc -> %x", s->pparc);
		break;
	case BCTD(CPM_REG_PSORC):
		s->psorc = value;
		DPRINTF("psorc -> %x", s->psorc);
		break;
	case BCTD(CPM_REG_PODRC):
		s->podrc = value;
		DPRINTF("podrc -> %x", s->podrc);
		break;
	case BCTD(CPM_REG_PDATC):
		s->pdatc = value;
		DPRINTF("pdatc -> %x", s->pdatc);
		break;
		/* Port D */
	case BCTD(CPM_REG_PDIRD):
		s->pdird = value;
		DPRINTF("pdird -> %x", s->pdird);
		break;
	case BCTD(CPM_REG_PPARD):
		s->ppard = value;
		DPRINTF("ppard -> %x", s->ppard);
		break;
	case BCTD(CPM_REG_PSORD):
		s->psord = value;
		DPRINTF("psord -> %x", s->psord);
		break;
	case BCTD(CPM_REG_PODRD):
		s->podrd = value;
		DPRINTF("podrd -> %x", s->podrd);
		break;
	case BCTD(CPM_REG_PDATD):
		s->pdatd = value;
		DPRINTF("pdatd -> %x", s->pdatd);
		break;
	
	/* BRGs 1 - 4 */	
	case BCTD(CPM_REG_BRGC1):
		s->brgc1 = value;
		DPRINTF("brgc1 -> %x", s->brgc1);
		break;
	case BCTD(CPM_REG_BRGC2):
		s->brgc2 = value;
		DPRINTF("brgc2 -> %x", s->brgc2);
		break;
	case BCTD(CPM_REG_BRGC3):
		s->brgc3 = value;
		DPRINTF("brgc3 -> %x", s->brgc3);
		break;
	case BCTD(CPM_REG_BRGC4):
		s->brgc4 = value;
		DPRINTF("brgc4 -> %x", s->brgc4);
		break;
	
	/* ****** CPM Mux ****** */	
	case BCTD(CPM_REG_CMXSI1CR): /* (0x91b00ULL) CPM mux SI1 clock route register / RW / 0x00 */
		value = value & 0xff;
		s->cmxsi1cr = value;
		DPRINTF("cmxsi1cr -> %x", s->cmxsi1cr);
		break;
	case BCTD(CPM_REG_CMXSI2CR): /* (0x91b02ULL) CPM mux SI2 clock route register / RW / 0x00 */
		value = value & 0xff;
		s->cmxsi2cr = value;
		DPRINTF("cmxsi2cr -> %x", s->cmxsi2cr);
		break;
	case BCTD(CPM_REG_CMXFCR):	/* (0x91b04ULL) CPM mux FCC clock route register / RW / 0x0000_0000 */
		s->cmxfcr = value;
		DPRINTF("cmxfcr -> %x", s->cmxfcr);
		break;
	case BCTD(CPM_REG_CMXSCR):	/* (0x91b08ULL) CPM mux SCC clock route register / RW / 0x0000_0000 */
		s->cmxscr = value;
		DPRINTF("cmxscr -> %x", s->cmxscr);
		break;
	case BCTD(CPM_REG_CMXUAR):	/* (0x91b0eULL) CPM mux UTOPIA address register / RW / 0x0000 */
		value = value & 0xffff;
		s->cmxuar = value;
		DPRINTF("cmxuar -> %x", s->cmxuar);
		break;
	
	default :
		fprintf( stderr, "%s:%d] Unknown register raed: %x\n",
				__func__, __LINE__, (int)addr);
	}
	return ;
}

/**
 *
 *
 *
 */
static uint64_t mpc8560_cpm_intctrl_read(void* opaque, hwaddr addr, unsigned size)
{
	uint32_t ret = 0;
	CPMState* s = (CPMState*)opaque;

	//addr &= MPC8560_CPM_MMIO_SIZE - 1;
	switch( addr ) {
	case BCTI(CPM_REG_SICR):
		ret = s->sicr;
		break;
	case BCTI(CPM_REG_SIVEC):
		PARTIAL_READ(BCTI(CPM_REG_SIVEC), s->sivec, addr, size, ret);
		break;
	case BCTI(CPM_REG_SIPNR_H):
		ret = s->sipnr_h;
		break;
	case BCTI(CPM_REG_SIPNR_L):
		ret = s->sipnr_l;
		break;
	case BCTI(CPM_REG_SCPRR_H):
		ret = s->scprr_h;
		break;
	case BCTI(CPM_REG_SCPRR_L):
		ret = s->scprr_l;
		break;
	case BCTI(CPM_REG_SIMR_H):
		ret = s->simr_h;
		break;
	case BCTI(CPM_REG_SIMR_L):
		ret = s->simr_l;
		break;
	case BCTI(CPM_REG_SIEXR):
		ret = s->siexr;
		default :
		fprintf( stderr, "%s:%d] Unknown register raed: %x\n",
				__func__, __LINE__, (int)addr);
	}

	DPRINTF("opaque= %p, addr= %x, size= %x, ret=%x", opaque, (uint32_t)addr, size, ret );
	return ret;
}

/**
 *
 *
 *
 */
static void mpc8560_cpm_intctrl_write(void* opaque, hwaddr addr, uint64_t value, unsigned size)
{
	CPMState* s = (CPMState*)opaque;
	
	/* FIXME: check size before value assign?? */
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;
	DPRINTF("opaque= %p, addr= %x, value= %lx size=%x", opaque, (uint32_t)addr, value, size);
	switch( addr ) {
	case BCTI(CPM_REG_SICR):
		s->sicr	= value;
		DPRINTF("sicr -> %x", s->sicr);
		break;
	case BCTI(CPM_REG_SIVEC):
		PARTIAL_WRITE(BCTI(CPM_REG_SIVEC), s->sivec, addr, size, value);
		DPRINTF("sivec -> %x", s->sivec);
		break;
	case BCTI(CPM_REG_SIPNR_H): /* SIPNR bits are cleared by writing onces to them */
		s->sipnr_h = s->sipnr_h & ~(value);
		DPRINTF("sipnr_h -> %x", s->sipnr_h);
		break;
	case BCTI(CPM_REG_SIPNR_L):
		s->sipnr_l = s->sipnr_l & ~(value);
		DPRINTF("sipnr_l -> %x", s->sipnr_l);
		break;
	case BCTI(CPM_REG_SCPRR_H):
		s->scprr_h = value;
		DPRINTF("scprr_h -> %x", s->scprr_h);
		break;
	case BCTI(CPM_REG_SCPRR_L):
		s->scprr_l = value;
		DPRINTF("scprr_l -> %x", s->scprr_l);
		break;
	case BCTI(CPM_REG_SIMR_H):
		s->simr_h = value;
		DPRINTF("simr_h -> %x", s->simr_h);
		break;
	case BCTI(CPM_REG_SIMR_L):
		s->simr_l = value;
		DPRINTF("simr_l -> %x", s->simr_l);
		/* TODO : If the user sets the SIMR bit later, a previously pending interrupt request
		   		is processed by the core, according to its assigned priority.
		*/
		break;
	case BCTI(CPM_REG_SIEXR):
		s->siexr = value;
		DPRINTF("siexr -> %x", s->siexr);
	
	default :
		fprintf( stderr, "%s:%d] Unknown register raed: %x\n",
				__func__, __LINE__, (int)addr);
	}

	return ;
}


/**
 *
 *
 *
 */
static uint64_t mpc8560_cpm_cp_read(void* opaque, hwaddr addr, unsigned size)
{
	uint32_t ret = 0;
	CPMState* s = (CPMState*)opaque;
	
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;
	switch( addr ) {
	case BCTCP(CPM_REG_CPCR):
		ret = s->cpcr;
		break;
	case BCTCP(CPM_REG_RCCR):
		ret = s->rccr;
		break;
	case BCTCP(CPM_REG_RTER):
		ret = s->rter;
		break;
	case BCTCP(CPM_REG_RTMR):
		ret = s->rtmr;
		break;
	case BCTCP(CPM_REG_RTSCR):
		ret = s->rtscr;
		break;
	case BCTCP(CPM_REG_RTSR):
		ret = s->rtsr;
		break;
	default :
		fprintf( stderr, "%s:%d] Unknown register raed: %x\n",
				__func__, __LINE__, (int)addr);
	}
	
	DPRINTF("opaque= %p, addr= %x, size= %x ret=%x", opaque, (uint32_t)addr, size, ret);
	return ret;
}

/**
 * @fn static void mpc8560_cpm_scc_init_params_txrx(void* opaque)
 * @brief
 *
 */
static void mpc8560_cpm_scc_init_params_txrx(void* opaque)
{
	CPMState* s = (CPMState*)opaque;
	uint32_t t_val;
	uint32_t page_idx = (s->cpcr & CPCR_PAGE_MASK) >> CPCR_PAGE_SHIFT;
	const hwaddr t_base = MPC8560_CCSRBAR_BASE + CPM_BASE_DPRAM2 + (0x100 * page_idx);
	
	/* Check page for SCC */
	if (page_idx > 0x3)	{
		fprintf( stderr, "%s:%d] Wrong PAGE(%d) for SCC(0 - 3).\n",
				__func__, __LINE__, page_idx);
		return ;
	} else { /* QAC */ }
	
	/* Init TX parameters */
	t_val = lduw_phys(t_base + SR_TBASE);
	stw_phys((t_base + SR_TBPTR), t_val);
	stl_phys((t_base + SR_TSTATE), 0x0);

	/* Init RX parameters */
	t_val = lduw_phys(t_base + SR_RBASE);
	stw_phys((t_base + SR_RBPTR), t_val);
	stl_phys((t_base + SR_RSTATE), 0x0);
	s->cur_rxbd_addr = MPC8560_CCSRBAR_BASE + CPM_BASE_DPRAM1 + t_val;

	DPRINTF("Initialized for page %d", page_idx);

	return ;
}

/**
 * @fn static void mpc8560_cpm_cp_cpcr_scc_op(void* opaque, TYPE_CPCR_SBC_CODE op)
 * @brief
 *
 */
static void mpc8560_cpm_cp_cpcr_scc_op(void* opaque)
{
	CPMState* s = (CPMState*)opaque;
	TYPE_CP_CMD_SCC op = s->cpcr & CPCR_OPCODE_MASK;

	/* Parsing OPCODE and process */
	switch( op ) {
	case OP_INIT_RX_TX_PARAMS:
		DPRINTF("INIT_RX_TX_PARAMS");
		mpc8560_cpm_scc_init_params_txrx(opaque);
		break;
	case OP_INIT_RX_PARAMS:
		/* Copies RBASE to RBPTR and sets RSTATE to zero */
		break;
	case OP_INIT_TX_PARAMS:
		/* Copies TBASE to TBPTR and sets TSTATE to zero */
		break;
	case OP_ENTER_HUNT_MODE:
		/* Issues a command to the channel to look for an IDLE or FLAG
		   and ignore all incoming data */
		break;
	case OP_STOP_TX:
		/* Tells the various transmit routines to take
		   requests but not send any more data */
		break;
	case OP_GRACEFUL_STOP_TX:
		/* Tells the various transmit routines to transmit to the end
		   of the current buffer/frame and then perform a STOP TX */
		break;
	case OP_RESTART_TX:
		/* Reverse the operation of STOP TX */
		break;
	case OP_SET_GROUP_ADDRESS:
		break;
	case OP_RESET_BCS:
		break;
	default:
		fprintf(stderr, "%s:%d] Unkown OPCODE.\n", __func__, __LINE__ );
	}

	return ;
}

/**
 * @fn static void mpc8560_cpm_cp_cpcr_cb(void* opaque)
 * @brief
 *
 */
static void mpc8560_cpm_cp_cpcr_cb(void* opaque)
{
	CPMState* s = (CPMState*)opaque;
	TYPE_CPCR_SBC_CODE sbc = (s->cpcr & CPCR_SBC_MASK) >> CPCR_SBC_SHIFT;

	/* Check RST and call mpc8560_cpm_reset() and clear RST, FLG both and return ? */
	if (s->cpcr & CPCR_RST_MASK) {
		mpc8560_cpm_reset_direct(opaque);
		s->cpcr = s->cpcr & ~(CPCR_RST_MASK | CPCR_FLG_MASK);

		return ; /* FIXME: confirm data sheet whether RST return immediately or not */
	} else { /* QAC */ }

	/* Parsing Sub-block code and process */
	switch( sbc ) {
	case SC_SCC_1:
	case SC_SCC_2:
	case SC_SCC_3:
	case SC_SCC_4:
		mpc8560_cpm_cp_cpcr_scc_op(opaque);
		break;
	/* TODO: continued */
	
	default :
		fprintf( stderr, "%s:%d] Unkown SBC.\n", __func__, __LINE__ );
	}

	
	/* Clears FLG after completing the comamnd */
	s->cpcr = s->cpcr & ~(CPCR_FLG_MASK);

	return ;
}

/**
 *
 *
 *
 */
static void mpc8560_cpm_cp_write(void* opaque, hwaddr addr, uint64_t value, unsigned size)
{
	CPMState* s = (CPMState*)opaque;
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;
	DPRINTF("opaque= %p, addr= %x, value= %lx size=%x", opaque, (uint32_t)addr, value, size);
	switch( addr ) {
		case BCTCP(CPM_REG_CPCR):
		s->cpcr = value;
		/* TODO : */
		DPRINTF("cpcr -> %x", s->cpcr);
		mpc8560_cpm_cp_cpcr_cb(opaque);
		break;
	case BCTCP(CPM_REG_RCCR):
		s->rccr = value;
		DPRINTF("rccr -> %x", s->rccr);
		break;
	case BCTCP(CPM_REG_RTER):
		s->rter = value;
		DPRINTF("rter -> %x", s->rter);
		break;
	case BCTCP(CPM_REG_RTMR):
		s->rtmr = value;
		DPRINTF("rtmr -> %x", s->rtmr);
		break;
	case BCTCP(CPM_REG_RTSCR):
		s->rtscr = value;
		DPRINTF("rtscr -> %x", s->rtscr);
		break;
	case BCTCP(CPM_REG_RTSR):
		s->rtsr = value;
		DPRINTF("rtsr -> %x", s->rtsr);
		break;
	default :
		fprintf( stderr, "%s:%d] Unknown register read: %x\n",
					__func__, __LINE__, (int)addr);
	}
	return ;
}

/**
 * @fn static void mpc8560_cpm_set_irq(void* opaque)
 * @brief Interrupt Request Masking
 *
 */
static void mpc8560_cpm_set_irq(void* opaque)
{
	CPMState* s = (CPMState*)opaque;
	uint16_t scce_masked;
	/* (SCCE & SCCM) -> | INPUT -> (SIPNR & SIMR) -> Request to the core */
	/* FIXME : process pending interrupts according to priority. currently only SCC1 */
	s->sccm1 = s->sccm1 | (SCCE_TX_MASK | SCCE_RX_MASK); /* FIXME : temp. not initialized from USER */
	/* Masking SCCE with SCCM */
	scce_masked = s->scce1 & s->sccm1;
	if ((scce_masked & SCCE_TX_MASK) || (scce_masked & SCCE_RX_MASK))
		s->sipnr_l = s->sipnr_l | 0x00800000;
	else { /* QAC */ }
	if (!((s->sipnr_l & s->simr_l) & 0x00800000)) {
		DPRINTF("TX or RX interrupts not allowed. SCCE %x SCCM %x sipnr_l %x simr_l %x", 
				s->scce1, s->sccm1, s->sipnr_l, s->simr_l);
		/* FIXME : clear SCCE[TX] ? */
		return ;
	} else { /* QAC */ }
	
	/* TODO : Setting SCC1's Interrupt Vector here ? */
	s->sivec = 0xa0000000; // 0x0b101000;

	DPRINTF("IRQ RAISE~~~~~~");	
	qemu_irq_raise(s->irq);
	
	return ;
}

/**
 * @fn static void mpc8560_cpm_frame_tx_cb(void* opaque, TPYE_PRAM_PAGE page)
 * @brief handler for TX Ready	
 * @opaque - 
 * @page - 
 */
static void mpc8560_cpm_frame_tx_cb(void* opaque, TYPE_PRAM_PAGE page)
{
	CPMState* s = (CPMState*)opaque;

	/* Copies buffer length from current TxBD to t_cnt, and
	   copies starting address to t_ptr.
	 */
	hwaddr scc_tbptr = MPC8560_CCSRBAR_BASE + page + SR_TBPTR;
	hwaddr txbd = MPC8560_CCSRBAR_BASE +  CPM_BASE_DPRAM1 + lduw_phys( scc_tbptr );
	int32_t t_cnt = lduw_phys( txbd + 0x2 );
	hwaddr t_ptr = ldl_phys( txbd + 0x4 );
	uint32_t ch;
	
	DPRINTF("txbd = %lx, t_cnt = %d, t_ptr = %lx", txbd, t_cnt, t_ptr);
	while (t_cnt > 0) {
		ch = ldub_phys( t_ptr );
		DPRINTF("Read ch = %c(0x%x) from addr = %lx, t_cnt = %d", (uint8_t)ch, (uint8_t)ch, t_ptr, t_cnt);
		write( 1, &ch, 1 );
		
		/* Decrements t_cnt and increments t_ptr */
		t_cnt--;
		t_ptr++;
	}
	
	/* Clears R bit after using TxBD */
	s->bd_status = s->bd_status & ~(0x8000);

	/* Set SCCE[TX] if I bit enabled */
	if (s->bd_status & 0x1000) {
	//	s->bd_status = s->bd_status & ~(0x1000); // temp inthandlr should do this

		switch( page ) {
		case PG_SCC_1:
			s->scce1 = s->scce1 | SCCE_TX_MASK;
			/* TODO : SIVEC should be touched after this modification */
			mpc8560_cpm_set_irq(opaque);
			break;
		default:
			fprintf( stderr, "%s:%d] Unkown page.\n", __func__, __LINE__ );
		}
	}
	
	return ;
}

/**
 * @fn static void mpc8560_cpm_frame_rx_cb(void* opaque, TPYE_PRAM_PAGE page)
 * @brief handler for RX Ready	
 * @opaque - 
 * @page - 
 */
static void mpc8560_cpm_frame_rx_cb(void* opaque, TYPE_PRAM_PAGE page)
{
	CPMState* s = (CPMState*)opaque;
	
	hwaddr scc_rbptr = MPC8560_CCSRBAR_BASE + page + SR_RBPTR;
	//hwaddr rxbd = MPC8560_CCSRBAR_BASE +  CPM_BASE_DPRAM1 + lduw_phys( scc_rbptr );
	hwaddr rxbd = s->cur_rxbd_addr;
	uint16_t rxbd_status = lduw_phys(rxbd);
	//int32_t r_cnt = lduw_phys(rxbd + 0x2);
	const uint16_t mrblr = lduw_phys(MPC8560_CCSRBAR_BASE + page + SR_MRBLR); /* maximum receive buffer length */
	int32_t r_cnt;
	hwaddr r_ptr;
	uint8_t ch;
	
	/* CPM does not use this BD while E = 0 */
	while (!(rxbd_status & 0x8000)) {
		if (rxbd_status & 0x2000) {
			/* TODO : init rxbd with first RxBD and check E bit */
			s->cur_rxbd_addr = MPC8560_CCSRBAR_BASE +  CPM_BASE_DPRAM1 + lduw_phys( scc_rbptr ); // temp
			return ; /* no more available BD in Dual port ram */
		} else { /* QAC */ }
		rxbd = rxbd + 0x8;
		s->cur_rxbd_addr = rxbd;
		rxbd_status = lduw_phys( rxbd );
	}

	/* Copies buffer length to R_CNT, and copies starting address to R_PTR */
	//r_cnt = s->recv_fifo_scc1.num;
	r_cnt = 0;
	r_ptr = ldl_phys( rxbd + 0x4 );
	
	/* Decrements R_CNT and increments R_PTR */
	while (s->recv_fifo_scc1.num > 0) {
		ch = fifo8_pop( &(s->recv_fifo_scc1) );
		stb_phys( r_ptr, (uint32_t)ch );

		r_cnt++;
		r_ptr++;
		stw_phys( rxbd + 0x2, r_cnt );
		
		if (r_cnt == mrblr) /* MRBLR = N bytes for this SCC */
		{
			if( s->recv_fifo_scc1.num > 0 ) // more data 
			{
				/* no more available BD */
				if(rxbd_status & 0x2000) {
					/* TODO : init rxbd with first RxBD and check E bit  */
					/* FIXME : timer here or scce rx clear time? */
					qemu_mod_timer( s->scc_status_poll, qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() / 50 ); // temp
					break;
				} else { /* QAC */ }
				rxbd_status = rxbd_status & ~(0x8000);
				stw_phys( rxbd, rxbd_status );
				/* TODO : should check I bit for current BD or last BD only? */
				/* get next bd and fill */
				rxbd = rxbd + 0x8;
				s->cur_rxbd_addr = rxbd;
				rxbd_status = lduw_phys( rxbd );
				r_cnt = 0;
				r_ptr = ldl_phys( rxbd + 0x4 );
				continue;
			} 	else { /* QAC */ }
		} 	else { /* QAC */ }
	}
	
	if (rxbd_status & 0x1000) {
		switch( page ) {
		case PG_SCC_1:
			s->scce1 = s->scce1 | SCCE_RX_MASK;
			mpc8560_cpm_set_irq(opaque);
			break;
		default:
			fprintf( stderr, "%s:%d] Unkown page.\n", __func__, __LINE__ );
		}
	}	else { /* QAC */ }

	rxbd_status = rxbd_status & ~(0x8000);
	stw_phys(rxbd, rxbd_status);

	if (rxbd_status & 0x2000)
		s->cur_rxbd_addr = MPC8560_CCSRBAR_BASE +  CPM_BASE_DPRAM1 + lduw_phys( scc_rbptr ); // temp
	else
		s->cur_rxbd_addr += 0x8; // temp

	return ;
}

static inline void scc_update_status_rxbd(CPMState* s)
{
	if (!fifo8_is_empty(&(s->recv_fifo_scc1)))
		mpc8560_cpm_frame_rx_cb( s, PG_SCC_1 ); 
	
	return ;
}

static int scc_can_receive(void* opaque)
{
//	CPMState* s = (CPMState*)opaque;
	int ret = 0;
	DPRINTF("HIT"); // etjotest
//	if (s->recv_fifo_scc1.num < SCC_TXRX_FIFO_LEN)
//		ret = s->recv_fifo_itl_scc1 - s->recv_fifo_scc1.num; 
	else { /* QAC */ }
	ret = 1; // temp
	return ret;
}

static void scc_update_timer_cb(CPMState* s)
{
	DPRINTF("HIT");	// etjotest
	qemu_del_timer( s->scc_status_poll );

	/* TODO : check poll mode and set poll flag */

	/* TODO : write code here */

	/* RxBD Empty */
	scc_update_status_rxbd( s );
	/* FIXME : setup response latency with proper value */
	//if( polling_enabled )
	if (0)
	qemu_mod_timer( s->scc_status_poll, qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() / 100 ); /* 10ms */
	
	return ;
}

static void scc_fifo_timeout_int(CPMState* s)
{
	return ;
}

static void scc_receive(void* opaque, const uint8_t* buf, int size)
{
	CPMState* s = (CPMState*)opaque;
	int i;
	DPRINTF("buf (%x) size %d", buf[0], size ); // etjotest
	for( i = 0; i < size; i++ ) {
		if( !fifo8_is_full(&(s->recv_fifo_scc1)) )
			fifo8_push(&(s->recv_fifo_scc1), buf[i]);
		else
			do { /* TODO: indicate overrun error */ } while(0);
	}
	
	/* Set Empty bit of RxBD */
	qemu_mod_timer( s->scc_status_poll, qemu_get_clock_ns(vm_clock) + get_ticks_per_sec() / 100 ); /* 10ms */
	
	return ;
}

static void scc_event(void* opaque, int event)
{
//	CPMState* s = (CPMState*)opaque;

	DPRINTF("event %x", event);
#if 0	
	/* push null byte into the fifo */
	if( event == CHR_EVENT_BREAK )
	{
		if( !fifo8_is_full(&(s->recv_fifo_scc1)) )
			fifo8_push( &(s->recv_fifo_scc1), '\0' );
		else
			do { /* TODO : indicate overrun error */ } while(0);
	}
#endif
	return ;
}

/**
 *
 *
 *
 */
static uint64_t mpc8560_cpm_sccx_read(void* opaque, hwaddr addr, unsigned size)
{
	uint32_t ret = 0;
	CPMState* s = (CPMState*)opaque;
	
	
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;

	switch( addr ) {
	/* SCC1 */
	case BCTS(CPM_REG_GSMR_L1): case BCTS(CPM_REG_GSMR_L1) + 0x1:
	case BCTS(CPM_REG_GSMR_L1) + 0x2: case BCTS(CPM_REG_GSMR_L1) + 0x3:
		PARTIAL_READ(BCTS(CPM_REG_GSMR_L1), s->gsmr_l1, addr, size, ret);
		break;
	case BCTS(CPM_REG_GSMR_H1): case BCTS(CPM_REG_GSMR_H1) + 0x1:
	case BCTS(CPM_REG_GSMR_H1) + 0x2: case BCTS(CPM_REG_GSMR_H1) + 0x3:
		PARTIAL_READ(BCTS(CPM_REG_GSMR_H1), s->gsmr_h1, addr, size, ret);
		break;
	case BCTS(CPM_REG_PSMR1):
	case BCTS(CPM_REG_PSMR1) + 0x1:
		PARTIAL_READ(BCTS(CPM_REG_PSMR1), s->psmr1, addr, size, ret);
		break;
	case BCTS(CPM_REG_TODR1):
		PARTIAL_READ(BCTS(CPM_REG_TODR1), s->todr1, addr, size, ret);
		break;
	case BCTS(CPM_REG_DSR1):
		ret = s->dsr1;
		break;
	case BCTS(CPM_REG_SCCE1):
	case BCTS(CPM_REG_SCCE1) + 0x1:
		PARTIAL_READ(BCTS(CPM_REG_SCCE1), s->scce1, addr, size, ret);
		break;
	case BCTS(CPM_REG_RESV1):
	case (BCTS(CPM_REG_RESV1) + 0x1):
		PARTIAL_READ(BCTS(CPM_REG_RESV1), s->scc_resv1, addr, size, ret);
		break;
	case BCTS(CPM_REG_SCCM1):
	case BCTS(CPM_REG_SCCM1) + 0x1:
		PARTIAL_READ(BCTS(CPM_REG_SCCM1), s->sccm1, addr, size, ret);
		break;
	case BCTS(CPM_REG_SCCS1):
		ret = s->sccs1;
		break;
		/* SCC2 */
	case BCTS(CPM_REG_GSMR_L2):
		ret = s->gsmr_l2;
		break;
	case BCTS(CPM_REG_GSMR_H2):
		ret = s->gsmr_h2;
		break;
	case BCTS(CPM_REG_PSMR2):
		ret = s->psmr2;
		break;
	case BCTS(CPM_REG_TODR2):
		ret = s->todr2;
		break;
	case BCTS(CPM_REG_DSR2):
		ret = s->dsr2;
		break;
	case BCTS(CPM_REG_SCCE2):
		ret = s->scce2;
		break;
	case BCTS(CPM_REG_SCCM2):
		ret = s->sccm2;
		break;
	case BCTS(CPM_REG_SCCS2):
		ret = s->sccs2;
		break;
#if 0
	/* SCC3 */
	case BCTS(CPM_REG_GSMR_L3):
		break;
	case BCTS(CPM_REG_GSMR_H3):
		break;
	case BCTS(CPM_REG_PSMR3):
		break;
	case BCTS(CPM_REG_TODR3):
		break;
	case BCTS(CPM_REG_DSR3):
		break;
	case BCTS(CPM_REG_SCCE3):
		break;
	case BCTS(CPM_REG_SCCM3):
		break;
	case BCTS(CPM_REG_SCCS3):
		break;
		/* SCC4 */	
	case BCTS(CPM_REG_GSMR_L4):
		break;
	case BCTS(CPM_REG_GSMR_H4):
		break;
	case BCTS(CPM_REG_PSMR4):
		break;
	case BCTS(CPM_REG_TODR4):
		break;
	case BCTS(CPM_REG_DSR4):
		break;
	case BCTS(CPM_REG_SCCE4):
		break;
	case BCTS(CPM_REG_SCCM4):
		break;
	case BCTS(CPM_REG_SCCS4):
		break;
#endif /* Not implemented yet */
		
	default : /* should never reach here */
		fprintf( stderr, "%s:%d] Unknown register read: %x\n",
					__func__, __LINE__, (int)addr);
	}

	DPRINTF("opaque=%p, addr=%x, size=%x ret=%x", opaque, (uint32_t)addr, size, ret);
	return ret;
}

/**
 *
 *
 *
 */
static void mpc8560_cpm_sccx_write(void* opaque, hwaddr addr, uint64_t value, unsigned size)
{
	CPMState* s = (CPMState*)opaque;
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;

	DPRINTF("opaque= %p, addr= %x, value= %lx size=%x", opaque, (uint32_t)addr, value, size);
	/* TODO : scale 'value' according to size?? */	
	switch( addr )
	{
		/* SCC1 */
		case BCTS(CPM_REG_GSMR_L1):	case BCTS(CPM_REG_GSMR_L1) + 0x1:
		case BCTS(CPM_REG_GSMR_L1) + 0x2: case BCTS(CPM_REG_GSMR_L1) + 0x3:
			PARTIAL_WRITE(BCTS(CPM_REG_GSMR_L1), s->gsmr_l1, addr, size, value);
			DPRINTF("gsmr_l1 -> %x", s->gsmr_l1);
			break;
		case BCTS(CPM_REG_GSMR_H1): case BCTS(CPM_REG_GSMR_H1) + 0x1:
		case BCTS(CPM_REG_GSMR_H1) + 0x2: case BCTS(CPM_REG_GSMR_H1) + 0x3:
			PARTIAL_WRITE(BCTS(CPM_REG_GSMR_H1), s->gsmr_h1, addr, size, value);
			DPRINTF("gsmr_h1 -> %x", s->gsmr_h1);
			break;
		case BCTS(CPM_REG_PSMR1):
		case BCTS(CPM_REG_PSMR1) + 0x1:
			PARTIAL_WRITE(BCTS(CPM_REG_PSMR1), s->psmr1, addr, size, value);
			DPRINTF("psmr1 -> %x", s->psmr1);
			break;
		case BCTS(CPM_REG_TODR1):
			PARTIAL_WRITE(BCTS(CPM_REG_TODR1), s->todr1, addr, size, value);
			DPRINTF("todr1 -> %x", s->todr1);
			break;
		case BCTS(CPM_REG_DSR1):
			value = value & 0xffff;
			s->dsr1 = value;
			DPRINTF("dsr1 -> %x", s->dsr1);
			break;
		case BCTS(CPM_REG_SCCE1):
		case BCTS(CPM_REG_SCCE1) + 0x1:
			{
				uint32_t old_scce = s->scce1;
				value = value & 0xffff;
				/* SCCE bits are cleared by writing ones; writing zeros has no effect */
				PARTIAL_WRITE_REVERSE(BCTS(CPM_REG_SCCE1), s->scce1, addr, size, value);
				//PARTIAL_WRITE(BCTS(CPM_REG_SCCE1), s->scce1, addr, size, value);
				/* When a pending interrupt is handled, the user clears the corresponding SIPNR bit.
				   If an event register exists, the unmasked event register bits should be cleared instead,
				   causing the SIPNR bit to be cleared.
				 */
				if( value & 0x0003 ) /* TX or RX */
				{
					s->sipnr_l = s->sipnr_l & ~(0x800000);

					if( old_scce )
					{
						DPRINTF("IRQ LOWER~~~~~~value (%d) scce(%d)", value, s->scce1);
						qemu_irq_lower(s->irq); // test
					}
				}
				DPRINTF("scce1 -> %x", s->scce1);
			}
			break;
		case BCTS(CPM_REG_RESV1): /* FIXME : behavior of reserved area is not defined */
		case (BCTS(CPM_REG_RESV1) + 1):
		//	s->scce1 = s->scce1 | value;
		//	s->sccm1 = s->sccm1 | value;
			PARTIAL_WRITE(BCTS(CPM_REG_RESV1), s->scc_resv1, addr, size, value);
			DPRINTF("scc_resv1 -> %x", s->scc_resv1);
			/* test start*/
			s->sccm1 = s->scc_resv1;
		//	if( s->sccm1 & SCCE_TX_MASK )
		//		mpc8560_cpm_set_irq(opaque);
			/* test end */
			break;
		case BCTS(CPM_REG_SCCM1):
		case BCTS(CPM_REG_SCCM1) + 0x1:
			value = value & 0xffff;
			PARTIAL_WRITE(BCTS(CPM_REG_SCCM1), s->sccm1, addr, size, value);
			DPRINTF("sccm1 -> %x", s->sccm1);
			break;
		case BCTS(CPM_REG_SCCS1):
			value = value & 0xff;
			s->sccs1 = value;
			DPRINTF("sccs1 -> %x", s->sccs1);
			break;

		/* SCC2 */
		case BCTS(CPM_REG_GSMR_L2):
			s->gsmr_l2 = value;
			DPRINTF("gsmr_l2 -> %x", s->gsmr_l2);
			break;
		case BCTS(CPM_REG_GSMR_H2):
			s->gsmr_h2 = value;
			DPRINTF("gsmr_h2 -> %x", s->gsmr_h2);
			break;
		case BCTS(CPM_REG_PSMR2):
			value = value & 0xffff;
			s->psmr2 = value;
			DPRINTF("psmr2 -> %x", s->psmr2);
			break;
		case BCTS(CPM_REG_TODR2):
			value = value & 0xffff;
			s->todr2 = value;
			DPRINTF("todr2 -> %x", s->todr2);
			break;
		case BCTS(CPM_REG_DSR2):
			value = value & 0xffff;
			s->dsr2 = value;
			DPRINTF("dsr2 -> %x", s->dsr2);
			break;
		case BCTS(CPM_REG_SCCE2):
			value = value & 0xffff;
			s->scce2 = value;
			DPRINTF("scce2 -> %x", s->scce2);
			break;
		case BCTS(CPM_REG_SCCM2):
			value = value & 0xffff;
			s->sccm2 = value;
			DPRINTF("sccm2 -> %x", s->sccm2);
			break;
		case BCTS(CPM_REG_SCCS2):
			value = value & 0xff;
			s->sccs2 = value;
			DPRINTF("sccs2 -> %x", s->sccs2);
			break;
#if 0
		/* SCC3 */
		case BCTS(CPM_REG_GSMR_L3):
			break;
		case BCTS(CPM_REG_GSMR_H3):
			break;
		case BCTS(CPM_REG_PSMR3):
			break;
		case BCTS(CPM_REG_TODR3):
			break;
		case BCTS(CPM_REG_DSR3):
			break;
		case BCTS(CPM_REG_SCCE3):
			break;
		case BCTS(CPM_REG_SCCM3):
			break;
		case BCTS(CPM_REG_SCCS3):
			break;

		/* SCC4 */	
		case BCTS(CPM_REG_GSMR_L4):
			break;
		case BCTS(CPM_REG_GSMR_H4):
			break;
		case BCTS(CPM_REG_PSMR4):
			break;
		case BCTS(CPM_REG_TODR4):
			break;
		case BCTS(CPM_REG_DSR4):
			break;
		case BCTS(CPM_REG_SCCE4):
			break;
		case BCTS(CPM_REG_SCCM4):
			break;
		case BCTS(CPM_REG_SCCS4):
			break;
#endif /* Not implemented yet */

		default :
			fprintf( stderr, "%s:%d] Unknown register write: %x = %x\n",
					__func__, __LINE__, (int)addr, (unsigned)value);
	}
	return ;
}

static uint64_t mpc8560_cpm_dpram_read(void* opaque, hwaddr addr, unsigned size)
{// FIXME : temporary
	uint32_t ret = 0;
	CPMState* s = (CPMState*)opaque;
	
	switch( addr )
	{
		case 0x0: 	/* 0xe1080080 -> bdstatus */
		case 0x1:
			PARTIAL_READ( 0x0, s->bd_status, addr, size, ret);
			break;
		case 0x2:	/* 0xe1080082 -> bdLength */
		case 0x3:
			PARTIAL_READ( 0x2, s->bd_dlen, addr, size, ret);
			break;
		default : /* should never reach here */
			fprintf( stderr, "%s:%d] Unknown register raed: %x\n",
					__func__, __LINE__, (int)addr);
	}


	DPRINTF("opaque=%p, addr=%x, size=%x ret=%x", opaque, (uint32_t)addr, size, ret);
	return ret;
}

static void mpc8560_cpm_dpram_write(void* opaque, hwaddr addr, uint64_t value, unsigned size)
{// FIXME : temporary
	CPMState* s = (CPMState*)opaque;
	//SysBusDevice* d = SYS_BUS_DEVICE(opaque);
	//addr &= MPC8560_CPM_MMIO_SIZE - 1;

	DPRINTF("opaque= %p, addr= %x, value= %lx size=%x", opaque, (uint32_t)addr, value, size);
		
	switch( addr )
	{
		case 0x0:	/* 0xe1080080 -> bdstatus */
		case 0x1:
			PARTIAL_WRITE( 0x0, s->bd_status, addr, size, value);
			DPRINTF("bd_status -> %x", s->bd_status );
			
			/* Detecting R bit */
			/* FIXME : Cannot actively response to a changes of TBPTR */
			if( s->bd_status & 0x8000 ) /* R bit set? */
				mpc8560_cpm_frame_tx_cb(opaque, PG_SCC_1);
			break;
		case 0x2:	/* 0xe1080082 -> bdlength */
		case 0x3:
			PARTIAL_WRITE( 0x2, s->bd_dlen, addr, size, value);
			DPRINTF("bd_dlen -> %x", s->bd_dlen );
			break;
		default :
			fprintf( stderr, "%s:%d] Unknown register write: %x = %x\n",
					__func__, __LINE__, (int)addr, (unsigned)value);
	}

	return ;
}

static uint64_t mpc8560_cpm_instram_debug_read(void* opaque, hwaddr addr, unsigned size)
{
	uint32_t ret = 0;
	DPRINTF("READ ACCESS INSTRUCTION RAM");
	return ret;
}

static void mpc8560_cpm_instram_debug_write(void* opaque, hwaddr addr, uint64_t value, unsigned size)
{
	DPRINTF("WRITE ACCESS INSTRUCTION RAM");
	return ;
}

static const MemoryRegionOps mpc8560_cpm_default_ops =
{
	.read		= mpc8560_cpm_default_read,
	.write		= mpc8560_cpm_default_write,
	.endianness	= DEVICE_BIG_ENDIAN,
//	.impl = {	.min_access_size = 4,
//				.max_access_size = 4,
//	},
};

static const MemoryRegionOps mpc8560_cpm_intctrl_ops = 
{
	.read		= mpc8560_cpm_intctrl_read,
	.write		= mpc8560_cpm_intctrl_write,
	.endianness = DEVICE_BIG_ENDIAN,
//	.impl = { 	.min_access_size = 4,
//				.max_access_size = 4,
//	},
};

static const MemoryRegionOps mpc8560_cpm_cp_ops =
{
	.read		= mpc8560_cpm_cp_read,
	.write		= mpc8560_cpm_cp_write,
	.endianness = DEVICE_BIG_ENDIAN,
//	.impl = {	.min_access_size = 4,
//				.max_access_size = 4,
//	},
};

static const MemoryRegionOps mpc8560_cpm_sccx_ops =
{
	.read		= mpc8560_cpm_sccx_read,
	.write		= mpc8560_cpm_sccx_write,
	.endianness	= DEVICE_BIG_ENDIAN,
//	.impl = {	.min_access_size = 4,
//				.max_access_size = 4,
//	},
};

static const MemoryRegionOps mpc8560_cpm_dpram_ops =
{// FIXME : for temporary use until cpm enabled, forcing dpram to mmio
	.read		= mpc8560_cpm_dpram_read,
	.write		= mpc8560_cpm_dpram_write,
	.endianness	= DEVICE_BIG_ENDIAN,
	.impl = {	.min_access_size = 1,
				.max_access_size = 2,
	},
};

static const MemoryRegionOps mpc8560_cpm_instram_debug_ops =
{
	.read		= mpc8560_cpm_instram_debug_read,
	.write		= mpc8560_cpm_instram_debug_write,
	.endianness = DEVICE_BIG_ENDIAN,
//	.impl = {	.min_access_size = 2,
//				.max_access_size = 2,
//	},
};


static void mpc8560_cpm_initfn(Object* obj)
{
//	SysBusDevice* d = SYS_BUS_DEVICE(obj);
	CPMState* s = MPC8560_CPM(obj);

	DPRINTF("HIT");	

	/* Creating a CPM memory address space and will be attached as a subregion of ccsr_space */
	memory_region_init(&(s->cpm_space), obj, "mpc8560-cpm", 0x40000);


//	memory_region_init_io(&s->cpm_space, OBJECT(s), &mpc8560_cpm_ops, s,
//							"mpc8560-cpm", MPC8560_CPM_MMIO_SIZE);
//	sysbus_init_mmio(d, &(s->iomem));
	
	return ;
}

void mpc8560_cpm_init_serial(CPMState* s, CharDriverState* chr)
{
	QEMUSerialSetParams ssp;
	
	if( NULL == chr )
		hw_error("%s:%d] Char device not initialized.\n", __func__, __LINE__);
	
	/* assign serial0 -> SCC1 */
	if( !strncmp( chr->label, "serial0", strlen("serial0")) )
	{
		s->serial_scc1 = chr;
		qemu_chr_add_handlers( s->serial_scc1, scc_can_receive, scc_receive, scc_event, s );
		fifo8_create( &(s->recv_fifo_scc1), SCC_TXRX_FIFO_LEN );
		s->scc_status_poll = qemu_new_timer_ns( vm_clock, (QEMUTimerCB*)scc_update_timer_cb, s );
		s->fifo_timeout_timer = qemu_new_timer_ns( vm_clock, (QEMUTimerCB*)scc_fifo_timeout_int, s );

		ssp.speed = 114583; /* baud rate 115200 actual freq 114583 */
		ssp.parity = 'N';
		ssp.data_bits = 8;
		ssp.stop_bits = 1;
		qemu_chr_fe_ioctl( s->serial_scc1, CHR_IOCTL_SERIAL_SET_PARAMS, &ssp );
	}
	else
		fprintf( stderr, "%s:%d] Initializing Char device (%s) ignored.\n", __func__, __LINE__, chr->label );
	
	return ;
}

static void mpc8560_cpm_realize(DeviceState* dev, Error** errp)
{
//	SysBusDevice* d = SYS_BUS_DEVICE(dev);
	CPMState* s = MPC8560_CPM(dev);

	DPRINTF("HIT");
	
	/* ** Initializing subregion ** */
	memory_region_init_ram( &(s->dpram1), NULL, "cpm.dpram1", CPM_SIZE_DPRAM );
	memory_region_add_subregion( &(s->cpm_space), BCTC(CPM_BASE_DPRAM1), &(s->dpram1) );

	memory_region_init_ram( &(s->dpram2), NULL, "cpm.dpram2", CPM_SIZE_DPRAM );
	memory_region_add_subregion( &(s->cpm_space), BCTC(CPM_BASE_DPRAM2), &(s->dpram2) );
	
	/* default mmio area(it cover the whole cpm registers area 0x90000 - 0x91fff) */
	memory_region_init_io( &(s->whole_mmio), OBJECT(s), &mpc8560_cpm_default_ops, s, "cpm.mmio", 0x2000 ); 
	memory_region_add_subregion( &(s->cpm_space), BCTC(CPM_REG_CEAR), &(s->whole_mmio) );

	/* interrupt controller */
	memory_region_init_io( &(s->intctrl), OBJECT(s), &mpc8560_cpm_intctrl_ops, s, "cpm.intctrl", 0x80 );
	memory_region_add_subregion_overlap( &(s->cpm_space), BCTC(CPM_REG_SICR), &(s->intctrl), 1 );
	
	/* Communications Processor */
	memory_region_init_io( &(s->cp_mmio), OBJECT(s), &mpc8560_cpm_cp_ops, s, "cpm.cp", 0x20 );
	memory_region_add_subregion_overlap( &(s->cpm_space), BCTC(CPM_REG_CPCR), &(s->cp_mmio), 1 );

	/* SCCx */
	memory_region_init_io( &(s->sccx), OBJECT(s), &mpc8560_cpm_sccx_ops, s, "cpm.sccx", 0x80 );
	memory_region_add_subregion_overlap( &(s->cpm_space), BCTC(CPM_REG_GSMR_L1), &(s->sccx), 1 );


	/* Initializing overlap memory space */
	// TODO : connect scattered address by using alias 
	// temporary use for detecting 1st TxBDs R bit because CPM clock source not enabled yet 
	memory_region_init_io( &(s->cpm_dpram_temp), OBJECT(s), &mpc8560_cpm_dpram_ops, s, "cpm.dpram.mmio.temp", 0x4 );
	memory_region_add_subregion_overlap( &(s->cpm_space), 0x80, &(s->cpm_dpram_temp), 1 );
	
	/* Instruction RAM */
	memory_region_init_ram( &(s->dpinstram), NULL, "cpm.instram", CPM_SIZE_DPINSTRAM );
	memory_region_add_subregion( &(s->cpm_space), BCTC(CPM_BASE_DPINSTRAM), &(s->dpinstram) );

	/* FIXME: for debug */
	memory_region_init_io( &(s->cpinstemp), OBJECT(s), &mpc8560_cpm_instram_debug_ops, s, "cpm.inst.dbg", CPM_SIZE_DPINSTRAM );
	memory_region_add_subregion_overlap( &(s->cpm_space), BCTC(CPM_BASE_DPINSTRAM), &(s->cpinstemp), 1 );
	
	return ;
}

static void mpc8560_cpm_class_init(ObjectClass* oc, void* data)
{
	DeviceClass* dc = DEVICE_CLASS(oc);

	DPRINTF("HIT");	
	dc->realize = mpc8560_cpm_realize;
	//dc->props = ;
	dc->reset = mpc8560_cpm_reset;

	return ;
}

static const TypeInfo mpc8560_cpm_info = 
{
	.name			= TYPE_MPC8560_CPM,
	.parent			= TYPE_SYS_BUS_DEVICE,
	.instance_size	= sizeof(CPMState),
	.instance_init	= mpc8560_cpm_initfn,
	.class_init 	= mpc8560_cpm_class_init,
};

static void mpc8560_cpm_register_types(void)
{
	type_register_static(&mpc8560_cpm_info);
	DPRINTF("HIT");	
	return ;
}

type_init(mpc8560_cpm_register_types)

/* ***************************** END ******************************* */
