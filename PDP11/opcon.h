/* opcon.h: Interface to a real operator console

   Copyright (c) 2006-2017, Edward Groenenberg & Henk Gooijen

   Permission is hereby granted, free of charge, to any person obtaining a
   copy of this software and associated documentation files (the "Software"),
   to deal in the Software without restriction, including without limitation
   the rights to use, copy, modify, merge, publish, distribute, sublicense,
   and/or sell copies of the Software, and to permit persons to whom the
   Software is furnished to do so, subject to the following conditions:

   The above copyright notice and this permission notice shall be included in
   all copies or substantial portions of the Software.

   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
   THE AUTHOR(S) BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
   IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
   CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

   Except as contained in this notice, the names of the author(s) shall not
   be used in advertising or otherwise to promote the sale, use or other
   dealings in this Software without prior written authorization from the
   author(s).

   21-sep-18    EG	Remove 11/45 code, cleanup
   28-feb-17    EG      Rewrote part of smh version
   27-apr-16    EG      Rewrote, consoletask is now a separate process
   20-mar-14    EG      New oc_svc, oc_get_rotary, minor changes
   08-feb-14    EG      Rewrite of original realcons.c & adapted for simh 4.0 
*/

#ifndef OC_DEFS
#define OC_DEFS 1
#include "sim_tmxr.h"

/*
 * Implementation notes are found in doc/opcon_doc.txt
*/

//#define DEBUG_OC 1 				/* enable/disable debug */

/*
 * Model to use, to be defined from compile command.
 *   Define OPCON_SHM for shared memory model,
 *   Define OPCON_THR for multi thread model
 *   Define OPCON_SER for direct serial model.
 */
//#define OPCON_SHM				/* shm model */
//#define OPCON_THR				/* pthread model */
//#define OPCON_SER				/* direct serial model */

#ifndef MOD_1170
#define MOD_1170	12
#endif

#define INP1			0
#define INP2			1
#define INP3			2
#define INP4			3
#define INP5			4
#define SWR_00_07_PORT	     INP1	/* SWITCH REGISTER 7-0 */
#define SWR_08_15_PORT	     INP2	/* SWITCH REGISTER 15-8 */
#define SWR_16_22_PORT	     INP3	/* SWITCH REGISTER 16-22 */

/* 11/70 switches / ports, etc. */
#define SW_PL_1170	     0x80	/* key switch bitfield */
#define SW_HE_1170	     0x40	/* HALT bitfield */
#define SW_SY_1170           0x20       /* SBUSSYC bitfield */

/* DISPLAY ADDRESS rotary switch for 11/70 */
#define DSPA_PROGPHY	     0x00	/* PROG PHY */
#define DSPA_KERNEL_D	     0x01	/* KERNEL D */
#define DSPA_KERNEL_I	     0x02	/* KERNEL I */
#define DSPA_CONSPHY	     0x03	/* CONS PHY */
#define DSPA_SUPER_D	     0x04	/* SUPER D */
#define DSPA_SUPER_I	     0x05	/* SUPER I */
#define DSPA_USER_D	     0x06	/* USER D */
#define DSPA_USER_I	     0x07	/* USER I */
#define DSPA_MASK	     0x07	/* mask for DSPA range */

/* DISPLAY DATA rotary switch for 11/70 */
#define DSPD_BUS_REG	     0x00	/* BUS REG */
#define DSPD_DATA_PATHS	     0x01	/* DATA PATHS */
#define DSPD_DISP_REG	     0x02	/* DISPLAY REGISTER */
#define DSPD_MU_ADRS	     0x03	/* uADRS FPP/CPU */
#define DSPD_MASK	     0x03	/* mask for DSPA range */

/* Ack_toggle flag definitions */
#define ACK_DEPO	     0x40
#define ACK_CONT	     0x08
#define ACK_LOAD	     0x04
#define ACK_START	     0x02
#define ACK_EXAM	     0x01
#define ACK_MASK	     0x4F

/* Definitions copied from pdp11_defs.h, including it directly causes errors. */
#define MMR0_MME	  0000001	/* 18 bit MMU enabled */
#define MMR3_M22E	      020	/* 22 bit MMU enabled */
#define MD_KER			0	/* protection mode - KERNEL */
#define MD_SUP			1	/* protection mode - SUPERVISOR */
#define MD_UND			2	/* protection mode - UNDEFINED */
#define MD_USR			3	/* protection mode - USER */

/* Shared definitions for 11/70 */
#define FSTS_RUN	0x80		/* common for both */
#define FSTS_MASTER	0x40
#define FSTS_INDDATA	0x04
#define FSTS_USER	0x03
#define FSTS_SUPER	0x01		/*  value 0x02 is all 3 OFF */
#define FSTS_KERNEL	0x00

/* STAT_1_OUTPORT 11/70 */
/* out3  [2] |  RUN  | MASTER| PAUSE |ADRSERR| PARERR|INDDATA|MMR0[1]|MMR0[0]*/
#define FSTS_PAUSE	0x20
#define FSTS_ADRSERR	0x10
#define FSTS_PARERR	0x08

/* STAT_2_OUTPORT 11/70 */
/* out2  [1] |       |       |       | PARHI | PARLO | 22BIT | 18BIT | 16BIT */
#define FSTS_PARHI	0x10
#define FSTS_PARLO	0x08
#define FSTS_22BIT	0x04
#define FSTS_18BIT	0x02
#define FSTS_16BIT	0x01

/* index values for data array */
#define DISP_SHFR	0	/* data paths (shiftr); normal setting  */
#define DISP_BR		1	/* read/write data                      */
#define DISP_FPP	2	/* uAdrs/FPP                            */
#define DISP_DR		3	/* Display Register                     */
#define DISP_BDV	4	/* non-standard BDV                     */

/* index values for address array */
#define ADDR_KERNI	0	/* Kernel I */
#define ADDR_KERND	1	/* Kernel D */
#define ADDR_SUPRI	2	/* Super  I */
#define ADDR_SUPRD	3	/* Super  D */
#define ADDR_ILLI	4	/* Only in 11/74 */
#define ADDR_ILLD	5	/* Only in 11/74 */
#define ADDR_USERI	6	/* User   I */
#define ADDR_USERD	7	/* User   D */
#define ADDR_PRGPA	8	/* Prog PA  */
#define ADDR_CONPA	9	/* Cons PA  */

/* OC controlblock */
struct OC_ST {
  t_bool sir;			/* copy of sim_is_running value */
  t_bool first_exam;		/* flag: first EXAM action */
  t_bool first_dep;		/* flag: first DEP action */
  t_bool ind_addr;		/* flag: indirect address */
  t_bool inv_addr;		/* flag: invalid address (out of range )*/
  uint32 act_addr;		/* used address for EXAM/DEP */
  uint16 act_data;		/* used data for EXAM/DEP */
  uint8  HALT;			/* HALT switch modes */
  uint8  PORT1;			/* status register 1 */
  uint8  PORT2;			/* status register 2 */
  uint32 A[10];			/* Address Mux array */
  uint16 D[5];			/* Data Mux array */
  uint8  S[5];			/* switches and toggles retrieved state */
#ifdef OPCON_SHM
  uint8  ACK;			/* single ack toggle data */
  uint8  to_cp;			/* cmd from SIMH */
  uint8  fm_cp;			/* cmd to   SIMH */
  int32  MMR0;			/* MMU register 0 */
  int32  MMR3;			/* MMU register 3 */
  uint8  line[32];		/* serial line to use */
  uint8	 cpu_model;		/* cpu model */
#endif
#ifdef OPCON_THR
  pthread_t t_thr;		/* thread */
  int       t_end;		/* thread argument */
#endif
  };
typedef struct OC_ST OC_ST;

extern OC_ST *ocp;   

#ifdef OPCON_SHM
#define OC_MMR0 ocp->MMR0 = MMR0
#define OC_MMR3 ocp->MMR3 = MMR3
#define oc_toggle_clear() oc_send_CMD('C', 0)
#else 
#define OC_MMR0
#define OC_MMR3
#define OC_INTERVAL 1000        /* in micro sec */                            
#define OC_MINVAL 4000          /* in micro sec */                             
# ifdef OPCON_THR
  int pthread_create(pthread_t *a, CONST pthread_attr_t *b, void *(*c)(void*), void *d);
  int pthread_join(pthread_t a, void **d);
# endif
#endif

struct SERPORT {                                                                
  int port;                                                                    
  };                                                                      
typedef struct SERPORT SERPORT;

extern uint32 cpu_model;                                                  
extern t_bool oc_active;                                                                                                                                                              
/* function prototypes simh integration */

t_stat oc_attach (UNIT *uptr, CONST char *cptr);
t_stat oc_detach (UNIT *uptr);
CONST char *oc_description (DEVICE *dptr);
t_stat oc_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, CONST char *cptr);
t_stat oc_help_attach (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, CONST char *cptr);
t_stat oc_reset (DEVICE *dptr);
t_stat oc_show (FILE *st, UNIT *uptr, int32 flag, CONST void *desc);
t_stat oc_svc (UNIT *uptr);

/* function prototypes OC */

t_bool  oc_check_halt (void);
void    oc_clear_halt (void);

#ifdef OPCON_SHM
void    oc_ack_ALL(int a, OC_ST *b);
void    oc_get_SWR (void);
void    oc_send_A (int a, OC_ST *b);
void    oc_send_AD (int a, OC_ST *b);
void    oc_send_ADS (int a, OC_ST *b);
void    oc_send_S (int a, OC_ST *b);
void    oc_send_CMD (uint8 cmd, uint8 mask);
#endif

#if defined(OPCON_THR) || defined(OPCON_SER)
void    oc_console (void);
int     oc_get_SWR (void);
void    oc_send_A (uint32 A);
void    oc_send_AD (uint32 A, uint16 D);
void    oc_send_ADS (void);
void    oc_send_S (void);
void    oc_toggle_ack (uint8 mask);
void    oc_toggle_clear (void);
#endif

#ifdef OPCON_THR
void   *oc_thread(void *end_thr);
#endif

uint32  oc_get_ADR (void);
t_bool  oc_get_CON (char *cptr);
uint16  oc_get_DTA (void);
t_bool  oc_get_HLT (void);
int     oc_get_RTR (void);
t_bool  oc_poll (SERHANDLE channel, int amount);
int     oc_read (SERHANDLE channel, char *b, int c, int d);
char   *oc_read_line_p (char *prompt, char *cptr, int32 size, FILE *stream, int32 do_echo);
void    oc_set_master (t_bool flag);
void    oc_set_mmu (void);
void    oc_set_port1 (uint8 flag, t_bool action);
void    oc_set_port2 (uint8 flag, t_bool action);
void    oc_set_ringprot (int value);
void    oc_set_wait (t_bool flag);

#endif /* OC_DEFS */

