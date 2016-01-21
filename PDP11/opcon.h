/* opcon.h: Interface to a real operator console

   Copyright (c) 2006-2015, Edward Groenenberg & Henk Gooijen

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

   20-mar-14    EG      new oc_svc, oc_get_rotary, minor changes
   08-Feb-14    EG      Rewrite of original realcons.c & adapted for simh 4.0 
*/

#ifndef OC_DEFS
#define OC_DEFS 1
/*
 * Implementation notes are found in doc/opcon_doc.txt
*/

//#define DEBUG_OC 1 				/* enable/disable debug */

#ifndef MOD_1105                
#define MOD_1105	 2
#define MOD_1120	 3
#define MOD_1140	 8
#define MOD_1145	10
#define MOD_1170	12
#endif

/* Shared function/status port LEDs definitions */
#define FSTS_RUN                0x80

/* STAT_1_OUTPORT 11/70 */
/* out3  [2] |  RUN  | MASTER| PAUSE |ADRSERR| PARERR|INDDATA|MMR0[1]|MMR0[0]*/
#define FSTS_1170_RUN           0x80
#define FSTS_1170_MASTER	0x40
#define FSTS_1170_PAUSE	        0x20
#define FSTS_1170_ADRSERR	0x10
#define FSTS_1170_PARERR	0x08
#define FSTS_1170_INDDATA	0x04
#define FSTS_1170_USER	        0x03
#define FSTS_1170_SUPER	        0x01		/*  value 0x02 is all 3 OFF */
#define FSTS_1170_KERNEL	0x00

/* STAT_2_OUTPORT 11/70 */
/* out2  [1] |       |       |       | PARHI | PARLO | 22BIT | 18BIT | 16BIT */
#define FSTS_1170_PARHI	        0x10
#define FSTS_1170_PARLO	        0x08
#define FSTS_1170_22BIT	        0x04
#define FSTS_1170_18BIT	        0x02
#define FSTS_1170_16BIT	        0x01

/* STAT_1_OUTPORT 11/45 (11/50 & 11/55)	*/
/* out6  [5] |  RUN  | MASTER|ADRSERR| PAUSE |       |INDATA |MMR0[1]|MMR0[0] */
#define FSTS_1145_RUN           0x80
#define FSTS_1145_MASTER        0x40
#define FSTS_1145_ADRSERR       0x20
#define FSTS_1145_PAUSE	        0x10
#define FSTS_1145_INDDATA       0x04
#define FSTS_1145_USER	        0x03
#define FSTS_1145_SUPER	        0x01		/*  value 0x02 is all 3 OFF */
#define FSTS_1145_KERNEL        0x00

/* STAT_2_OUTPORT 11/45, 11/50 & 11/55  --> not used */


/* STAT_1_OUTPORT 11/40 ( & 11/35) */
/* out3  [2] |  RUN  |  PROC |  BUS  |       |       | USER  |CONSOLE|VIRTUAL*/
#define FSTS_1140_RUN           0x80
#define FSTS_1140_PROC	        0x40
#define FSTS_1140_BUS	        0x20
#define FSTS_1140_USER	        0x04
#define FSTS_1140_CONSOLE	0x02
#define FSTS_1140_VIRTUAL	0x01
#define FSTS_1140_18BIT	        0x01

/* STAT_2_OUTPORT 11/40  --> not used */

/* STAT_1_OUTPORT 11/20 TBD! */
/* out3  [2] |  RUN  | FETCH |  BUS  | EXEC  | SOURCE| DEST  | ADDR1 | ADDR2 |*/
#define FSTS_1120_RUN           0x80
#define FSTS_1120_PROC	        0x40
#define FSTS_1120_BUS	        0x20
#define FSTS_1120_EXEC          0x10
#define FSTS_1120_SOURCE        0x08
#define FSTS_1120_DEST   	0x04
#define FSTS_1120_ADDR1  	0x02
#define FSTS_1120_ADDR2	        0x01

/* STAT_2_OUTPORT 11/20  --> not used */

/* STAT_1_OUTPORT 11/05 ( & 11/10) TBD! */
/* out3  [2] |  RUN  |       |       |       |       |       |       |       |*/
#define FSTS_1105_RUN           0x80

/* STAT_2_OUTPORT 11/05  --> not used */


/* index values for oc_ctl.D data array */
#define DISP_SHFR	0	/* data paths (shiftr); normal setting  */
#define DISP_BR		1	/* read/write data                      */
#define DISP_FPP	2	/* uAdrs/FPP                            */
#define DISP_DR		3	/* Display Register                     */
#define DISP_BDV	4	/* non-standard BDV                     */

/* index values for oc_ctl.A address array */
#define ADDR_KERNI	0	/* Kernel I */
#define ADDR_KERND	1	/* Kernel D */
#define ADDR_SUPRI	2	/* Super  I */
#define ADDR_SUPRD	3	/* Super  D */
#define ADDR_ILLI	4	/* Not used */
#define ADDR_ILLD	5	/* Not used */
#define ADDR_USERI	6	/* User   I */
#define ADDR_USERD	7	/* User   D */
#define ADDR_PRGPA	8	/* Prog PA  */
#define ADDR_CONPA	9	/* Cons PA  */

/* OC controlblock */
struct oc_st {
  t_bool key;			/* flag: panel key in POWER or LOCK pos. */
  t_bool first_exam;		/* flag: first EXAM action */
  t_bool first_dep;		/* flag: first DEP action */
  t_bool ind_addr;		/* flag: indirect data access */
  t_bool inv_addr;		/* flag: invalid address (out of config range */
  uint8  halt;			/* HALT switch modes */
  uint8  c_upd;			/* update counter */
  uint8  c_rot;			/* rotation switch update counter */
  uint8  port1;			/* status LED register - port 1 */
  uint8  port2;			/* status LED register - port 2 */
  uint32 act_addr;		/* used address for EXAM/DEP */
  uint32 resched;		/* oc_svc timing */
  uint32 A[10];			/* Address Mux led array */
  uint16 D[5];			/* Data Mux Led array */
  uint8  S[5];			/* switches and toggles retrieved state */
  };
typedef struct oc_st oc_st;

/* function prototypes simh integration */

extern uint32 cpu_model;

t_stat oc_attach (UNIT *uptr, char *cptr);
t_stat oc_detach (UNIT *uptr);
char  *oc_description (DEVICE *dptr);
t_stat oc_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat oc_reset (DEVICE *dptr);
t_stat oc_show (FILE *st, UNIT *uptr, int32 flag, void *desc);
t_stat oc_svc (UNIT *uptr);

/* function prototypes OC */

void   oc_clear_halt (void);
uint16 oc_extract_data (void);
uint32 oc_extract_address (void);
t_bool oc_get_console (char *cptr);
t_bool oc_get_halt (void);
int    oc_get_rotary (void);
int    oc_get_swr (void);
int    oc_halt_status (void);
void   oc_mmu (void);
void   oc_port1 (uint8 flag, t_bool action);
void   oc_port2 (uint8 flag, t_bool action);
char  *oc_read_line_p (char *prompt, char *cptr, int32 size, FILE *stream);
void   oc_ringprot (int value);
void   oc_master (t_bool flag);
t_bool oc_poll (int channel, int amount);
void   oc_send_address (uint32 A);
void   oc_send_address_data (uint32 A, uint16 D);
void   oc_send_all (uint32 A, uint16 D);
void   oc_send_data (uint16 D);
void   oc_send_status ();
void   oc_toggle_ack (uint8 mask);
void   oc_toggle_clear (void);
void   oc_wait (t_bool flag);
t_stat oc_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);
t_stat oc_help_attach (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, char *cptr);

#endif /* OC_DEFS */
