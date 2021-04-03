/* pdp11_dx.c : Interface to DX-11 display panel
 
   Copyright (c) 2021 Edward Groenenberg

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
   THE AUTHOR or AUTHORS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
   WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
   OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
   SOFTWARE.

   Except as contained in this notice, the name of the Author or Authors `
   shall not be used in advertising or otherwise to promote the sale, use
   or other dealings in this Software without prior written
   authorization from the Author or Authors.

   14-dec-20    EG	Initial build
*/

/* SIMH integration
 * dx_attach()			: attach device (initialize link)
 * dx_detach()			: detach device (close link)
 * dx_help()			: Generic help
 * dx_help_attach()		: Help for attach command
 * dx_reset()			: device reset
 * dx_show()			: show status of the device (link)
 * dx_svc()			: service routine
 *
 */

#ifdef VM_PDP11
#include <stdio.h>
#include <unistd.h>
#include <sys/time.h>
#include <sys/types.h>
#include "sim_defs.h"
#include "scp.h"
#include "pdp11_defs.h"
#include <ctype.h>
#include <termio.h>
#include <sys/ioctl.h>

/* Declarations & references for required external routines & variables. */
extern SERHANDLE sim_open_serial (char *name, TMLN *lp, t_stat *status);
extern SERHANDLE sim_close_serial (SERHANDLE port);
extern int32 R[];			/* Current registers R0 - R7 */
extern int32 PSW;			/* PSW register */

/* Defines */
#define msleep(n) usleep(n * 1000);	/* Millisecond sleep */

/* Debug levels for the DX device */
#define DXDEB_SVC	      001	/* service calls  */
#define DXDEB_TRC	      002	/* trace calls */

/* Debug flags & keywords for the DX device */
DEBTAB oc_debug[] = {
    { "SVC", DXDEB_SVC },		/* used in dx_svc */
    { "TRC", DXDEB_TRC },		/* used in all major entry points */
    { 0 }
    };

/* UNIT definition */
UNIT dx_unit = { UDATA (&dx_svc, UNIT_ATTABLE + UNIT_DISABLE, 0) };

/* Modifiers definitions */
MTAB dx_mod[] = { 
    { MTAB_XTD|MTAB_VDV|MTAB_NMO, 0, "STATUS", NULL,
        NULL, &dx_show, NULL, "Display console link status" },
    { 0 } 
    };	

/* DEVICE definition */
DEVICE dx_dev = {
    "DX",					/* device code */
    (UNIT *)&dx_unit,				/* UNIT structure */
    NULL,					/* register (N/U) */
    (MTAB *)dx_mod,				/* modifier options */
    1,						/* # of units (N/U) */
    0,						/* address radix (N/U) */
    0,						/* address width (N/U) */
    0,						/* address increment (N/U) */
    0,						/* data radix (N/U) */
    0,						/* data width (N/U) */
    NULL,					/* examine function (N/U) */
    NULL,					/* deposit function (N/U) */
    &dx_reset,					/* reset function */
    NULL,					/* boot function (N/U) */
    &dx_attach,					/* attach function */
    &dx_detach,					/* detach function */
    NULL,					/* context (N/U) */
    DEV_UBUS | DEV_DIS | DEV_DISABLE | DEV_DEBUG,	/* device flags */
    0,						/* debug control (N/U) */
    dx_debug,					/* debug options */
    NULL,					/* memory size function (N/U) */
    NULL,					/* logical name (N/U) */
    &dx_help,					/* help function */
    &dx_help_attach,				/* attach help function */
    NULL,					/* help context (N/U) */
    &dx_description,				/* description function */
    NULL					/* break type table */
    };

uint8 dx_data[34];
SERHANDLE dx_serhandle;
struct termios dx_tty;

/*   ***   ***   ***   ***   SIMH device integration   ***   ***   ***   ***   */

/*
 * Function : dx_attach()
 * Note	    : Attach the serial line for the DX11 microcontroller.
 *	      2 bytes per action, 1st byte is byte index number,
 *            2nd byte is the data.
 * Returns  : SCPE_OK or -1
 */
t_stat dx_attach (UNIT *uptr, CONST char *cptr)
{
char *cmdp, *tptr;
t_stat r;

sim_debug (DXDEB_TRC, &dx_dev, "dx_attach : called\n");

if (cptr == NULL)
    return SCPE_ARG;

if ((tptr = strchr (cptr, '=')) == NULL)
    return SCPE_ARG;
*tptr++;						/* skip '=' */

if ((dx_serhandle = sim_open_serial (tptr, NULL, &r)) != ((SERHANDLE)(void *)-1)) { /* port usable? */
    if (r != SCPE_OK) {
        printf ("DX    : serial link open error (%d).\n", errno);
        return -1;
        }
    }

				/* Configure port reading	      */
if (tcgetattr(dx_serhandle->port, &dx_tty)) {
    printf("DX : failed to get line attributes (%d)\n", errno);
    sim_close_serial (dx_serhandle);
    return -1;
    }
fcntl(dx_serhandle->port, F_SETFL);
cfmakeraw(&dx_tty);
dx_tty.c_cc[VMIN] = 0;
dx_tty.c_cc[VTIME] = 0;			/* no timeout */
if (tcsetattr(dx_serhandle->port, TCSANOW, &dx_tty)) {
    printf("DX : failed to set attributes for raw mode\n");
    sim_close_serial (dx_serhandle);
    return -1;
    }

/* 
 * Initialize the data array with positioning data
 * row 1 : left 2 bytes, right 2 bytes
 */
dx_data[0]  =  0;	/* row 1, lamps no.  1 -  8 */
dx_data[2]  =  1;	/* row 1, lamps no.  9 - 16 */
dx_data[4]  =  2;	/* row 1, lamps no. 21 - 28 */
dx_data[6]  =  3;	/* row 1. lamps no. 29 - 36 */
dx_data[8]  =  4;	/* row 2, lamps no.  1 -  8 */
dx_data[10] =  5;	/* row 2, lamps no.  9 - 16 */
dx_data[12] =  6;	/* row 2, lamps no. 21 - 28 */
dx_data[14] =  7;	/* row 2, lamps no. 29 - 36 */
dx_data[16] =  8;	/* row 3, lamps no.  1 -  8 */
dx_data[18] =  9;	/* row 3, lamps no.  9 - 16 */
dx_data[20] = 10;	/* row 3, lamps no. 21 - 28 */
dx_data[22] = 11;	/* row 3, lamps no. 29 - 36 */
dx_data[24] = 12;	/* row 4, lamps no.  1 -  8 */
dx_data[26] = 13;	/* row 4, lamps no.  9 - 16 */
dx_data[28] = 14;	/* row 4, lamps no. 21 - 28 */
dx_data[30] = 15;	/* row 4, lamps no. 29 - 36 */
dx_data[32] = 16;	/* row 1, lamps 17 - 20 & row 2 lamps 17 - 20 */
dx_data[34] = 17;	/* row 3, lamps 17 - 20 & row 4 lamps 17 - 20 */

uptr->flags = uptr->flags | UNIT_ATT;
return SCPE_OK;
}

/*
 * Function : dx_detach()
 * Note	    : Deactivate & detach the console processor link
 * Returns  : SCPE_OK or -1
 */
t_stat dx_detach (UNIT *uptr)
{
if (!(uptr->flags & UNIT_ATT)) retrun SCPE_OK; 

sim_cancel(&dx_unit);
sim_close_serial (dx_serhandle);

if (uptr == NULL)
    return SCPE_IERR;
if (!(uptr->flags & UNIT_ATTABLE))			/* attachable? */
    return SCPE_NOATT;
if (!(uptr->flags & UNIT_ATT)) {			/* not attached? */
    return SCPE_OK;					/* allow detach */
    }
uptr->flags = uptr->flags & ~(UNIT_ATT | 0);

return 0;
}

/*
 * Function : dx_reset()
 * Note	    : Reset the device and queue the service routine.
 *            Called on start of simulator, at 'reset <dev>', 'reset all'
 *            and 'boot <dev>'.
 * Returns  : SCPE_OK
 */
t_stat dx_reset (DEVICE *dptr)
{
sim_debug (DXDEB_TRC, &dx_dev, "dx_reset : called\n");
sim_activate_after (&dx_unit, DX_INTERVAL);     /* queue service routine */
return SCPE_OK;
}

/*
 * Function : dx_svc()
 * Note	    : This is the service routine.
 */

t_stat dx_svc (UNIT *uptr)
{
uint32 ms_cur, r;
uint16 I;
static uint32 ms_old;

sim_debug (DXDEB_TRC, &dx_dev, "dx_svc : called\n");
ms_cur = sim_os_msec();
sim_debug (DXDEB_SVC, &dx_dev, "dx_svc : delta = %d\n", ms_cur - ms_old);

if ((ms_cur - ms_old) < DX_MINVAL) {         /* sufficient time passed by? */
    sim_activate_after (uptr, DX_INTERVAL);       /* reschedule */
    return 0;
    }
ms_old = ms_cur;					/* set as new marker */

dx_data[1]  = (uint8)(((uint16)R[0] >> 8) & 0xFF);
dx_data[3]  = (uint8)((uint16)R[0] & 0xFF);

dx_data[5]  = (uint8)(((uint16)R[1] >> 8) & 0xFF);
dx_data[7]  = (uint8)((uint16)R[1] & 0xFF);

dx_data[9]  = (uint8)(((uint16)R[2] >> 8) & 0xFF);
dx_data[11] = (uint8)((uint16)R[2] & 0xFF);

dx_data[13] = (uint8)(((uint16)R[3] >> 8) & 0xFF);
dx_data[15] = (uint8)((uint16)R[3] & 0xFF);

dx_data[17] = (uint8)(((uint16)R[4] >> 8) & 0xFF);
dx_data[19] = (uint8)((uint16)R[4] & 0xFF);

dx_data[21] = (uint8)(((uint16)R[5] >> 8) & 0xFF);
dx_data[23] = (uint8)((uint16)R[5] & 0xFF);

dx_data[25] = (uint8)(((uint16)R[6] >> 8) & 0xFF);
dx_data[27] = (uint8)((uint16)R[6] & 0xFF);

dx_data[29] = (uint8)(((uint16)R[7] >> 8) & 0xFF);
dx_data[31] = (uint8)((uint16)R[7] & 0xFF);

dx_data[33] = (uint8)(((uint16)PSW  >> 8) & 0xFF);
dx_data[35] = (uint8)((uint16)PSW  & 0xFF);

if ((r = write (oc_serhandle->port, dx_data, sizeof(dx_data))) != (sizeof(dx_data))
  printf("OC    : Error sending data (e=%d, wr=%d)\n", errno, r);

sim_activate_after (uptr, DX_INTERVAL);         /* reschedule */

return SCPE_OK;
}

/*
 * Function : dx_show()
 * Note	    : Show the status of the link
 * Returns  : SCPE_OK
 */
t_stat dx_show (FILE *st, UNIT *uptr, int32 val, CONST void *desc)
{
if (uptr->flags & UNIT_ATT) 
    fputs ("active\n", st);
else
    fputs ("not active\n", st);
return SCPE_OK;
}

/*
 * Function : dx_help()
 * Note     : Help about opcon
 *            Processes 'help dx' (not 'help set dx')
 * Returns  : Nothing
 */
t_stat dx_help (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, CONST char *cptr)
{
CONST char *const text =
" Display panel DX11 controller\n"
"\n"
" The DX11 is a pseudo driver and is an interface to a real DX11 display panel.\n"
" The panel can display all kinds of data the coder wants. Here it display's the\n"
" Contents of R0 - R7 + PSW\n"
;

fprintf (st, "%s", text);
fprint_set_help (st, dptr);
fprint_show_help (st, dptr);
dx_help_attach (st, dptr, uptr, flag, cptr);
return SCPE_OK;
}

/*
 * Function : dx_help_attach()
 * Note     : Help about opcon attach
 *            Processes 'help dx' (not 'help set dx')
 * Returns  : Nothing
 */
t_stat dx_help_attach (FILE *st, DEVICE *dptr, UNIT *uptr, int32 flag, CONST char *cptr)
{
CONST char *const text =
" DX device ATTACH help."
"\n"
" The DX driver uses a serial port to send commands and data to the apnel uC."
"\n"
" The ATTACH command specifies which serial port to be used.\n"
" A serial port may be specified as an operating system specific device name\n"
" or using simh generic serial name. SIMH generic names are of the form\n"
" serN, where N is from 0 thru one less than the maximum number of serial\n"
" ports on the local system. The mapping of simh generic port names to OS \n"
" specific names can be displayed using the following command:\n"
"\n"
"   sim> SHOW SERIAL\n"
"   Serial devices:\n"
"    ser0   /dev/ttyS0\n"
"    ser1   /dev/ttyS1\n"
"\n"
"   sim> ATTACH DX connect=ser0\n"
"\n"
" or equivalently:\n"
"\n"
"   sim> ATTACH DX connect=/dev/ttyS1\n"
"\n"
" The connection configured for the DX device are unconfigured by:\n"
"\n"
"   sim> DETACH DX\n"
"\n"
" This will close the communication subsystem.\n"
"\n"
;

fprintf (st, "%s", text);
return SCPE_OK;
}

/*
 * Function : dx_description()
 * Note     : Single line description
 * Returns  : Pointer to the text
 */
CONST char *dx_description (DEVICE *dptr)
{
return "DX11 : Interface to DX-11 display panel";
}

#endif
/* EOF */
