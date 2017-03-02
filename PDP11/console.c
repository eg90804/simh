/*
** console sub processor program.
**
** Attach to the shared memory segment and start reading register data and
** writing switch and knob settings.
** The console processor board (CPB) is doing the reading and writing to
** the actual physical console.
*/

#include <stdio.h>
#include <sys/types.h>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <time.h>
#include <signal.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <sim_defs.h>
#include <opcon.h>

#define msleep(a) usleep((a * 1000))

int end_prog = 0;
void sighan() { end_prog = 1; }
struct termios tty;

//#define DEBUG 1
#ifdef DEBUG
int debug = 0;
#endif

/*
** Cleanup and exit
*/
void oc_exit(int oc_fd, OC_ST *ocp, char *txt)
{
  printf("%s\n", txt);
  if (ocp) shmdt((char *)ocp);
  close(oc_fd);
  exit(1);
}

/*
** Read data from the CPB.
** Mode = 0 -> read a number of bytes, must succeed
** Mode = 1 -> read one byte, may succeed
*/
int oc_read(int fd, char *p, int c, int m)
{
  int x;
  extern int errno;
  extern struct termios tty;

  if (m == 0) {
    tty.c_cc[VMIN] = c;		/* Must read 'c' chars	*/
    tcsetattr(fd, TCSANOW, &tty);
    }
    
  if ((x = read(fd, p, c)) != c && errno != EAGAIN && errno != EWOULDBLOCK)
    x = 0;
    
  tty.c_cc[VMIN] = 0;			/* reset to previous state	*/
  tcsetattr(fd, TCSANOW, &tty);

  return(x);
}

/*
** Send Address, Data and Port info to console processor.
*/
void oc_send_ADS(int oc_fd, OC_ST *ocp)
{
  uint8 mask = 0, c[8];
  uint32 A;
  uint16 D;

  if (ocp->cpu_model == MOD_1145) {
    switch ((ocp->S[INP3] >> 4) & DSPA_MASK) {
	case DSPA_PROGPHY : A = ocp->A[ADDR_PRGPA] & 0x3FFFF;break;
	case DSPA_CONSPHY : A = ocp->A[ADDR_CONPA] & 0x3FFFF;break;
	case DSPA_KERNEL_D: A = ocp->A[ADDR_KERND] & 0xFFFF; break;
	case DSPA_KERNEL_I: A = ocp->A[ADDR_KERNI] & 0xFFFF; break;
	case DSPA_SUPER_D : A = ocp->A[ADDR_SUPRD] & 0xFFFF; break;
	case DSPA_SUPER_I : A = ocp->A[ADDR_SUPRI] & 0xFFFF; break;
	case DSPA_USER_D  : A = ocp->A[ADDR_USERD] & 0xFFFF; break;
	case DSPA_USER_I  : A = ocp->A[ADDR_USERI] & 0xFFFF; break;
	}
    switch ((ocp->S[INP3] >> 2) & DSPD_MASK) {
	case DSPD_DATA_PATHS : D = ocp->D[DISP_SHFR]; break;
	case DSPD_BUS_REG    : D = ocp->D[DISP_BR];   break;
	case DSPD_MU_ADRS    : D = ocp->D[DISP_FPP];  break;
	case DSPD_DISP_REG   : D = ocp->D[DISP_DR];   break;
	}
    }
  else {
    switch (ocp->S[INP5] & DSPA_MASK) {
	case DSPA_PROGPHY : A = ocp->A[ADDR_PRGPA] & 0x3FFFFF; break;
	case DSPA_CONSPHY : A = ocp->A[ADDR_CONPA] & 0x3FFFFF; break;
	case DSPA_KERNEL_D: A = ocp->A[ADDR_KERND] & 0xFFFF;   break;
	case DSPA_KERNEL_I: A = ocp->A[ADDR_KERNI] & 0xFFFF;   break;
	case DSPA_SUPER_D : A = ocp->A[ADDR_SUPRD] & 0xFFFF;   break;
	case DSPA_SUPER_I : A = ocp->A[ADDR_SUPRI] & 0xFFFF;   break;
	case DSPA_USER_D  : A = ocp->A[ADDR_USERD] & 0xFFFF;   break;
	case DSPA_USER_I  : A = ocp->A[ADDR_USERI] & 0xFFFF;   break;
	}
    switch ((ocp->S[INP5] >> 3) & DSPD_MASK) {
	case DSPD_DATA_PATHS : D = ocp->D[DISP_SHFR]; break;
	case DSPD_BUS_REG    : D = ocp->D[DISP_BR];   break;
	case DSPD_MU_ADRS    : D = ocp->D[DISP_FPP];  break;
	case DSPD_DISP_REG   : D = ocp->D[DISP_DR];   break;
	}
    }

  if (ocp->MMR0 & MMR0_MME) {
    mask = 0x03;
    if (ocp->MMR3 & MMR3_M22E)
      mask = 0x3F;
    }

  c[0] = 'U';
  c[1] = (uint8)((A >> 16) & mask); 
  c[2] = (uint8)((A >>  8) & 0xFF);
  c[3] = (uint8) (A & 0xFF);
  c[4] = (uint8)((D >> 8) & 0xFF);
  c[5] = (uint8) (D & 0xFF);
  c[6] = ocp->PORT1;
  c[7] = ocp->PORT2;
  if (write(oc_fd, c, 8) != 8)
    oc_exit(oc_fd, ocp, "Exit : oc_send_ADS");
}

/*
** Send single Address & Data to CPB.
*/
void oc_send_AD(int oc_fd, OC_ST *ocp)
{
  uint8 mask = 0, c[6];

  if (ocp->MMR0 & MMR0_MME) {
    mask = 0x03;
    if (ocp->MMR3 & MMR3_M22E)
      mask = 0x3F;
    }

  c[0] = 'B';
  c[1] = (uint8)((ocp->act_addr >> 16) & mask) ;
  c[2] = (uint8)((ocp->act_addr >> 8) & 0xFF) ;
  c[3] = (uint8) (ocp->act_addr & 0xFF);
  c[4] = (uint8)((ocp->act_data >> 8) & 0xFF) ;
  c[5] = (uint8) (ocp->act_data & 0xFF);
  if (write(oc_fd, c, 6) != 6)
    oc_exit(oc_fd, ocp, "Exit : oc_send_AD");
}

/*
** Send single Address to CPB.
*/
void oc_send_A(int oc_fd, OC_ST *ocp)
{
  uint8 mask = 0, c[4];

  if (ocp->MMR0 & MMR0_MME) {
    mask = 0x03;
    if (ocp->MMR3 & MMR3_M22E)
      mask = 0x3F;
    }

  c[0] = 'A';
  c[1] = (uint8)((ocp->act_addr >> 16) & mask) ;
  c[2] = (uint8)((ocp->act_addr >>  8) & 0xFF) ;
  c[3] = (uint8) (ocp->act_addr & 0xFF);
  if (write(oc_fd, c, 4) != 4)
    oc_exit(oc_fd, ocp, "Exit : oc_send_A");
}

/*
** Send status info to CPB.
*/
void oc_send_S(int oc_fd, OC_ST *ocp)
{
  uint8 c[4];

  c[0] = 'F';
  c[1] = ocp->PORT1;
  c[2] = ocp->PORT2;
  if (write(oc_fd, c, 3) != 3)
    oc_exit(oc_fd, ocp, "Exit : oc_send_S");
}

/*
** Check if the HLT switch is flipped.
*/
void oc_read_HLT(int oc_fd, OC_ST *ocp)
{
  uint8 c;
  
  if (oc_read(oc_fd, &c, 1, 1) == 1) {		/* look for halt      */
#ifdef DEBUG
    if (debug) printf("A : got byte : %c\r", c);
#endif
    if (c == 'H') 					/* 'HALT' is set?     */
      ocp->HALT = 2;				/* Yes, set to mode 2 */
    else 
      if (c == 'E') { 				/* 'ENABLE' is set?   */
        ocp->HALT = 1;				/* Yes, set to mode 1 */
        oc_ack_ALL(oc_fd, ocp);		/* Acknowledge all toggles    */
        }
      else
	if (strchr ("cdlsx", c) != NULL)		/* Stray toggle?      */
          oc_ack_ALL(oc_fd, ocp);			/* Just ack it        */
    }
}

/*
** Request rotary knobs settings from CPB.
*/
void oc_read_RTR(int oc_fd, OC_ST *ocp)
{
  uint8 c = 'R';

  if (write(oc_fd, &c, 1) != 1)
    oc_exit(oc_fd, ocp, "Exit : oc_read_RTR - 1");
  if (oc_read(oc_fd, &c, 1, 0) != 1)
    oc_exit(oc_fd, ocp, "Exit : oc_read_RTR - 2");

  if (ocp->cpu_model == MOD_1145)
    ocp->S[INP3] = c;
  else
    ocp->S[INP5] = c;
}

/*
** Request setting of the switches.
*/
void oc_read_SWR(int oc_fd, OC_ST *ocp)
{
  uint8 c = 'Q';

  if (write(oc_fd, &c, 1) != 1 || oc_read(oc_fd, ocp->S, 5, 0) != 5)
    oc_exit(oc_fd, ocp, "Exit : oc_read_SWR");
    
#ifdef DEBUG
  if (debug) printf("oc_read_SWR : 0x%02X, 0x%02X, 0x%02X, 0x%02X, 0x%02X\n",
  	ocp->S[0], ocp->S[1], ocp->S[2], ocp->S[3], ocp->S[4]);
#endif
}

/*
** Acknowledge all toggle commands.
*/
void oc_ack_ALL(int oc_fd, OC_ST *ocp)
{
  uint8 c = 'i';	/* clear all */

  if (write(oc_fd, &c, 1) != 1)
    oc_exit(oc_fd, ocp, "Exit : oc_ack_ALL");
}

/*
** Acknowledge one toggle command using the mask.
*/
void oc_ack_ONE(int oc_fd, OC_ST *ocp)
{
  uint8 c[4];
  
  c[0] = 'c';
  c[1] = 0x30 + INP3;
  c[2] = ocp->ACK;
  if (write(oc_fd, c, 3) != 3)
    oc_exit(oc_fd, ocp, "Exit : oc_ack_ONE");
  ocp->ACK = 0;
}

/*
** Main routine
*/
int main(int ac, char **av)
{
  int x, oc_fd, oc_shmid, c_cnt = 0;
  key_t	oc_key = 201702;
  char c, cmd_buf[2];
  OC_ST *ocp;
  struct termios savetty;
  extern struct termios tty;
  extern int errno, end_prog;
  extern void sighan();

  end_prog = 0;

  signal(SIGHUP, sighan);

#ifdef DEBUG
  if ( ac > 1 && av[1][0] == '-' && av[1][1] == 'd')
    debug = 1;
#endif 
		/* attach to shm exchange area */

  if ((oc_shmid = shmget(oc_key, sizeof(OC_ST), 0)) == -1 ||
      (ocp = (OC_ST *)shmat(oc_shmid, NULL, 0)) == (OC_ST *)-1) {
    printf("OCC : shmget/shmat error (errno = %d).\n", errno);
    exit(1);
    }

#ifdef DEBUG
  if (debug) {
    printf("Dump of ocp :\n");
    printf("  First exam address : 0x%08o\n", ocp->first_exam);
    printf("  First dep address  : 0x%08o\n", ocp->first_dep);
    printf("  Invalid address    : 0x%08o\n", ocp->inv_addr);
    printf("  Serial line        : %s\n", ocp->line);
    printf("  CPU model          : %d\n", ocp->cpu_model);
    printf("  Active address     : 0x%08o\n", ocp->act_addr);
    printf("  Active data        : 0x%06o\n", ocp->act_data);
    printf("  MMR0               : %d\n", ocp->MMR0);
    printf("  MMR3               : %d\n", ocp->MMR3);
    printf("  HALT value         : %d\n", ocp->HALT);
    printf("  Port 1 data        : 0x%02X\n", ocp->PORT1);
    printf("  Port 2 data        : 0x%02X\n", ocp->PORT2);
    printf("  CMD from SIMH      : %c\n", ocp->to_cp);
    printf("  CMD to SIMH        : %c\n", ocp->fm_cp);
    printf("  Acknowledge mask   : %d\n", ocp->ACK);
    printf("  Address array      : 0x%08o,0x%08o,0x%08o,0x%08o\n",
    		ocp->A[0], ocp->A[1], ocp->A[2], ocp->A[3]);
    printf("                       0x%08o,0x%08o,0x%08o,0x%08o\n",
    		ocp->A[4], ocp->A[5], ocp->A[6], ocp->A[7]);
    printf("  Data array         : 0x%06o,0x%06o,0x%06o,0x%06o\n",
    		ocp->D[0], ocp->D[1], ocp->D[2], ocp->D[3]);
    printf("  Switch data array: 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",
    	ocp->S[0], ocp->S[1], ocp->S[2], ocp->S[3], ocp->S[4]);
    }
    sleep(1);    
#endif

	/* open the serial line as passed in the control block */
  if ((oc_fd = open(ocp->line, O_RDWR|O_NOCTTY|O_NONBLOCK, 0666)) < 0) {
    printf("OCC : open error (%d on %s).\n", errno, ocp->line);
    shmdt((char *)ocp);			/* detach shared mem	*/
    exit(1);
    }

#ifdef DEBUG
  if (debug) printf("Line '%s' (%d) is open\n", ocp->line, oc_fd);
#endif
  
	/* set line dicipline (9600-8n1, raw) */

  if ((x = tcgetattr(oc_fd, &tty)) < 0) {
    printf("OCC : failed to get line attributes: %d, %s", x, strerror(errno));
    shmdt((char *)ocp);			/* detach shared mem	*/
    exit (1);
    }
  savetty = tty;		/* preserve original settings for restoration */
  fcntl(oc_fd, F_SETFL);	/* Configure port reading		      */
  cfsetispeed(&tty, B9600);	/* Set the baud rate to 9600		      */
  cfsetospeed(&tty, B9600);

  cfmakeraw(&tty);  
//  tty.c_cflag &= ~CSTOPB; 				/* 1 stop bit	      */
  tty.c_cflag |= CSTOPB;  /* 2 stop bits as current FW does require it */
  tty.c_cflag &= ~CRTSCTS;		/* Disable hardware flow control      */
  tty.c_cflag |= (CLOCAL | CREAD);	/* Enable the receiver, local mode    */
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;						/* no timeout */
  
  // Set the new attributes
  if (tcsetattr(oc_fd, TCSANOW, &tty) != 0) {
    printf("OCC : failed to set attributes for raw mode\n");
    shmdt((char *)ocp);
    exit(1);
    }

/* init the console processor board */
#ifdef DEBUG
 if(!debug) {
#endif

  cmd_buf[0] = 'p';
  cmd_buf[1] = '5';
  if (ocp->cpu_model == MOD_1145)
     cmd_buf[1] = '4';

  if (write(oc_fd, cmd_buf, 2) != 2) {
    printf("OCC : failed to send cpu type (errno = %d)\n", errno);
    shmdt((char *)ocp);					/* detach shared mem  */
    exit (1);
    }
#ifdef DEBUG
  if (debug) printf("CPU type is written, errno = %d\n", errno);
  }
#endif

/*
**  We are in business, let the other side know we are ready.
**  The 'to_cp' field was set by SIMH for observation
*/
  ocp->A[ADDR_PRGPA] = 0x002017;
  ocp->D[DISP_SHFR]  = 0x0ED0;
  oc_send_ADS(oc_fd, ocp);
  msleep(10);
  ocp->to_cp = 0;
  ocp->fm_cp = 0;
  
/*
** Loop until a signal is received.
** There are 2 modes, non interactive and interactive.
** In the first mode, the following steps are executed :
**   - check for a SWR get request (only during pre-boot of simulated cpu).
**   - send current A, D & P to the CPB
**   - check if HALT switch is used
**     set halt mode, read SWR
**     stray toggles are cleared.
**     drop to interactive mode.
**   - every 5th iteration, the rotary switch is read.
**
** In the 2nd mode, the following steps are executed :
**   - check if previous cmd was processed, loop until it is
**   - wait for an input command
**   - perform action based on input command.
**
*/
  while(end_prog == 0) {
    if (!ocp->HALT) {			/* if 0, we are not interactive       */
      c = 0;
      if (ocp->to_cp == 'Q') {				/* SWR requested?     */
#ifdef DEBUG
	if (debug) printf("A : Q -> call oc_read_SWR()\n");
#endif
        oc_read_SWR(oc_fd, ocp);			/* Get switch data    */
        ocp->to_cp = 0;						/* Clear flag */
        continue;
        }
      else					/* Unknow cmd, just clear it  */
        ocp->to_cp = 0;
        
      oc_send_ADS(oc_fd, ocp);				/* Update all leds    */
      switch(c_cnt++) {
        default : break;
        case  3 : oc_read_RTR(oc_fd, ocp);	/* Get rotary switch data     */
        	  break;
        case  7 : oc_read_HLT(oc_fd, ocp);
        	  break;
        case 10 : oc_read_SWR(oc_fd, ocp);
        	  break;
        case 11 : c_cnt = 0;
                  break;
        }
      }
    else { 					/* In 'interactive' mode      */
      oc_send_S(oc_fd, ocp);			/* Update the status leds     */
//      oc_read_SWR(oc_fd, ocp);
      msleep(10);
#ifdef DEBUG
      if (debug) printf("B : fm_cp = %c, to_cp = %c\n", ocp->fm_cp, ocp->to_cp);
#endif
      if (!ocp->fm_cp) { 			/* No pending cmd to SIMH?    */
        c = 0;
        if (oc_read(oc_fd, &c, 1, 1) == 1) {	/* Check for toggle switch    */
#ifdef DEBUG
          if (debug && c != 0) printf("B : got cmd %c (%d)\n", c, c);
#endif
          switch (c) {
            case 'H' : ocp->HALT = 2;
        	       break;
            case 'E' : ocp->HALT = 1;
		       oc_ack_ALL(oc_fd, ocp);
		       msleep(5);
		       break;
            case 'c' :
            case 'd' :
            case 'l' :
            case 's' :
            case 'x' : ocp->fm_cp = c;
          	       break;
            default  : ocp->fm_cp = 0;
          	       break;
            }
          }
	}
      
      if (ocp->to_cp != 0) {			/* Pending cmd from SIMH?     */
        switch (ocp->to_cp) {			/* Yes, let's see what it is  */
          case 'A' : oc_send_A(oc_fd, ocp);			break;
          case 'B' : oc_send_AD(oc_fd, ocp);			break;
	  case 'C' : oc_ack_ALL(oc_fd, ocp);   			break;
          case 'F' : oc_send_S(oc_fd, ocp);			break;
	  case 'O' : oc_ack_ONE(oc_fd, ocp);			break;
          case 'Q' : oc_read_SWR(oc_fd, ocp);			break;
	  case 'R' : oc_ack_ALL(oc_fd, ocp); ocp->HALT = 0;	break;
          default  :						break;
	  }
        ocp->to_cp = 0;					/* clear command      */
        }
      }
    } /* end loop */

  tcsetattr(oc_fd, TCSANOW, &savetty);				/* reset line */
  close(oc_fd);							/* close line */
  shmdt((char *)ocp);					/* detach shared mem  */
  exit(0);
}

