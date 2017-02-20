/*
* console sub processor program.
*
* Attach to the shared memory segment and start reading register data and
* writing switch and knob settings.
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

/* Defines */
#define INP1                    0
#define INP2                    1
#define INP3                    2
#define INP4                    3
#define INP5                    4
#define SWR_00_07_PORT       INP1       /* SWITCH REGISTER 7-0 */
#define SWR_08_15_PORT       INP2       /* SWITCH REGISTER 15-8 */
#define SWR_16_22_PORT       INP3       /* SWITCH REGISTER 16-22 */

/* DISPLAY ADDRESS rotary switch for 11/45 & 11/70 */
#define DSPA_PROGPHY         0x00       /* PROG PHY */
#define DSPA_KERNEL_D        0x01       /* KERNEL D */
#define DSPA_KERNEL_I        0x02       /* KERNEL I */
#define DSPA_CONSPHY         0x03       /* CONS PHY */
#define DSPA_SUPER_D         0x04       /* SUPER D */
#define DSPA_SUPER_I         0x05       /* SUPER I */
#define DSPA_USER_D          0x06       /* USER D */
#define DSPA_USER_I          0x07       /* USER I */
#define DSPA_MASK            0x07       /* mask for DSPA range */

/* DISPLAY DATA rotary switch for 11/45 & 11/70 */
#define DSPD_BUS_REG         0x00       /* BUS REG */
#define DSPD_DATA_PATHS      0x01       /* DATA PATHS */
#define DSPD_DISP_REG        0x02       /* DISPLAY REGISTER */
#define DSPD_MU_ADRS         0x03       /* uADRS FPP/CPU */
#define DSPD_MASK            0x03       /* mask for DSPA range */

/* Definitions copied from pdp11_defs.h, including it directly causes errors. */
#define MMR0_MME          0000001       /* 18 bit MMU enabled */
#define MMR3_M22E             020       /* 22 bit MMU enabled */
#define MD_KER                  0       /* protection mode - KERNEL */
#define MD_SUP                  1       /* protection mode - SUPERVISOR */
#define MD_UND                  2       /* protection mode - UNDEFINED */
#define MD_USR                  3       /* protection mode - USER */

int end_prog = 0;
int sighan() { end_prog = 1; }

/*
** Read data from the console processor.
*/
int oc_read (int oc_fd, struct termios *tty, char *p, int c, int m)
{
  extern int errno;
  int x;
  fd_set s;
  struct timeval t;

  if (m == 0) {
    tty->c_cc[VMIN] = 0;		/* if no char, return		*/
    tcsetattr (oc_fd, TCSANOW, tty);
    if ((x = read (oc_fd, p, c)) != c && errno != EAGAIN && errno != EWOULDBLOCK)
      x = 0;
    }
  else {
    tty->c_cc[VMIN] = c;		/* at least 'c' chars		*/
    tcsetattr (oc_fd, TCSANOW, tty);
    t.tv_sec = 0;
    t.tv_usec = 100;
    FD_ZERO (&s);
    FD_SET (oc_fd, &s);
    select (FD_SETSIZE, &s, NULL, NULL, &t);
    if (FD_ISSET (oc_fd, &s)) {
      if ((x = read (oc_fd, p, c)) != c && errno != EAGAIN && errno != EWOULDBLOCK)
        x = 0;
      }
    else 
      x = 0;
    }

  /* reset to previous state */
  tty->c_cc[VMIN] = 1;
  tcsetattr (oc_fd, TCSANOW, tty);

  return (x);
}

/*
** Send single Address display to processor.
*/
void oc_send_A (int oc_fd, OC_ST *ocp)
{
  uint8 mask = 0, cmd[4];

if (ocp->MMR0 & MMR0_MME) {
    mask = 0x03;
    if (ocp->MMR3 & MMR3_M22E)
	mask = 0x3F;
    }

  cmd[0] = 'A';
  cmd[1] = (uint8)((ocp->act_addr >> 16) & mask) ;
  cmd[2] = (uint8)((ocp->act_addr >>  8) & 0xFF) ;
  cmd[3] = (uint8) (ocp->act_addr & 0xFF);
  write (oc_fd, cmd, 4);
}

/*
** Send single Addres & Data to processor.
*/
void oc_send_AD (int oc_fd, OC_ST *ocp)
{
  uint8 mask = 0, cmd[6];

if (ocp->MMR0 & MMR0_MME) {
    mask = 0x03;
    if (ocp->MMR3 & MMR3_M22E)
	mask = 0x3F;
    }

  cmd[0] = 'B';
  cmd[1] = (uint8)((ocp->act_addr >> 16) & mask) ;
  cmd[2] = (uint8)((ocp->act_addr >> 8) & 0xFF) ;
  cmd[3] = (uint8) (ocp->act_addr & 0xFF);
  cmd[4] = (uint8)((ocp->D[0] >> 8) & 0xFF) ;
  cmd[5] = (uint8) (ocp->D[0] & 0xFF);
  write (oc_fd, cmd, 6);
}

/*
** Send Address, Data and Port info to console processor.
*/
void oc_send_ADS (int oc_fd, OC_ST *ocp)
{
  uint8 mask = 0, cmd[8];
  uint32 A;
  uint16 D;

  if (ocp->cpu_model == MOD_1145) {
    switch ((ocp->S[INP3] >> 4) & DSPA_MASK) {
      case DSPA_PROGPHY : A = ocp->A[ADDR_PRGPA]&0x3FFFF;break;
      case DSPA_CONSPHY : A = ocp->A[ADDR_CONPA]&0x3FFFF;break;
      case DSPA_KERNEL_D: A = ocp->A[ADDR_KERND]&0xFFFF; break;
      case DSPA_KERNEL_I: A = ocp->A[ADDR_KERNI]&0xFFFF; break;
      case DSPA_SUPER_D : A = ocp->A[ADDR_SUPRD]&0xFFFF; break;
      case DSPA_SUPER_I : A = ocp->A[ADDR_SUPRI]&0xFFFF; break;
      case DSPA_USER_D  : A = ocp->A[ADDR_USERD]&0xFFFF; break;
      case DSPA_USER_I  : A = ocp->A[ADDR_USERI]&0xFFFF; break;
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
      case DSPA_PROGPHY : A = ocp->A[ADDR_PRGPA]&0x3FFFFF;break;
      case DSPA_CONSPHY : A = ocp->A[ADDR_CONPA]&0x3FFFFF;break;
      case DSPA_KERNEL_D: A = ocp->A[ADDR_KERND]&0xFFFF; break;
      case DSPA_KERNEL_I: A = ocp->A[ADDR_KERNI]&0xFFFF; break;
      case DSPA_SUPER_D : A = ocp->A[ADDR_SUPRD]&0xFFFF; break;
      case DSPA_SUPER_I : A = ocp->A[ADDR_SUPRI]&0xFFFF; break;
      case DSPA_USER_D  : A = ocp->A[ADDR_USERD]&0xFFFF; break;
      case DSPA_USER_I  : A = ocp->A[ADDR_USERI]&0xFFFF; break;
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

  cmd[0] = 'U';
  cmd[1] = (uint8)((A >> 16) & mask); /* & mask */
  cmd[2] = (uint8)((A >>  8) & 0xFF);
  cmd[3] = (uint8) (A & 0xFF);
  cmd[4] = (uint8)((D >> 8) & 0xFF);
  cmd[5] = (uint8) (D & 0xFF);
  cmd[6] = ocp->PORT1;
  cmd[7] = ocp->PORT2;
  write (oc_fd, cmd, 8);
}

/*
** Send status info to console processor.
*/
void oc_send_S (int oc_fd, OC_ST *ocp)
{
  uint8 cmd[4];

  cmd[0] = 'F';
  cmd[1] = ocp->PORT1;
  cmd[2] = ocp->PORT2;
  write(oc_fd, cmd, 3);
}

/*
** Check for HALT switch change.
*/
void oc_read_HLT (int oc_fd, OC_ST *ocp, termios *tty)
{
  if (oc_read (oc_fd, tty, &c, 1, 0) == 1) {/* look for halt 	*/
    if (c == 'H') {			/* HALT set?		*/
      ocp->HALT = 2;			/* Yes, set tmo mode 2	*/
      oc_read_SWR (oc_fd, ocp);		/* Get switch data	*/
      oc_ack_all (oc_fd);		/* Just ack it		*/
      }
    else {
      if (strchr ("cdlsx", c) != NULL)	/* Stray toggle?	*/
        oc_ack_all (oc_fd);		/* Just ack it		*/
    }
  }
}


/*
** Request setting of the switches.
*/
void oc_read_RTR (int oc_fd, OC_ST *ocp)
{
  uint8 c = 'R';

  write (oc_fd, &c, 1);
  read (oc_fd, &c, 1);

  if (ocp->cpu_model == MOD_1145)
    ocp->S[INP3] = c;
  else
    ocp->S[INP5] = c;
}

/*
** Request setting of the switches.
*/
void oc_read_SWR (int oc_fd, OC_ST *ocp)
{
  uint8 c = 'Q';

  write (oc_fd, &c, 1);
  read (oc_fd, ocp->S, 5);
}

/*
** Acknowledge all toggle commands.
*/
void oc_ack_all (int oc_fd)
{
  uint8 c = 'i';	/* clear all */

  write (oc_fd, &c, 1);
}

/*
** Acknowledge on etoggle command using the mask.
*/
void oc_ack_one (int oc_fd, OC_ST *ocp)
{
  write (oc_fd, ocp->ACK, 3);
}


int main (int ac, char **av)
{
  int x, oc_fd, oc_shmid, mask = 0, c_cnt = 0;
  uint32 *shm_addr;
  key_t	oc_key;
  char c, cmd_buf[4], *cmdp;
  OC_ST *ocp;
  struct termios tty, savetty;
  struct timespec ns;
  extern int errno, end_prog;

  struct shmid_ds shminfo;

  end_prog = 0;
  if (ac < 2) {
    printf ("Usage: %s <shm_address>\n", av[0]);
    exit (1);
    }

  shm_addr = atoi (av[1]);
  signal (SIGHUP, sighan);

		/* attach to shm exchange area */

  oc_key = 201604;
  if ((oc_shmid = shmget (oc_key, sizeof(OC_ST), 0)) == -1 ||
      (ocp = (OC_ST *)shmat (oc_shmid, NULL, 0)) == (OC_ST *)-1) {
    printf ("OCC : shmget/shmat error (errno = %d).\n", errno);
    exit (1);
    }

	/* open the serial line as passed in the control block */

  if ((oc_fd = open (ocp->line, O_RDWR|O_NOCTTY|O_NONBLOCK, 0666)) < 0) {
    printf ("OCC : open error (%d on %s).\n", errno, ocp->line);
    shmdt ((char *)ocp);			/* detach shared mem	*/
    exit (1);
    }

	/* set line dicipline (9600-8n1, raw) */

  if ((x = tcgetattr (oc_fd, &tty)) < 0) {
    printf ("failed to get attr: %d, %s", x, strerror (errno));
    shmdt ((char *)ocp);			/* detach shared mem	*/
    exit (1);
    }
  savetty = tty;    /* preserve original settings for restoration */
  fcntl (oc_fd, F_SETFL);	// Configure port reading
  tcgetattr (oc_fd, &tty);	// Get the current options for the port
  cfsetispeed (&tty, B9600);	// Set the baud rate to 9600
  cfsetospeed (&tty, B9600);

  tty.c_cflag |= (CLOCAL | CREAD); // Enable the receiver and set local mode
  tty.c_cflag &= ~PARENB;        // No parity bit
/*  tty.c_cflag &= ~CSTOPB; */       // 1 stop bit
  tty.c_cflag &= CSTOPB;	/* 2 stop bits */
  tty.c_cflag &= ~CSIZE;         // Mask data size
  tty.c_cflag |=  CS8;           // Select 8 data bits
  tty.c_cflag &= ~CRTSCTS;       // Disable hardware flow control  

  // Enable data to be processed as raw input
  tty.c_lflag &= ~(ICANON | ECHO | ISIG);

  tty.c_cc[VMIN] = 1;
  tty.c_cc[VTIME] = 0; /* no timeout */

  // Set the new attributes
  tcsetattr (oc_fd, TCSANOW, &tty);

/* init the console processor board */
  cmdp = "p5";
  if (ocp->cpu_model == MOD_1145)
    cmdp = "p4";
  write (oc_fd, cmdp, 2);

/* we are in business, let the other side know we are ready */
/* The A[0] field is set by SIMH for observation */
  ocp->A[0] = 0;

/*
 * Loop until a signal is received.
 * There are 2 modes, non interactive and interactive.
 * In the first mode, the following steps are executed :
 *   - check for a SWR get request (only during pre-boot of simulated cpu).
 *   - send current A, D & P to the CPB
 *   - check if HALT switch is used
 *     set halt mode, read SWR
 *     stray toggles are cleared.
 *     drop to interactive mode.
 *   - every 5th iteration, all switch settings are read.
 *
 * In the 2nd mode, the following steps are executed :
 *   - check if previous cmd was processed, loop until it is
 *   - wait for an input command
 *   - perform action based on input command.
 *
*/
  while (end_prog == 0) {
    if (ocp->HALT == 0) {		/* if 0, we are not interactive	*/
      if (ocp->to_cp == 'Q') {			/* SWR requested?	*/
        oc_read_SWR (oc_fd, ocp);		/* Get switch data	*/
        ocp->to_cp = 0;
        continue;
        }
      oc_send_ADS (oc_fd, ocp);
      switch (c_cnt++) {
        default : break;
        case  3 : oc_read_RTR (oc_fd, ocp);	break;
        case  7 : oc_read_HLT (oc_fd, ocp, &tty); break;	/* look for halt 	*/
        case 10 : oc_read_SWR (oc_fd, ocp);		/* get switch values	*/
		  mask = 0x40;
		  if (ocp->cpu_model == MOD_1145)
		    mask = 0x01;
		  if (ocp->S[1] & mask)			/* halt switch used?	*/
		    ocp->HALT = 2;			/* Yes, set it		*/
                  break;
        case 11 : c_cnt = 0;     break;
        }
      }
    else { /* in 'interactive' mode */
      while (ocp->fm_cp != 0) 		/* previous cmd not processed yet	*/
        usleep (1000);
      while ((c = oc_read (oc_fd, &tty, &ocp->fm_cp, 1, 1)) == (char)0 ||
	     ocp->to_cp == 0)
        ;

      switch (ocp->to_cp) {
         case 'A' : oc_send_A (oc_fd, ocp);	break;
         case 'B' : oc_send_AD (oc_fd, ocp);	break;
         case 'F' : oc_send_S (oc_fd, ocp);	break;
         case 'Q' : oc_read_SWR (oc_fd, ocp);	break;
	 case 'a' : oc_ack_all (oc_fd);       	break;
	 case 'o' : oc_ack_one (oc_fd, ocp);	break;
         default  :				break;
	 }
      ocp->to_cp = 0;				/* clear command	*/
      }
    }						/* end for loop		*/

  tcsetattr (oc_fd, TCSANOW, &savetty);		/* reset line		*/
  close (oc_fd);				/* close line		*/
  shmdt ((char *)ocp);				/* detach shared mem	*/
  exit (0);
}

