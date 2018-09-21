/*
* console shm monitor.
*
* Attach to the shared memory segment and display data
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

int end_prog = 0;
void sighan() { end_prog = 1; }

/*
** Main routine
*/
int main(int ac, char **av)
{
  int x, oc_shmid;
  key_t	oc_key = 201809;
  char c;
  OC_ST *ocp;
  extern int errno, end_prog;
  extern void sighan();

  end_prog = 0;

  signal(SIGHUP, sighan);

		/* attach to shm exchange area */

  if ((oc_shmid = shmget(oc_key, sizeof(OC_ST), 0)) == -1 ||
      (ocp = (OC_ST *)shmat(oc_shmid, NULL, 0)) == (OC_ST *)-1) {
    printf("OCC : shmget/shmctl/shmat error (errno = %d).\n", errno);
    exit(1);
    }

  printf("\e[H;\e[J");
  while(end_prog == 0) {
    printf("\e[H");
    printf("Contents of ocp data structure :\n");
    printf("  Line = %s, cpu = %d\n", ocp->line, ocp->cpu_model);
    printf("  sim_is_running     : %d\n", ocp_sir);
    printf("  First exam address : 0x%06X\n", ocp->first_exam);
    printf("  First dep address  : 0x%06X\n", ocp->first_dep);
    printf("  Invalid address    : 0x%06X\n", ocp->inv_addr);
    printf("  Active address     : 0x%06X\n", ocp->act_addr);
//    printf("  Active data        : 0x%04X\n", ocp->act_data);
    printf("  MMR0, MMR3         : 0x%02X, 0x%02X\n", ocp->MMR0, ocp->MMR3);
    printf("  HALT value         : %d\n", ocp->HALT);
    printf("  Port 1 data        : 0x%02X\n", ocp->PORT1);
    printf("  Port 2 data        : 0x%02X\n", ocp->PORT2);
    printf("  CMD from SIMH      : %1c (%01X)\n", ocp->to_cp, ocp->to_cp);
    printf("  CMD to SIMH        : %1c (%01X)\n", ocp->fm_cp, ocp->fm_cp);
    printf("  Acknowledge mask   : %d\n", ocp->ACK);
    printf("  Address array      : KERNI : 0x%06X, KERND : 0x%06X\n",
    		ocp->A[0], ocp->A[1]);
    printf("                     : SUPRI : 0x%06X, SUPRD : 0x%06X\n",
    		ocp->A[2], ocp->A[3]);
    printf("                     : USERI : 0x%06X, USERD : 0x%06X\n",
    		ocp->A[6], ocp->A[7]);
    printf("                     : PRGPA : 0x%06X, CONPA : 0x%06X\n",
    		ocp->A[8], ocp->A[9]);
    printf("  Data array         : SHFR  : 0x%06X, BR    : 0x%06X\n",
    		ocp->D[0], ocp->D[1]);
    printf("                     : FPP   : 0x%06X, DR    : 0x%06X\n",
    		ocp->D[2], ocp->D[3]);
    printf("  Switch data array  : 0x%02X,0x%02X,0x%02X,0x%02X,0x%02X\n",
    	ocp->S[0], ocp->S[1], ocp->S[2], ocp->S[3], ocp->S[4]);
    }

  shmdt((char *)ocp);				/* detach shared mem	*/
  exit(0);
}
