/*
Copyright (c) 2012, Daniel M. Lofaro
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the author nor the names of its contributors may 
      be used to endorse or promote products derived from this software 
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT, 
INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <net/if.h>
#include <time.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <string.h>
#include <stdio.h>

// for timer
#include <time.h>
#include <sched.h>
#include <sys/io.h>
#include <unistd.h>

// for RT
#include <stdlib.h>
#include <sys/mman.h>

// for hubo
#include "../../hubo-ach/include/hubo.h"

// for ach
#include <errno.h>
#include <fcntl.h>
#include <assert.h>
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include "ach.h"

//#include "../include/hubo_ref_filter.h"
#include "hubo-ref-filter.h"


/* At time of writing, these constants are not defined in the headers */
#ifndef PF_CAN
#define PF_CAN 29
#endif

#ifndef AF_CAN
#define AF_CAN PF_CAN
#endif

/* ... */

/* Somewhere in your app */

// Priority
#define MY_PRIORITY (49)/* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */

#define MAX_SAFE_STACK (1024*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */


// Timing info
#define NSEC_PER_SEC    1000000000

char* fileName = "";
//int interval = 1000000000; // 1hz (1.0 sec)
//int interval = 500000000; // 2hz (0.5 sec)
int interval =   40000000; // 25 hz (0.04 sec)
//int interval = 20000000; // 50 hz (0.02 sec)
//int interval = 10000000; // 100 hz (0.01 sec)
//int interval = 5000000; // 200 hz (0.005 sec)
//int interval = 2000000; // 500 hz (0.002 sec)

struct timeb {
        time_t   time;
        unsigned short millitm;
        short    timezone;
        short    dstflag;
};



/* functions */
void stack_prefault(void);
static inline void tsnorm(struct timespec *ts);
void getMotorPosFrame(int motor, struct can_frame *frame);
int huboLoop();
int ftime(struct timeb *tp);
int getArg(char* s,struct hubo_ref *r);
int runTraj(char* s, struct hubo_ref *r, struct timespec *t);
// ach message type
//typedef struct hubo h[1];

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach
ach_channel_t chan_hubo_ref_filter;      // hubo-ach-filter
ach_channel_t chan_hubo_init_cmd; // hubo-ach-console
ach_channel_t chan_hubo_state;    // hubo-ach-state
ach_channel_t chan_hubo_param;    // hubo-ach-param

int debug = 0;
int hubo_debug = 1;
int i = 0;
int huboLoop() {
	double newRef[2] = {1.0, 0.0};
        // get initial values for hubo
        struct hubo_ref H_ref;
        struct hubo_ref H_ref_filter;
	struct hubo_ref H_ref_filter_buff;
	struct hubo_state H_state;
	memset( &H_ref,   0, sizeof(H_ref));
	memset( &H_ref_filter,   0, sizeof(H_ref_filter));
	memset( &H_ref_filter_buff,   0, sizeof(H_ref_filter_buff));

        size_t fs;
        //int r = ach_get( &chan_hubo_ref, &H, sizeof(H), &fs, NULL, ACH_O_LAST );
        //assert( sizeof(H) == fs );
	int r = ach_get( &chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(hubo_debug) {
                       	printf("Ref ini r = %s\n",ach_result_to_string(r));}
		}
	else{   assert( sizeof(H_ref) == fs ); }

	r = ach_get( &chan_hubo_ref_filter, &H_ref_filter, sizeof(H_ref_filter), &fs, NULL, ACH_O_LAST );
	if(ACH_OK != r) {
		if(hubo_debug) {
                       	printf("State ini r = %s\n",ach_result_to_string(r));}
		}
	else{   
		assert( sizeof(H_ref_filter) == fs );
	 }

        // time info
        struct timespec t;


	/* Sampling Period */
	double T = (double)interval/(double)NSEC_PER_SEC; // (sec)
	clock_gettime( 0,&t);
// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT AVBOE THIS LINE]---------------------------------
// ------------------------------------------------------------------------------

//	char* fileName = "valve0.traj";

	runTraj(fileName, &H_ref_filter, &t);


//	runTraj("ybTest1.traj",&H_ref_filter, &t);

//	runTraj("valve0.traj", &H_ref_filter, &t);
//	runTraj("valve1.traj", &H_ref_filter, &t);
//	runTraj("valve2.traj", &H_ref_filter, &t);
//	runTraj("valve1.traj", &H_ref_filter, &t);
//	runTraj("valve2.traj", &H_ref_filter, &t);
//	runTraj("valve1.traj", &H_ref_filter, &t);
//	runTraj("valve2.traj", &H_ref_filter, &t);
//	runTraj("valve1.traj", &H_ref_filter, &t);
//	runTraj("valve2.traj", &H_ref_filter, &t);
//	runTraj("valve1.traj", &H_ref_filter, &t);
//	runTraj("valve2.traj", &H_ref_filter, &t);
//	runTraj("valve3.traj", &H_ref_filter, &t);
// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT BELOW THIS LINE]---------------------------------
// ------------------------------------------------------------------------------


	printf("Trajectory Finished\n");
	return 0;
}


int runTraj(char* s, struct hubo_ref *r, struct timespec *t) {
	int i = 0;
// int interval = 10000000; // 100 hz (0.01 sec)

 	char str[1000];
	FILE *fp;		// file pointer
	fp = fopen(s,"r");
	if(!fp) {
		printf("No Trajectory File!!!\n");
		return 1;  // exit if not file
	}

//	printf("Reading %s\n",s);
        while(fgets(str,sizeof(str),fp) != NULL) {
	//	printf("i = %d\n",i);
	//	i = i+1;
                // wait until next shot
                clock_nanosleep(0,TIMER_ABSTIME,t, NULL);

// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT AVBOE THIS LINE]---------------------------------
// ------------------------------------------------------------------------------
		int len = strlen(str)-1;
		if(str[len] == "\n") {
			str[len] = 0;
		}

		getArg(str, r); 	
// ------------------------------------------------------------------------------
// ---------------[ DO NOT EDIT BELOW THIS LINE]---------------------------------
// ------------------------------------------------------------------------------

		// Cheeting No more RAP or LAP
//		r->ref[RHP] = 0.0;
//		r->ref[LHP] = 0.0;
//		r->ref[RAP] = 0.0;
//		r->ref[LAP] = 0.0;
//		r->ref[RKN] = 0.0;
//		r->ref[LKN] = 0.0;

        	ach_put( &chan_hubo_ref_filter, r, sizeof(*r));
		//printf("Ref r = %s\n",ach_result_to_string(r));
                t->tv_nsec+=interval;
                tsnorm(t);
        }

}

int  getArg(char* s,struct hubo_ref *r) {

sscanf(s, "%lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf %lf", 
	&r->ref[RHY],
	&r->ref[RHR],
	&r->ref[RHP],
	&r->ref[RKN],
	&r->ref[RAP],
	&r->ref[RAR],
	&r->ref[LHY],
	&r->ref[LHR],
	&r->ref[LHP],
	&r->ref[LKN],
	&r->ref[LAP],
	&r->ref[LAR],
	&r->ref[RSP],
	&r->ref[RSR],
	&r->ref[RSY],
	&r->ref[REB],
	&r->ref[RWY],
	&r->ref[RWR],
	&r->ref[RWP],
	&r->ref[LSP],
	&r->ref[LSR],
	&r->ref[LSY],
	&r->ref[LEB],
	&r->ref[LWY],
	&r->ref[LWR],
	&r->ref[LWP],
	&r->ref[NKY],
	&r->ref[NK1],
	&r->ref[NK2],
	&r->ref[WST],
	&r->ref[RF1],
	&r->ref[RF2],
	&r->ref[RF3],
	&r->ref[RF4],
	&r->ref[RF5],
	&r->ref[LF1],
	&r->ref[LF2],
	&r->ref[LF3],
	&r->ref[LF4],
	&r->ref[LF5]);

        return 0;
}





void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset( dummy, 0, MAX_SAFE_STACK );
}


		
static inline void tsnorm(struct timespec *ts){

//	clock_nanosleep( NSEC_PER_SEC, TIMER_ABSTIME, ts, NULL);
        // calculates the next shot
        while (ts->tv_nsec >= NSEC_PER_SEC) {
                //usleep(100);	// sleep for 100us (1us = 1/1,000,000 sec)
                ts->tv_nsec -= NSEC_PER_SEC;
                ts->tv_sec++;
        }
}

int main(int argc, char **argv) {

        int vflag = 0;
        int c;


        int i = 1;
        while(argc > i) {
                if(strcmp(argv[i], "-d") == 0) { // debug
                        debug = 1;
                }
                if(strcmp(argv[i], "-n") == 0) {
			if( argc > (i+1)) {
	                        fileName = argv[i+1];
				printf("Trejectory file changed\n");
			}
			else {
				printf("ERROR: File name not changed\n");
			}
                }
		if(strcmp(argv[i], "-f") == 0) {
			int j = i+1;
			if( argc > (j)) {
				if(strcmp(argv[j],      "100") == 0) { interval = 10000000; /* 100 hz (0.01 sec) */ }
				else if(strcmp(argv[j], "50") == 0)  { interval = 20000000; /* 50 hz (0.02 sec)  */ }
				else if(strcmp(argv[j], "25") == 0)  { interval = 40000000; /* 25 hz (0.04 sec)  */ }
				else if(strcmp(argv[j], "10") == 0)  { interval = 100000000; /* 25 hz (0.04 sec)  */}
				else if(strcmp(argv[j], "200") == 0) { interval = 5000000;  /* 200 hz (0.005 sec)*/ }
				else if(strcmp(argv[j], "500") == 0) { interval = 2000000;  /* 500 hz (0.002 sec)*/ }
				else { printf("ERROR: Frequency not changed\n"); }
			}
			else {
				printf("ERROR: Frequency not changed\n");
			}
		}
		if(strcmp(argv[i], "-h") == 0) {
                	printf("------------------------------------------\n");
                	printf("-----------hubo-read-trajectory-----------\n");
                	printf("------------------------------------------\n");
			printf("\n");
			printf("Usage: ./hubo-read-trajectory -f 100 -n fileName.traj\n");
			printf("\tOptions:\n");
			printf("\t\t-h   help menu\n");
			printf("\t\t-n   change trajectory\n");
			printf("\t\t\t\tdefault: no file\n");
			printf("\t\t\t\tatguements: filename\n");
			printf("\t\t-f   change frequency\n");
			printf("\t\t\tdefault: 25hz\n");
			printf("\t\t\tatguements: frequency\n");
			printf("\t\t\t\toptions (hz):\n");
			printf("\t\t\t\t\t10\n");
			printf("\t\t\t\t\t25\n");
			printf("\t\t\t\t\t50\n");
			printf("\t\t\t\t\t100\n");
			printf("\t\t\t\t\t200\n");
			printf("\t\t\t\t\t500\n");
			printf("\n");
			printf("File format (Each Column)\n");
			printf("\tRHY RHR RHP RKN RAP RAR LHY LHR LHP LKN LAP LAR RSP RSR RSY REB RWY RWR RWP LSP LSR LSY LEB LWY LWR LWP NKY NK1 NK2 WST RF1 RF2 RF3 RF4 RF5 LF1 LF2 LF3 LF4 LF5\n");
			printf("\n");
			return 0;
		}
		i++;
        }
	

	printf("\n");
	printf("-----------------------------\n");
	printf("Using file: %s \n",fileName);
	printf("Sampling frequency %f\n", 1/((double)interval/(double)NSEC_PER_SEC));
	printf("-----------------------------\n");
	printf("\n");
        /* RT */
        struct sched_param param;
        /* Declare ourself as a real time task */
        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }

        /* Lock memory */
        if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();


        /* open ach channel */
        int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
        assert( ACH_OK == r );

    	/* open ach-filter channel */
    	r = ach_open(&chan_hubo_ref_filter, HUBO_CHAN_REF_FILTER_NAME , NULL);
	
    	assert( ACH_OK == r );

 
	huboLoop();
//        pause();
        return 0;

}
