/**
 * Sleep processes for a specified period of time.
 *
 * @author Connor Imes
 * @date 2014-09-02
 */
#include <stdio.h>
#include <stdlib.h>
#include <signal.h>
#include <time.h>

#ifndef DEBUG
#define DEBUG 0
#endif

#define ONE_BILLION 1000000000
#define ARG_OFFSET 2

static inline void usage(char* cmd) {
  printf("Usage:\n");
  printf("\t%s <nanoseconds> <pid> [...]\n", cmd);
}

int main(int argc, char** argv) {
  struct timespec ts_sleep;
  pid_t* pids;
  int i;
  long long nanosec;

  if (argc < 3) {
    usage(argv[0]);
    return 1;
  }

  nanosec = atoll(argv[1]);
  if (nanosec < 0) {
    fprintf(stderr, "Time must be > 0\n");
    return 1;
  }

  pids = calloc(argc - ARG_OFFSET, sizeof(pid_t));
  if (pids == NULL) {
    return 1;
  }
  // send SIGSTOP to processes
  for (i = 0; i < argc - ARG_OFFSET; i++) {
    pids[i] = (pid_t) atol(argv[i + ARG_OFFSET]);
#if DEBUG
    printf("Stopping pid: %ld\n", (long) pids[i]);
#endif
    kill(pids[i], SIGSTOP);
  }

  // now sleep
  ts_sleep.tv_sec = nanosec / ONE_BILLION;
  ts_sleep.tv_nsec = nanosec % ONE_BILLION;
#if DEBUG
  printf("Sleeping for %lld sec, %ld ns\n",
         (long long) ts_sleep.tv_sec,
         ts_sleep.tv_nsec);
#endif
  nanosleep(&ts_sleep, NULL);

  // resume processes
  for (i = 0; i < argc - ARG_OFFSET; i++) {
#if DEBUG
    printf("Resuming pid: %ld\n", (long) pids[i]);
#endif
    kill(pids[i], SIGCONT);
  }

  free(pids);
  return 0;
}
