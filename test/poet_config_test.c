#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
#include "poet_config.h"
#include "poet.h"

poet_control_state_t * cstates_speed;
unsigned int nctl_states;
poet_cpu_state_t * cpu_states;
unsigned int ncpu_states;

void* thread_worker() {
  int i = 0;
  while(1) {
    i++;
  }
}

void test_idle() {
  // test idle states
  unsigned int nthreads = 32;
  pthread_t threads[nthreads];
  int i;
  for (i = 0; i < nthreads; i++) {
    if (pthread_create(&threads[i], NULL, thread_worker, NULL)) {
      fprintf(stderr, "Thread creation failed\n");
      exit(1);
    }
  }
  sleep(5);
  apply_cpu_config(cpu_states, ncpu_states, 0, 1, 15123000000, 1);
  sleep(5);
  for (i = 0; i < nthreads; i++) {
    if (pthread_cancel(threads[i])) {
      fprintf(stderr, "Thread cancel failed\n");
      exit(1);
    }
  }
}

int main() {
  unsigned int curr_state_id;
  unsigned int i;

  if(get_control_states(NULL, &cstates_speed, &nctl_states)) {
    fprintf(stderr, "Failed to get control states.\n");
    return 1;
  }
  printf("get_control_state size: %u\n", nctl_states);
  for (i = 0; i < nctl_states; i++) {
    printf("%u %f %f %u\n", cstates_speed[i].id, cstates_speed[i].speedup,
           cstates_speed[i].cost, cstates_speed[i].idle_partner_id);
  }

  if(get_cpu_states(NULL, &cpu_states, &ncpu_states)) {
    fprintf(stderr, "Failed to get CPU states.\n");
    return 1;
  }
  printf("get_cpu_state size: %u\n", ncpu_states);
  for (i = 0; i < ncpu_states; i++) {
    printf("%u %s %s\n", cpu_states[i].id, cpu_states[i].core_mask, cpu_states[i].freqs);
  }

  if (nctl_states != ncpu_states) {
    fprintf(stderr, "Error: got different number of states for control and cpu.\n");
    return 1;
  }

  if (get_current_cpu_state(cpu_states, ncpu_states, &curr_state_id)) {
    printf("Failed to get current CPU state.\n");
  } else {
    printf("Current state id: %u\n", curr_state_id);
  }
  for (i = 0; i < nctl_states; i++) {
    printf("Applying configuration %u\n", i);
    apply_cpu_config(cpu_states, ncpu_states, i, i == 0 ? nctl_states - 1 : i - 1, 0, 1);
    sleep(1);
    if (get_current_cpu_state(cpu_states, ncpu_states, &curr_state_id)) {
      fprintf(stderr, "Failed to get current CPU state after setting configuration %u\n", i);
    } else {
      printf("New state id: %u\n", curr_state_id);
    }
    sleep(2);
  }

  test_idle();

  free(cstates_speed);
  free(cpu_states);

  return 0;
}
