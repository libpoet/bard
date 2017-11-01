#define _GNU_SOURCE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sched.h>
#include <unistd.h>
#include "poet.h"
#include "poet_config.h"

#ifndef POET_CONTROL_STATE_CONFIG_FILE
  #define POET_CONTROL_STATE_CONFIG_FILE "/etc/poet/control_config"
#endif
#ifndef POET_CPU_STATE_CONFIG_FILE
  #define POET_CPU_STATE_CONFIG_FILE "/etc/poet/cpu_config"
#endif

#ifdef POET_CONFIG_DVFS_GOVERNOR_PERFORMANCE
  #define POET_CONFIG_DVFS_GOVERNOR "performance"
  #define POET_CONFIG_DVFS_FILE "scaling_max_freq"
#else
  #define POET_CONFIG_DVFS_GOVERNOR "userspace"
  #define POET_CONFIG_DVFS_FILE "scaling_setspeed"
#endif

#ifndef POET_CONFIG_IDLE_PATH
  // binary that enforces idling our process
  #define POET_CONFIG_IDLE_PATH "bard_idle"
#endif

/**
 * Get the current number of CPUs allocated for this process.
 */
static inline cpu_set_t* alloc_curr_cpu_assignment(unsigned long* num_cpus) {
  cpu_set_t* cur_mask;
  long num_configured_cpus = sysconf(_SC_NPROCESSORS_CONF);
  if (num_configured_cpus <= 0) {
    fprintf(stderr, "alloc_curr_cpu_assignment: Failed to get the number of "
            "configured CPUs\n");
    return NULL;
  }

  cur_mask = CPU_ALLOC(num_configured_cpus);
  if (cur_mask == NULL) {
    fprintf(stderr, "alloc_curr_cpu_assignment: Failed to alloc cpu_set_t\n");
    return NULL;
  }

  if (sched_getaffinity(getpid(), sizeof(cur_mask), cur_mask)) {
    fprintf(stderr, "alloc_curr_cpu_assignment: Failed to get CPU affinity\n");
    CPU_FREE(cur_mask);
    return NULL;
  }
  *num_cpus = num_configured_cpus;
  return cur_mask;
}

/**
 * Compare the current CPU governor state with the provided one.
 * Returns -1 on failure.
 */
static inline int cpu_governor_cmp(unsigned int cpu, const char* governor) {
  FILE* fp;
  char buffer[128];
  int governor_cmp = -1;
  size_t len;

  snprintf(buffer, sizeof(buffer),
           "/sys/devices/system/cpu/cpu%u/cpufreq/scaling_governor", cpu);
  fp = fopen(buffer, "r");
  if (fp == NULL) {
    fprintf(stderr, "cpu_governor_cmp: Failed to open %s\n", buffer);
    return -1;
  }

  if (fgets(buffer, sizeof(buffer), fp) != NULL) {
    // replace trailing newline
    len = strlen(buffer);
    if (len > 0 && buffer[len - 1] == '\n') {
        buffer[len - 1] = '\0';
    }
    governor_cmp = strcmp(governor, buffer);
  }
  fclose(fp);

  return governor_cmp;
}

/**
 * Attempt to get the current frequency of the cpu.
 */
static inline unsigned long get_curr_cpu_frequency(unsigned long cpu) {
  FILE* fp;
  char buffer[128];
  unsigned long curr_freq = 0;

  snprintf(buffer, sizeof(buffer),
           "/sys/devices/system/cpu/cpu%lu/cpufreq/scaling_cur_freq", cpu);
  fp = fopen(buffer, "r");
  if (fp == NULL) {
    fprintf(stderr, "get_curr_cpu_frequency: Failed to open %s\n", buffer);
    return 0;
  }

  if (fgets(buffer, sizeof(buffer), fp) != NULL) {
    curr_freq = strtoul(buffer, NULL, 0);
  } else {
    fprintf(stderr,
            "get_curr_cpu_frequency: Failed to read frequency from core %lu\n",
            cpu);
  }
  fclose(fp);

  return curr_freq;
}

// Read current CPU frequencies for assigned cores.
// Returns -1 if bad DVFS governor is found for an assigned core.
static inline int get_cpu_frequencies(cpu_set_t* curr_cpu_mask,
                                      unsigned long* curr_freq_assignment,
                                      unsigned long ncpus) {
  unsigned long i;
  for (i = 0; i < ncpus; i++) {
    // assigned cores must be in proper scaling governor,
    // otherwise could get a false reading
    if(CPU_ISSET_S(i, CPU_ALLOC_SIZE(ncpus), curr_cpu_mask) &&
       cpu_governor_cmp(i, POET_CONFIG_DVFS_GOVERNOR)) {
      fprintf(stderr,
              "get_cpu_frequencies: Not all affected cores in "
              POET_CONFIG_DVFS_GOVERNOR" governor\n");
      return -1;
    }
    curr_freq_assignment[i] = get_curr_cpu_frequency(i);
  }
  return 0;
}

// parse core and frequency lists and fill in the assignments arrays
static inline int parse_cores_and_freqs(const char* state_core_mask,
                                        const char* state_freqs,
                                        unsigned long ncpus,
                                        cpu_set_t* i_core_mask,
                                        unsigned long* i_freq_assignment) {
  unsigned long i = 0;
  char* freqs = strdup(state_freqs);
  if (freqs == NULL) {
    fprintf(stderr, "parse_cores_and_freqs: strdup failed\n");
    return -1;
  }
  char* freq;
  // clear assignments
  CPU_ZERO_S(CPU_ALLOC_SIZE(ncpus), i_core_mask);
  memset(i_freq_assignment, 0, ncpus * sizeof(unsigned long));
  // now parse
  freq = strtok(freqs, ",");
  while (freq != NULL) {
    if (freq[0] != '-') {
      i_freq_assignment[i] = strtoul(freq, NULL, 0);
    }
    freq = strtok(NULL, ",");
    i++;
  }
  char c[] = {'0', 'x', '0', '\0'};
  for (i = 0; i < ncpus; i++) {
    // check if core is active in this state
    // need to convert char representing hex value to a number format
    c[2] = state_core_mask[POET_LEN_CORE_MASK - 2 - (i / 4)];
    if ((1 << (i % 4)) & strtoul(c, NULL, 16)) {
      CPU_SET_S(i, CPU_ALLOC_SIZE(ncpus), i_core_mask);
    }
  }
  free(freqs);
  return 0;
}

static inline int dvfs_freqs_equal(unsigned long* curr_freq_assignment,
                                   unsigned long* i_freq_assignment,
                                   unsigned long ncpus) {
  unsigned long i;
  for (i = 0; i < ncpus; i++) {
    if (i_freq_assignment[i] != 0 &&
        curr_freq_assignment[i] != i_freq_assignment[i]) {
      // we care about this core, but the frequencies don't match
      return 0;
    }
  }
  return 1;
}

// try to get current CPU configuration state
static int get_cpu_state(const poet_cpu_state_t* states,
                         unsigned int num_states,
                         unsigned int* curr_state_id) {
  int ret = -1;
  unsigned long ncpus = 0;
  unsigned long i;
  cpu_set_t* curr_cpu_mask = NULL;
  unsigned long* curr_freq_assignment = NULL;
  cpu_set_t* i_cpu_mask = NULL;
  unsigned long* i_freq_assignment = NULL;

  // first determine the current core assignment
  curr_cpu_mask = alloc_curr_cpu_assignment(&ncpus);
  if (!curr_cpu_mask) {
    return ret;
  }
  curr_freq_assignment = malloc(ncpus * sizeof(unsigned long));
  i_cpu_mask = CPU_ALLOC(ncpus);
  i_freq_assignment = malloc(ncpus * sizeof(unsigned long));

  if (curr_freq_assignment && i_cpu_mask && i_freq_assignment) {
    // now determine the frequency assignments for active cores
    if(get_cpu_frequencies(curr_cpu_mask, curr_freq_assignment, ncpus)) {
        fprintf(stderr, "get_cpu_state: Failed to get CPU frequencies\n");
    } else {
      // search all states for a match
      for (i = 0; i < num_states; i++) {
        // parse core and freq lists for this state
        if (parse_cores_and_freqs(states[i].core_mask, states[i].freqs, ncpus,
                                  i_cpu_mask, i_freq_assignment)) {
          // should only happen with a bad configuration
          fprintf(stderr,
                  "get_cpu_state: Failed to parse config for state %lu", i);
          // no reason we can't keep searching
          continue;
        }
        // compare current CPU and frequency assignments with this state
        if (CPU_EQUAL_S(CPU_ALLOC_SIZE(ncpus), curr_cpu_mask, i_cpu_mask) &&
            dvfs_freqs_equal(curr_freq_assignment, i_freq_assignment, ncpus)) {
          // all fields matched
          *curr_state_id = states[i].id;
          ret = 0;
          // don't break yet - if this is an idle state, we actually want to
          // find its partner state
        }
      }
    }
  } else {
    fprintf(stderr, "get_cpu_state: Failed to malloc\n");
  }

  if (i_cpu_mask) {
    CPU_FREE(i_cpu_mask);
  }
  free(i_freq_assignment);
  CPU_FREE(curr_cpu_mask);
  free(curr_freq_assignment);
  return ret;
}

int get_current_cpu_state(const void* states,
                          unsigned int num_states,
                          unsigned int* curr_state_id) {
  return get_cpu_state((const poet_cpu_state_t*) states, num_states, curr_state_id);
}

static void apply_cpu_idle_state(unsigned long long nanosec) {
  char command[4096];
  snprintf(command, sizeof(command), POET_CONFIG_IDLE_PATH" %llu %ld", nanosec, (long) getpid());
  printf("apply_cpu_idle_state: %s\n", command);
  if (system(command)) {
    fprintf(stderr, "apply_cpu_idle_state: ERROR idling process\n");
  }
}

// Set CPU frequency and number of cores using taskset system call
static void apply_cpu_config_taskset(poet_cpu_state_t* cpu_states,
                                     unsigned int num_states,
                                     unsigned int id,
                                     unsigned int last_id,
                                     unsigned int is_first_apply) {
  int retvalsyscall = 0;
  char command[4096];

  if (id >= num_states || last_id >= num_states) {
    fprintf(stderr, "apply_cpu_config_taskset: id '%u' or last_id '%u' are not "
            "acceptable values, they must be less than the number of states, "
            "'%u'.\n", id, last_id, num_states);
    return;
  }

  if (cpu_states == NULL) {
    fprintf(stderr, "apply_cpu_config_taskset: cpu_states cannot be null.\n");
    return;
  }

  printf("apply_cpu_config_taskset: Applying state: %u\n", id);

  // only run taskset if the core assignment has changed
  if (is_first_apply || strcmp(cpu_states[id].core_mask, cpu_states[last_id].core_mask)) {
    // apply taskset to this PID and all subordinate PIDs
    snprintf(command, sizeof(command),
            "ps -eLf | awk '(/%d/) && (!/awk/) {print $4}' | xargs -n1 taskset -p %s > /dev/null",
            getpid(), cpu_states[id].core_mask);
    printf("apply_cpu_config_taskset: Applying core allocation: %s\n", command);
    retvalsyscall = system(command);
    if (retvalsyscall != 0) {
      fprintf(stderr, "apply_cpu_config_taskset: ERROR running taskset: %d\n",
              retvalsyscall);
    }
  }

  unsigned int i = 0;
  char* freqs = strdup(cpu_states[id].freqs);
  char* freq = strtok(freqs, ",");
  while (freq != NULL) {
    if (freq[0] != '-') {
      snprintf(command, sizeof(command),
              "echo %lu > /sys/devices/system/cpu/cpu%u/cpufreq/"
              POET_CONFIG_DVFS_FILE,
              strtoul(freq, NULL, 0), i);
      printf("apply_cpu_config_taskset: Applying CPU frequency: %s\n", command);
      retvalsyscall = system(command);
      if (retvalsyscall != 0) {
        fprintf(stderr, "apply_cpu_config_taskset: ERROR setting frequencies: %d\n",
                retvalsyscall);
      }
    }
    freq = strtok(NULL, ",");
    i++;
  }
  free(freqs);
}

void apply_cpu_config(void* states,
                      unsigned int num_states,
                      unsigned int id,
                      unsigned int last_id,
                      unsigned long long idle_ns,
                      unsigned int is_first_apply) {
  apply_cpu_config_taskset((poet_cpu_state_t*) states, num_states, id,
                           last_id, is_first_apply);
  // idle this process if desired
  if (idle_ns > 0) {
    apply_cpu_idle_state(idle_ns);
  }
}

static inline unsigned int get_num_states(FILE* rfile) {
  char line[BUFSIZ];
  unsigned int linenum = 0;
  char id_str[BUFSIZ];
  unsigned int nstates = 0;
  unsigned int nstates_tmp;
  while (fgets(line, BUFSIZ, rfile) != NULL) {
    linenum++;
    if (line[0] == '#') {
      continue;
    }
    if (sscanf(line, "%s", id_str) < 1) {
      fprintf(stderr, "get_num_states: Syntax error, line %u.\n", linenum);
      return 0;
    }
    nstates_tmp = strtoul(id_str, NULL, 0) + 1;
    if (nstates_tmp != nstates + 1) {
      fprintf(stderr, "get_num_states: States are missing or out of order.\n");
      return 0;
    }
    nstates = nstates_tmp;
  }
  return nstates;
}

/* Example file:
  #id   speedup     powerup       partner_id
  0     0           0.25          1
  1     1           1             0
  2     1.206124137 1.084785357   0
  3     1.387207669 1.196666697   0
 */
int get_control_states(const char* path,
                       poet_control_state_t** cstates,
                       unsigned int* num_states) {
  poet_control_state_t * states;
  FILE * rfile;
  char line[BUFSIZ];
  unsigned int linenum = 0;
  char argA[BUFSIZ];
  char argB[BUFSIZ];
  char argC[BUFSIZ];
  char argD[BUFSIZ];
  unsigned int id;

  if (cstates == NULL) {
    fprintf(stderr, "get_control_states: cstates cannot be NULL.\n");
    return -1;
  }

  if (path == NULL) {
    path = POET_CONTROL_STATE_CONFIG_FILE;
  }

  rfile = fopen(path, "r");
  if (rfile == NULL) {
    fprintf(stderr, "get_control_states: Could not open file %s\n", path);
    return -1;
  }

  *num_states = get_num_states(rfile);
  if (*num_states == 0) {
    fclose(rfile);
    return -1;
  }
  rewind(rfile);

  // allocate the space
  states = (poet_control_state_t *) malloc(*num_states * sizeof(poet_control_state_t));
  if (states == NULL) {
    fprintf(stderr, "get_control_states: malloc failed.\n");
    return -1;
  }

  // now iterate again to get the lines and fill in the data structure
  while (fgets(line, BUFSIZ, rfile) != NULL) {
    linenum++;
    if (line[0] == '#') {
      continue;
    }

    if (sscanf(line, "%s %s %s %s", argA, argB, argC, argD) < 4) {
      fprintf(stderr, "get_control_states: Syntax error, line %u\n", linenum);
      fclose(rfile);
      free(states);
      return -1;
    }
    id = strtoul(argA, NULL, 0);
    states[id].id = id;
    states[id].speedup = atof(argB);
    states[id].cost = atof(argC);
    states[id].idle_partner_id = strtoull(argD, NULL, 0);
  }

  fclose(rfile);
  *cstates = states;
  return 0;
}

/* Example file:
  #id   core_mask   freqs
  0     0x00000001  250000
  1     0x00000003  250000,400000
  2     0x0000000F  250000,250000,450000,450000
 */
int get_cpu_states(const char* path,
                   poet_cpu_state_t** cstates,
                   unsigned int* num_states) {
  poet_cpu_state_t * states;
  FILE * rfile;
  char line[BUFSIZ];
  unsigned int linenum = 0;
  char argA[8];
  char argB[POET_LEN_CORE_MASK];
  char argC[POET_LEN_FREQS];
  unsigned int id;

  if (cstates == NULL) {
    fprintf(stderr, "get_cpu_states: cstates cannot be NULL.\n");
    return -1;
  }

  if (path == NULL) {
    path = POET_CPU_STATE_CONFIG_FILE;
  }

  rfile = fopen(path, "r");
  if (rfile == NULL) {
    fprintf(stderr, "get_cpu_states: Could not open file %s\n", path);
    return -1;
  }

  *num_states = get_num_states(rfile);
  if (*num_states == 0) {
    fclose(rfile);
    return -1;
  }
  rewind(rfile);

  // allocate the space
  states = (poet_cpu_state_t *) malloc(*num_states * sizeof(poet_cpu_state_t));
  if (states == NULL) {
    fprintf(stderr, "get_cpu_states: malloc failed.\n");
    return -1;
  }

  // now iterate again to get the lines and fill in the data structure
  while (fgets(line, BUFSIZ, rfile) != NULL) {
    linenum++;
    if (line[0] == '#') {
      continue;
    }

    if (sscanf(line, "%s %s %s", argA, argB, argC) < 3) {
      fprintf(stderr, "get_cpu_states: Syntax error, line %u\n", linenum);
      fclose(rfile);
      free(states);
      return -1;
    }
    id = strtoul(argA, NULL, 0);
    states[id].id = id;
    // initialize all core mask values to 0
    memset(states[id].core_mask, '0', POET_LEN_CORE_MASK);
    // copy core configuration to end of array, not beginning
    states[id].core_mask[0] = '0';
    states[id].core_mask[1] = 'x';
    strcpy(&states[id].core_mask[POET_LEN_CORE_MASK - 1 - strlen(&argB[2])],
           &argB[2]);
    strcpy(states[id].freqs, argC);
  }

  fclose(rfile);
  *cstates = states;
  return 0;
}
