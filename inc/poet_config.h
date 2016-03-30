#ifndef _POET_CONFIG_H
#define _POET_CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "poet.h"

#define DIV_ROUND_UP(N, S) (((N) + (S) - 1) / (S))

// Configure the max number of cores supported
#ifndef POET_MAX_CORES
  #define POET_MAX_CORES 64
#endif
// "0x" + (num required chars) + terminating char
#define POET_LEN_CORE_MASK (2 + DIV_ROUND_UP(POET_MAX_CORES, 4) + 1)
// 7 is the max length of a DVFS frequency in KHz
// (digits) + commas + terminating char; (commas = POET_MAX_CORES - 1)
#define POET_LEN_FREQS ((POET_MAX_CORES * 7) + POET_MAX_CORES)

/**
 * This struct defines a system configuration.
 * It specifies the configuration identifier, the active cores, and the
 * DVFS frequencies to use.
 */
typedef struct {
  unsigned int id;
  /*
   * Indicates which cores on the system are active and which are inactive for
   * a configuration.
   * This is best represented in hex, e.g. 0x0000002B enables 4 cores: 5,3,1,0
   */
  char core_mask[POET_LEN_CORE_MASK];
  /*
   * A comma-delimited list of DVFS frequencies.
   * There should be an entry in this list for all cores on the system, even
   * if they are not active in core_mask. However...
   *
   * If core_mask uses cores in order (e.g. cores 0-2), you only need to list
   * frequencies for those cores.  But, if cores are used out of order
   * (e.g. cores 0,2,3) then there must be a spot in the list for each core up
   * through least the last active core.
   *
   * A '-' should be used to indicate that the frequency of a core (like a
   * disabled one) does not matter.
   */
  char freqs[POET_LEN_FREQS];
} poet_cpu_state_t;

/**
 * Change the CPU configuration on the system by setting the frequency and
 * number of cores to be used as configured in the state with the provided id.
 *
 * Compatible with the poet_apply_func definition.
 *
 * @param states - must be a poet_cpu_state_t* (array).
 * @param num_states
 * @param id
 * @param last_id
 * @param idle_ns
 * @param is_first_apply
 */
void apply_cpu_config(void* states,
                      unsigned int num_states,
                      unsigned int id,
                      unsigned int last_id,
                      unsigned long long idle_ns,
                      unsigned int is_first_apply);

/**
 * Read the control states from the file at the provided path and store in the
 * states pointer (states* is assigned). The number of states found is stored
 * in num_states. Returns 0 on success.
 *
 * The caller is responsible for freeing the memory this function allocates.
 *
 * @param path
 * @param states
 * @param num_states
 */
int get_control_states(const char* path,
                       poet_control_state_t** states,
                       unsigned int* num_states);

/**
 * Read the CPU states from the file at the provided path and store in the
 * states pointer (states* is assigned). The number of states found is stored
 * in num_states. Returns 0 on success.
 *
 * The caller is responsible for freeing the memory this function allocates.
 *
 * @param path
 * @param states
 * @param num_states
 */
int get_cpu_states(const char* path,
                   poet_cpu_state_t** states,
                   unsigned int* num_states);

/**
 * Attempt to determine the current system state and return the id.
 * Set curr_state_id if possible and return 0, otherwise return -1.
 *
 * Compatible with the poet_curr_state_func definition.
 *
 * @param states
 * @param num_states
 * @param curr_state_id
 */
int get_current_cpu_state(const void* states,
                          unsigned int num_states,
                          unsigned int* curr_state_id);

#ifdef __cplusplus
}
#endif

#endif
