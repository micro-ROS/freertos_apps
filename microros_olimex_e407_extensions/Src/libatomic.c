#include <stdbool.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>

void get_lock(uint64_t *mem, int model){
  // RTOS dependant lock implementation
}
void free_lock(uint64_t *mem, int model){
  // RTOS dependant lock implementation
}

uint64_t __atomic_load_8 (uint64_t *mem, int model) { 
  uint64_t ret; 
  get_lock (mem, model); 
  ret = *mem; 
  free_lock (mem, model); 
  return ret; 
}

void __atomic_store_8 (uint64_t *mem, uint64_t val, int model) { 
  get_lock (mem, model); 
  *mem = val; 
  free_lock (mem, model); 
}

uint64_t __atomic_exchange_8 (uint64_t *mem, uint64_t val, int model) { 
  uint64_t ret; 
  get_lock (mem, model); 
  ret = *mem; 
  *mem = val; 
  free_lock (mem, model); 
  return ret; 
}

uint64_t __atomic_fetch_add_8 (uint64_t *mem, uint64_t val, int model) { 
  uint64_t ret; 
  get_lock (mem, model); 
  ret = *mem; 
  *mem += val; 
  free_lock (mem, model); 
  return ret; 
}
