/* Offline paired experiment only. Freeze libc time(), used by Quatro's existing
 * srand(time(NULL)); steady_clock, sensor stamps and performance clocks remain real.
 * Load explicitly via LD_PRELOAD. No production binary links this shim. */
#define _GNU_SOURCE
#include <time.h>
#include <stdlib.h>
#include <errno.h>
#include <dlfcn.h>
#include <unistd.h>

time_t time(time_t *output) {
  const char *value=getenv("A2W_FROZEN_MATCHER_TIME");
  time_t result;
  if (value && *value) {
    char *end=0; errno=0;
    long long parsed=strtoll(value,&end,10);
    if (errno || !end || *end || parsed < 0) {_exit(127);}
    result=(time_t)parsed;
    if ((long long)result != parsed) {_exit(127);}
  } else {
    time_t (*original)(time_t *)=(time_t (*)(time_t *))dlsym(RTLD_NEXT,"time");
    if (!original) {_exit(127);}
    result=original(0);
  }
  if (output) {*output=result;}
  return result;
}
