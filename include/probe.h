#ifndef PROBE_H
#define PROBE_H

#include <stdbool.h>

void probe_set_protocol(bool useNew);
int probe_task(bool dontRunSWDCommands);

#endif // PROBE_H