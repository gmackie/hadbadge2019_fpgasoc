#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>

#include "mach_defines.h"
#include "sdk.h"
#include "graphics.h"

// A quick note on timing.  Everything here is in duration counts.

// A duration count is 5.33 ms, or 1000 * 256 / 48MHz 
// On a musical scale, a quarter note at 120 BPM is 500 ms
//  so 80 durations is a quarter note at 140.625 BPM (fast)
//     90 durations is a quarter note at 125     BPM
//    100 durations is a quarter note at  93.75  BPM (mellow)

void main(int argc, char **argv)
{
    do_graphics();
	// Finito bandito!
	button_wait();
}

