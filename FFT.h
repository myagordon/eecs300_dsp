#include <arduinoFFT.h>

// Sampling and FFT stuff
#define NUM_SAMPLES         512          // Must be a power of 2
#define SAMPLING_FREQ   8192         // Hz, must be 40000 or less due to ADC conversion time. Determines maximum frequency that can be analysed by the FFT Fmax=sampleF/2.

/*
 * Function:  printBins
 * --------------------
 * prints the bins in a table format
 * E.g. the following tells us that we have a strength of 40 for the frequency range of 0 - 255 Hz,
 * and we have a strenght of 80 for the frequency range of 256 - 511 Hz
 * | 0-  | 256-  |
 * | 255 | 511   |
 * _______________
 *   40  | 80    |
 *
 *  returns: void
 */
void printBins(int bins[]);
