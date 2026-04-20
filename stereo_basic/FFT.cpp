#include "FFT.h"

void printBins(int bins[])
{
  static int binWidth = SAMPLING_FREQ / 32;
  for(int i = 0; i < 256; ++i)
  {
    Serial.print("_");
  }
  Serial.print("\n");
  for(int i = 0; i < 16; ++i)
  {
    Serial.print("|\t");
    Serial.print(i * binWidth);
    Serial.print("-");
    Serial.print("\t");
  }
  Serial.print("|");
  Serial.print("\n");
  for(int i = 0; i < 16; ++i)
  {
    Serial.print("|\t");
    Serial.print((i + 1) * binWidth - 1);
    Serial.print("\t");
  }
  Serial.print("|");
  Serial.print("\n");
  for(int i = 0; i < 256; ++i)
  {
    Serial.print("_");
  }
  Serial.print("|");
  Serial.print("\n");
  for(int i = 0; i < 16; ++i)
  {
    Serial.print("|\t");
    Serial.print(bins[i] / 100);// so we son't have to read big numbers (we only care about these values relative to each other)
    Serial.print("\t");
  }
  Serial.print("|");
  Serial.println("\n");
}