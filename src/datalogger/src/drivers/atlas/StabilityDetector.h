#ifndef _STABILITYDETECTOR_H
#define _STABILITYDETECTOR_H

#include <Arduino.h>
#include <stdio.h>

#define arrayLength(x)    sizeof(x)/sizeof(x[0])

template <typename T, size_t size_>
float getSum( T (&series)[size_])
{
  float sum = 0;
  for(size_t i = 0; i < size_; i++)
  {
    sum+=series[i];
  }
  return sum;
}

template <typename T, size_t size_>
float getDeviasion( T (&series)[size_])
{
  float sum = getSum(series)/size_;
  float std = 0;
  for(size_t id = 0; id<size_; id++)
  {
    std+=pow((series[id]-sum),2);
  }
  return (sqrt(std/(size_-1)));
}

class StabilityDetector 
{
  private:
    bool stable;
    uint8_t stableCount;
    float precision,
          deviasion;
    uint8_t index;
    float valueBuffer[10];
  
  public:
    StabilityDetector (const float precision = 0.1);
    void setPrecision(float precision) { this->precision = precision; };
    void pushToBuffer(const float value);
    bool isStable(void) { return stable; };
    uint8_t getStableCount(void) { return stableCount; };
    float getDeviasionValue(void) { return deviasion; };
};

#endif
