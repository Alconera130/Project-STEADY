#ifndef TREMOR_FILTER_H
#define TREMOR_FILTER_H

void tremorBegin();
void tremorRecord(float gx, float gy, float gz);
void tremorUpdate(float gx, float gy, float gz);
bool tremorReady();
float getTremorFrequency();
float getFilteredGX();
float getFilteredGY();
float getFilteredGZ();

#endif