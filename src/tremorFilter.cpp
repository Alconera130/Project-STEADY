#include <Arduino.h>
#include <math.h>

#include "config.h"
#include "tremorFilter.h"

#define SAMPLE_RATE 200.0f
#define MAX_SAMPLES 1600

float xSamples[MAX_SAMPLES];
float ySamples[MAX_SAMPLES];
float zSamples[MAX_SAMPLES];

int sampleCount = 0;

bool learning = true;
bool ready = false;

float detectedFrequency = 0.0f;

float filteredGX = 0.0f;
float filteredGY = 0.0f;
float filteredGZ = 0.0f;

float notchX1 = 0;
float notchX2 = 0;

float notchY1 = 0;
float notchY2 = 0;

float notchZ1 = 0;
float notchZ2 = 0;

float notchGX1 = 0;
float notchGX2 = 0;

float notchGY1 = 0;
float notchGY2 = 0;

float notchGZ1 = 0;
float notchGZ2 = 0;

float b0;
float b1;
float b2;
float a1;
float a2;

void createNotch(float frequency) {
  float w0 = 2.0f * PI * frequency / SAMPLE_RATE;

  float alpha = sin(w0) / (2.0f * NOTCH_Q);

  float cosW = cos(w0);

  float a0 = 1.0f + alpha;

  b0 = 1.0f / a0;
  b1 = -2.0f * cosW / a0;
  b2 = 1.0f / a0;

  a1 = -2.0f * cosW / a0;
  a2 = (1.0f - alpha) / a0;
}

float notch(float input, float &x1, float &x2, float &y1, float &y2) {
    float output =
        b0 * input +
        b1 * x1 +
        b2 * x2 -
        a1 * y1 -
        a2 * y2;

    x2 = x1;
    x1 = input;

    y2 = y1;
    y1 = output;

    return output;
}

float calculatePower(float *data, float frequency) {
    float real = 0.0f;
    float imag = 0.0f;

    float omega =
        2.0f * PI * frequency / SAMPLE_RATE;

    for (int n = 0; n < MAX_SAMPLES; n++) {
        float angle = omega * n;

        real += data[n] * cos(angle);
        imag -= data[n] * sin(angle);
    }

    return real * real + imag * imag;
}

float findDominantFrequency() {
    float bestFrequency = TREMOR_MIN_HZ;
    float bestPower = 0.0f;

    for (float frequency = TREMOR_MIN_HZ; frequency <= TREMOR_MAX_HZ; frequency += TREMOR_STEP_HZ) {
        float px = calculatePower(xSamples, frequency);
        float py = calculatePower(ySamples, frequency);
        float pz = calculatePower(zSamples, frequency);

        float totalPower = px + py + pz;

        if (totalPower > bestPower) {
            bestPower = totalPower;
            bestFrequency = frequency;
        }
    }

    return bestFrequency;
}

void tremorBegin() {
    sampleCount = 0;
    learning = true;
    ready = false;
    detectedFrequency = 0;

    Serial.println();
    Serial.println("TREMOR LEARNING");
    Serial.println("Hold wallet naturally.");
    Serial.println("Do not intentionally move.");
}

void tremorRecord(float gx, float gy, float gz) {
    if (!learning) {
        return;
    }

    if (sampleCount < MAX_SAMPLES) {
        xSamples[sampleCount] = gx;
        ySamples[sampleCount] = gy;
        zSamples[sampleCount] = gz;

        sampleCount++;
    }

    if (sampleCount >= MAX_SAMPLES) {
        Serial.println();
        Serial.println("ANALYZING TREMOR...");

        detectedFrequency =
        findDominantFrequency();

        createNotch(detectedFrequency);

        learning = false;
        ready = true;

        Serial.print("TREMOR FREQUENCY: ");
        Serial.print(detectedFrequency, 2);

        Serial.println(" Hz");

        Serial.println(
        "TREMOR FILTER ACTIVE"
        );
    }
}

    void tremorUpdate(float gx, float gy, float gz) {
    if (!ready) {
        filteredGX = gx;
        filteredGY = gy;
        filteredGZ = gz;

        return;
    }

    filteredGX = notch(gx, notchGX1, notchGX2, notchX1, notchX2);
    filteredGY = notch(gy, notchGY1, notchGY2, notchY1, notchY2);
    filteredGZ = notch(gz, notchGZ1, notchGZ2, notchZ1, notchZ2);
}

bool tremorReady() { return ready; }
float getTremorFrequency() { return detectedFrequency; }
float getFilteredGX() { return filteredGX; }
float getFilteredGY() { return filteredGY; }
float getFilteredGZ() { return filteredGZ; }