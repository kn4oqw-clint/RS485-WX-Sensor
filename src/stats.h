#pragma once
// ============================================================
// stats.h — rolling window with min/max and a least-squares trend.
//
// Trend is a linear regression slope, not a first-vs-last difference.
// Differencing the endpoints is one line of code and is badly behaved:
// a single noisy sample at either end swings the answer completely.
// Least squares uses every sample in the window, which is what you want
// for pressure, where the slope is the actual forecast signal and the
// per-sample noise is comparable to the change you are trying to see.
//
// Samples carry their own timestamp so an irregular sampling interval —
// a watchdog reset, a slow I2C read, a skipped cycle — does not distort
// the slope.
// ============================================================

#include <Arduino.h>
#include "config.h"

template <uint8_t N>
class RollingStats {
public:
    void reset() { count = 0; head = 0; }

    void add(float value, uint32_t timeSec) {
        vals[head]  = value;
        times[head] = timeSec;
        head = (head + 1) % N;
        if (count < N) count++;
    }

    uint8_t size() const { return count; }
    bool    empty() const { return count == 0; }

    float latest() const {
        if (!count) return 0.0f;
        return vals[(head + N - 1) % N];
    }

    float min() const {
        float m = vals[idx(0)];
        for (uint8_t i = 1; i < count; i++) if (vals[idx(i)] < m) m = vals[idx(i)];
        return m;
    }

    float max() const {
        float m = vals[idx(0)];
        for (uint8_t i = 1; i < count; i++) if (vals[idx(i)] > m) m = vals[idx(i)];
        return m;
    }

    float mean() const {
        if (!count) return 0.0f;
        float s = 0;
        for (uint8_t i = 0; i < count; i++) s += vals[idx(i)];
        return s / count;
    }

    // Slope in units per hour. Returns 0 with fewer than 3 samples —
    // two points always fit a line perfectly, so a "trend" from two
    // readings is just noise wearing a hat.
    float trendPerHour() const {
        if (count < 3) return 0.0f;

        // Time relative to the first sample keeps the sums small and
        // avoids losing precision on large epoch values in a float.
        uint32_t t0 = times[idx(0)];
        float sx = 0, sy = 0, sxx = 0, sxy = 0;
        for (uint8_t i = 0; i < count; i++) {
            float x = (float)(times[idx(i)] - t0);
            float y = vals[idx(i)];
            sx += x; sy += y; sxx += x * x; sxy += x * y;
        }

        float n     = (float)count;
        float denom = n * sxx - sx * sx;
        if (fabsf(denom) < 1e-6f) return 0.0f;      // all samples same instant

        float slopePerSec = (n * sxy - sx * sy) / denom;
        return slopePerSec * 3600.0f;
    }

private:
    float    vals[N];
    uint32_t times[N];
    uint8_t  count = 0;
    uint8_t  head  = 0;

    // Oldest-first indexing into the ring.
    uint8_t idx(uint8_t i) const {
        return (uint8_t)((head + N - count + i) % N);
    }
};

// Saturating float -> int16 with scaling, so an absurd reading cannot
// silently wrap into a plausible-looking one on the wire.
static inline int16_t scaleI16(float v, float scale) {
    float s = v * scale;
    if (s >  32767.0f) return  32767;
    if (s < -32768.0f) return -32768;
    return (int16_t)lroundf(s);
}

static inline uint16_t scaleU16(float v, float scale) {
    float s = v * scale;
    if (s > 65535.0f) return 65535;
    if (s < 0.0f)     return 0;
    return (uint16_t)lroundf(s);
}

static inline uint32_t scaleU32(float v, float scale) {
    float s = v * scale;
    if (s > 4294967000.0f) return 4294967000UL;
    if (s < 0.0f)          return 0;
    return (uint32_t)llroundf(s);
}
