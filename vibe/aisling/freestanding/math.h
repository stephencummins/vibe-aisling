/* SPDX-License-Identifier: GPL-3.0-only */
/* The landscape's maths (land.c), for the freestanding WebAssembly build, which has no libm. Whole numbers, roots and
 * magnitudes are WebAssembly's own instructions; the sine and the power are series (within a few in a million: paint, not
 * sound; the SDK's quicker ones are too coarse for the paint's slow fades). The engine uses none of this: it has its
 * own. */
#pragma once
#include <stdint.h>
static inline float floorf(float x) { return __builtin_floorf(x); }
static inline float ceilf(float x) { return __builtin_ceilf(x); }
static inline float sqrtf(float x) { return __builtin_sqrtf(x); }
static inline float fabsf(float x) { return __builtin_fabsf(x); }
static inline float fminf(float a, float b) { return a < b ? a : b; }
static inline float fmaxf(float a, float b) { return a > b ? a : b; }
static inline float fmodf(float x, float y) { return x - y * __builtin_truncf(x / y); }
static inline long lroundf(float x) { return (long)(x < 0.0f ? x - 0.5f : x + 0.5f); }
static inline float sinf(float x)
{
    float t = x * 0.15915494f, y, y2;            /* in turns; to -1/2 .. 1/2, then folded to -1/4 .. 1/4 */
    t -= __builtin_floorf(t + 0.5f);
    if (t > 0.25f)
        t = 0.5f - t;
    else if (t < -0.25f)
        t = -0.5f - t;
    y = t * 6.2831853f;
    y2 = y * y;
    return y * (1.0f + y2 * (-1.0f / 6.0f + y2 * (1.0f / 120.0f + y2 * (-1.0f / 5040.0f + y2 * (1.0f / 362880.0f)))));
}
static inline float cosf(float x) { return sinf(x + 1.5707963f); }
static inline float powf(float a, float b)       /* (a > 0, or 0) */
{
    union { float f; uint32_t i; } u;
    float m, s, s2, x, f, r;
    int e, k;
    if (!(a > 0.0f))
        return 0.0f;
    u.f = a;
    e = (int)(u.i >> 23 & 255u) - 127;
    u.i = (u.i & 0x007FFFFFu) | 0x3F800000u;
    m = u.f;                                     /* 1 .. 2; to 0.707 .. 1.414, where the series is short */
    if (m > 1.4142135f) {
        m *= 0.5f;
        e++;
    }
    s = (m - 1.0f) / (m + 1.0f);
    s2 = s * s;
    x = b * ((float)e + 2.8853900f * s * (1.0f + s2 * (1.0f / 3.0f + s2 * (1.0f / 5.0f + s2 * (1.0f / 7.0f + s2 * (1.0f / 9.0f))))));
    if (x < -126.0f)
        return 0.0f;
    if (x > 126.0f)
        x = 126.0f;
    k = (int)__builtin_floorf(x + 0.5f);
    f = (x - (float)k) * 0.69314718f;            /* -0.35 .. 0.35 */
    r = 1.0f + f * (1.0f + f * (0.5f + f * (1.0f / 6.0f + f * (1.0f / 24.0f + f * (1.0f / 120.0f + f * (1.0f / 720.0f))))));
    u.i = (uint32_t)(k + 127) << 23;
    return r * u.f;
}
