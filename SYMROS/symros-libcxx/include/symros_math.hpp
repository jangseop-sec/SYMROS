#pragma once
#include <stdint.h>

// ── bit manipulation helper ────────────────────────────────────────────────────────────
static inline uint64_t to_bits(double x) {
    uint64_t b; __builtin_memcpy(&b, &x, 8); return b;
}
static inline double from_bits(uint64_t b) {
    double x; __builtin_memcpy(&x, &b, 8); return x;
}

static inline int symros_isnan(double x)    { uint64_t b=to_bits(x); return ((b>>52)&0x7FF)==0x7FF && (b&0x000FFFFFFFFFFFFFULL)!=0; }
static inline int symros_isinf(double x)    { uint64_t b=to_bits(x); return ((b>>52)&0x7FF)==0x7FF && (b&0x000FFFFFFFFFFFFFULL)==0; }
static inline int symros_isfinite(double x) { return !symros_isnan(x) && !symros_isinf(x); }
static inline int symros_signbit(double x)  { return (to_bits(x) >> 63) != 0; }

static inline double symros_abs(double x)  { return x < 0 ? -x : x; }
static inline double symros_fabs(double x) { return symros_abs(x); }

static inline double symros_copysign(double x, double y) {
    return from_bits((to_bits(x) & 0x7FFFFFFFFFFFFFFFULL) |
                     (to_bits(y) & 0x8000000000000000ULL));
}

static inline double symros_trunc(double x) { return (double)(long long)x; }
static inline double symros_floor(double x) { double t=symros_trunc(x); return x<t ? t-1.0 : t; }
static inline double symros_ceil (double x) { double t=symros_trunc(x); return x>t ? t+1.0 : t; }
static inline double symros_round(double x) { return x>=0 ? symros_floor(x+0.5) : symros_ceil(x-0.5); }
static inline long      symros_lround(double x)  { return (long)     symros_round(x); }
static inline long long symros_llround(double x) { return (long long)symros_round(x); }
static inline double symros_rint(double x) { return symros_round(x); }

// ── min / max / dim ───────────────────────────────────────────────────────────
static inline double symros_fmin(double x, double y) { if(symros_isnan(x))return y; if(symros_isnan(y))return x; return x<y?x:y; }
static inline double symros_fmax(double x, double y) { if(symros_isnan(x))return y; if(symros_isnan(y))return x; return x>y?x:y; }
static inline double symros_fdim(double x, double y) { return x>y ? x-y : 0.0; }

static inline double symros_fmod     (double x, double y) { return x - y * symros_trunc(x/y); }
static inline double symros_remainder(double x, double y) { return x - y * symros_round(x/y); }

static inline double symros_modf(double x, double* iptr) {
    double t = symros_trunc(x); *iptr = t; return x - t;
}
static inline double symros_frexp(double x, int* exp) {
    if (x == 0.0) { *exp = 0; return 0.0; }
    uint64_t b = to_bits(x);
    *exp = (int)((b >> 52) & 0x7FF) - 1022;
    return from_bits((b & 0x800FFFFFFFFFFFFFULL) | (1022ULL << 52));
}
static inline double symros_ldexp(double x, int exp) {
    uint64_t b = to_bits(x);
    int e = (int)((b >> 52) & 0x7FF) + exp;
    return from_bits((b & 0x800FFFFFFFFFFFFFULL) | ((uint64_t)e << 52));
}

static inline double symros_sqrt(double x) {
    if (x < 0) return 0.0/0.0;
    if (x == 0) return 0.0;
    double g = x;
    for (int i = 0; i < 60; i++) g = (g + x/g) * 0.5;
    return g;
}
static inline double symros_cbrt(double x) {
    if (x == 0) return 0.0;
    double s = x<0?-1.0:1.0, a = x<0?-x:x, g = a;
    for (int i = 0; i < 60; i++) g = (2.0*g + a/(g*g)) / 3.0;
    return s * g;
}
static inline double symros_hypot(double x, double y) { return symros_sqrt(x*x + y*y); }
static inline double symros_hypot3(double x, double y, double z) { return symros_sqrt(x*x + y*y + z*z); }

#ifdef __cplusplus
// C++ only: overload so symros_hypot(x, y, z) works without renaming call sites
inline double symros_hypot(double x, double y, double z) { return symros_hypot3(x, y, z); }
#endif

// ── exp / log ─────────────────────────────────────────────────────────────────
static inline double symros_exp(double x) {
    const int N = 20;
    double r = 1.0 + x / (double)(1 << N);
    for (int i = 0; i < N; i++) r *= r;
    return r;
}
static inline double symros_log(double x) {
    if (x <= 0) return x==0 ? -1.0/0.0 : 0.0/0.0;
    uint64_t b = to_bits(x);
    int e = (int)((b >> 52) & 0x7FF) - 1023;
    double m = from_bits((b & 0x800FFFFFFFFFFFFFULL) | (1023ULL << 52));
    double y = (m-1.0)/(m+1.0), y2 = y*y, s = y, t = y;
    for (int i = 1; i <= 30; i++) { t *= y2; s += t/(2*i+1); }
    return 2.0*s + e * 0.6931471805599453;
}
static inline double symros_exp2 (double x) { return symros_exp(x * 0.6931471805599453); }
static inline double symros_log2 (double x) { return symros_log(x) * 1.4426950408889634; }
static inline double symros_log10(double x) { return symros_log(x) * 0.4342944819032518; }

// ── pow ───────────────────────────────────────────────────────────────────────
static inline double symros_pow(double x, double y) {
    if (y == 0) return 1.0;
    if (x == 0) return 0.0;
    if (x < 0 && y == (long long)y) {
        long long n = (long long)y;
        double r = 1.0, b = n<0 ? 1.0/x : x;
        for (long long i = 0, m = n<0?-n:n; i < m; i++) r *= b;
        return r;
    }
    return symros_exp(y * symros_log(x));
}

static inline double symros_sin(double x) {
    const double PI  = 3.141592653589793;
    const double PI2 = 6.283185307179586;
    x = symros_fmod(x, PI2);
    if (x >  PI) x -= PI2;
    if (x < -PI) x += PI2;
    double r=x, t=x, x2=x*x;
    for (int i=1; i<=12; i++) { t *= -x2/((2*i)*(2*i+1)); r += t; }
    return r;
}
static inline double symros_cos(double x) {
    const double PI  = 3.141592653589793;
    const double PI2 = 6.283185307179586;
    x = symros_fmod(x, PI2);
    if (x >  PI) x -= PI2;
    if (x < -PI) x += PI2;
    double r=1, t=1, x2=x*x;
    for (int i=1; i<=12; i++) { t *= -x2/((2*i-1)*(2*i)); r += t; }
    return r;
}
static inline double symros_tan(double x) { return symros_sin(x) / symros_cos(x); }

static inline double symros_atan(double x) {
    const double PI_2 = 1.5707963267948966;
    if (x >  1.0) return  PI_2 - symros_atan(1.0/x);
    if (x < -1.0) return -PI_2 - symros_atan(1.0/x);
    double r=x, t=x, x2=x*x;
    for (int i=1; i<=30; i++) { t *= -x2; r += t/(2*i+1); }
    return r;
}
static inline double symros_asin(double x) { return symros_atan(x / symros_sqrt(1.0 - x*x)); }
static inline double symros_acos(double x) { return 1.5707963267948966 - symros_asin(x); }
static inline double symros_atan2(double y, double x) {
    const double PI = 3.141592653589793;
    if (x > 0)           return symros_atan(y/x);
    if (x < 0 && y >= 0) return symros_atan(y/x) + PI;
    if (x < 0 && y <  0) return symros_atan(y/x) - PI;
    if (x == 0 && y > 0) return  PI/2;
    if (x == 0 && y < 0) return -PI/2;
    return 0.0;
}

// ── fma ───────────────────────────────────────────────────────────────────────
static inline double symros_fma(double x, double y, double z) { return x*y + z; }