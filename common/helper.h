// Some basic math helper functions
#pragma once


#include <cmath>
#include <stdio.h>
#include <vector>
#include <cinttypes>

// This is important on gcc. Otherwise it will call the int math versions.
// I have now enabled warnings that check for this.
// See also: https://stackoverflow.com/questions/19159541/disable-math-h-crap-when-working-with-cmath
using std::tan;
using std::atan;
using std::atan2;
using std::abs;
using std::sin;
using std::cos;
using std::sqrt;
using std::pow;

using u64 = long long unsigned int; // uint64_t produces warnings in printf statements on Linux when used with %llu
using u32 = uint32_t;
using u16 = uint16_t;
using u8  = uint8_t;

using s64 = long long int;
using s32 = int32_t;
using s16 = int16_t;
using s8  = int8_t;
static_assert(sizeof(int) == 4, "");
static_assert(sizeof(u64) == 8, "");
static_assert(sizeof(s64) == 8, "");

#ifndef _MSC_VER
#define sprintf_s(buf, ...) snprintf((buf), sizeof(buf), __VA_ARGS__)
#define strcat_s strcat
#endif

template<typename T>
T sq(T x)
{
	return x*x;
}

inline float clamp(float val, float min, float max)
{
	if (val < min) return min;
	if (val > max) return max;
	return val;
}

const float pi = 3.141592653589f;
const float tau = pi*2.0f;

inline float to_degree(float rad)
{
	return rad*180/pi;
}
inline float to_rad(float degree)
{
	return degree*pi/180;
}

