/**
 * @file angle.h
 * @brief Scalar angle operations on S1 for embedded C11.
 * Part of se3kit. */

#ifndef SE3KIT_ANGLE_H
#define SE3KIT_ANGLE_H

#include <math.h>

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#ifndef SE3_PI
#define SE3_PI          (3.14159265358979323846f)
#endif

#ifndef SE3_HALF_PI
#define SE3_HALF_PI     (1.57079632679489661923f)
#endif

#ifndef SE3_TWO_PI
#define SE3_TWO_PI      (6.28318530717958647692f)
#endif

#ifndef SE3_DEG_TO_RAD
#define SE3_DEG_TO_RAD  (0.01745329251994329576f)
#endif

#ifndef SE3_RAD_TO_DEG
#define SE3_RAD_TO_DEG  (57.2957795130823208767f)
#endif

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Normalization & Wrapping
static inline float se3_angle_wrap_pi(float rad);     // (-pi, pi]
static inline float se3_angle_wrap_two_pi(float rad); // [0, 2pi)

// Distance
static inline float se3_angle_diff(float target, float source); // Shortest signed arc from source to target

// Conversions
static inline float se3_deg_to_rad(float deg);
static inline float se3_rad_to_deg(float rad);

// =============================================================================
// Normalization & Wrapping definitions ---------------------------------------|
// =============================================================================

/**
 * @brief Wraps an angle in radians to the half-open interval (-pi, pi].
 */
static inline float se3_angle_wrap_pi(float rad) {
    float wrapped = remainderf(rad, SE3_TWO_PI);
    if (wrapped <= -SE3_PI) {
        wrapped = SE3_PI;
    }
    return wrapped;
}

/**
 * @brief Wraps an angle in radians to the half-closed interval [0, 2pi).
 */
static inline float se3_angle_wrap_two_pi(float rad) {
    float a = fmodf(rad, SE3_TWO_PI);
    if (a < 0.0f) {
        a += SE3_TWO_PI;
    }
    return a;
}

// =============================================================================
// Distance definitions -------------------------------------------------------|
// =============================================================================

/**
 * @brief Computes the shortest signed angular difference (target - source) in (-pi, pi].
 */
static inline float se3_angle_diff(float target, float source) {
    return se3_angle_wrap_pi(target - source);
}

// =============================================================================
// Conversion definitions -----------------------------------------------------|
// =============================================================================

static inline float se3_deg_to_rad(float deg) {
    return deg * SE3_DEG_TO_RAD;
}

static inline float se3_rad_to_deg(float rad) {
    return rad * SE3_RAD_TO_DEG;
}


#endif /* SE3KIT_ANGLE_H */
