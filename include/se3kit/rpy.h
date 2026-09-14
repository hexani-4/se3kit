/**
 * @file rpy.h
 * @brief Rigidbody transformations for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_RPY_H
#define SE3KIT_RPY_H

#include <stdbool.h>
#include <math.h>
#include "angle.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

// Float rounding tolerance
#define SE3_RPY_ASSERT_TOLERANCE (1e-4f)

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Tait-Bryan Roll-Pitch-Yaw angles (Z-Y-X / 3-2-1 convention) in radians.
 * - roll  (phi):   Rotation about body X-axis in (-pi, pi].
 * - pitch (theta): Rotation about intermediate Y-axis in [-pi/2, pi/2].
 * - yaw   (psi):   Rotation about reference Z-axis in (-pi, pi].
 */
typedef struct {
    float roll;
    float pitch;
    float yaw;
} se3_rpy_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_rpy_t se3_rpy(float roll, float pitch, float yaw);
static inline se3_rpy_t se3_rpy_zero(void);

// Unary operations
static inline bool se3_rpy_is_valid(se3_rpy_t rpy);
static inline se3_rpy_t se3_rpy_normalize(se3_rpy_t rpy);

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_rpy_t se3_rpy(float roll, float pitch, float yaw) {
    return (se3_rpy_t){.roll = roll, .pitch = pitch, .yaw = yaw};
}

static inline se3_rpy_t se3_rpy_zero(void) {
    return se3_rpy(0.0f, 0.0f, 0.0f);
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

/**
 * @brief Checks if RPY angles satisfy bounds:
 *        roll in (-pi, pi], pitch in [-pi/2, pi/2], yaw in (-pi, pi].
 */
static inline bool se3_rpy_is_valid(se3_rpy_t rpy) {
    if (fabsf(rpy.pitch) > (SE3_HALF_PI + SE3_RPY_ASSERT_TOLERANCE)) {
        return false;
    }
    if (rpy.roll <= (-SE3_PI - SE3_RPY_ASSERT_TOLERANCE) || rpy.roll > (SE3_PI + SE3_RPY_ASSERT_TOLERANCE)) {
        return false;
    }
    if (rpy.yaw  <= (-SE3_PI - SE3_RPY_ASSERT_TOLERANCE) || rpy.yaw  > (SE3_PI + SE3_RPY_ASSERT_TOLERANCE)) {
        return false;
    }
    return true;
}

/**
 * @brief Normalizes unconstrained RPY angles into canonical Tait-Bryan ranges:
 *        roll in (-pi, pi], pitch in [-pi/2, pi/2], yaw in (-pi, pi].
 */
static inline se3_rpy_t se3_rpy_normalize(se3_rpy_t rpy) {
    float p = se3_angle_wrap_pi(rpy.pitch);
    float r = rpy.roll;
    float y = rpy.yaw;

    if (p > SE3_HALF_PI) {
        p = SE3_PI - p;
        r += SE3_PI;
        y += SE3_PI;
    } else if (p < -SE3_HALF_PI) {
        p = -SE3_PI - p;
        r += SE3_PI;
        y += SE3_PI;
    }

    return se3_rpy(se3_angle_wrap_pi(r), p, se3_angle_wrap_pi(y));
}

#endif /* SE3KIT_RPY_H */
