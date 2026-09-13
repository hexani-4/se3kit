/**
 * @file axis_angle.h
 * @brief Rigidbody transformations for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_AXIS_ANGLE_H
#define SE3KIT_AXIS_ANGLE_H

#include <stdbool.h>
#include <math.h>
#include "angle.h"
#include "vec.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_AXIS_ANGLE_NEGLIGIBLE_ANGLE (1e-6f)
#define SE3_AXIS_ANGLE_DIV_MIN          (1e-6f)
#define SE3_AXIS_ANGLE_DIV_MIN_SQR      (SE3_AXIS_ANGLE_DIV_MIN * SE3_AXIS_ANGLE_DIV_MIN)
#define SE3_AXIS_ANGLE_DIV_FALLBACK     (se3_axis_angle_identity())

#define SE3_AXIS_ANGLE_ANGLE_ASSERT_TOLERANCE (1e-4f)
#define SE3_AXIS_ANGLE_AXIS_NORM_SQR_ASSERT_TOLERANCE (2e-4f)

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Axis-angle rotation in SO(3).
 * - axis:  Normalized unit vector indicating rotation axis.
 * - angle: Rotation in radians (right-handed about axis; positive -> clockwise rotation when facing along axis).
 */
typedef struct {
    se3_vec3_t axis;
    float     angle;
} se3_axis_angle_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_axis_angle_t se3_axis_angle(se3_vec3_t axis, float angle);    // {axis, angle}
static inline se3_axis_angle_t se3_axis_angle_identity(void);                   // {{1, 0, 0}, 0}

// Unary operations
static inline bool se3_axis_angle_is_valid(se3_axis_angle_t aa);
static inline bool se3_axis_angle_negligible(se3_axis_angle_t aa); // small transform check
static inline se3_axis_angle_t se3_axis_angle_normalize(se3_axis_angle_t aa); // axis normalization, angle reduction to (-π; π]

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_axis_angle_t se3_axis_angle(se3_vec3_t axis, float angle) {
    return (se3_axis_angle_t){.axis = axis, .angle = angle};
}

static inline se3_axis_angle_t se3_axis_angle_identity(void) {
    return se3_axis_angle(se3_vec3(1.0f, 0.0f, 0.0f), 0.0f);
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

/**
 * @brief Checks if an axis-angle struct satisfies canonical invariants:
 *        unit-length axis and angle in (-pi, pi].
 */
static inline bool se3_axis_angle_is_valid(se3_axis_angle_t aa) {
    if (aa.angle <= -SE3_PI - SE3_AXIS_ANGLE_ANGLE_ASSERT_TOLERANCE ||
        aa.angle >   SE3_PI + SE3_AXIS_ANGLE_ANGLE_ASSERT_TOLERANCE) {
        return false;
    }

    float norm_sq = se3_vec3_norm_sqr(aa.axis);
    return fabsf(norm_sq - 1.0f) <= SE3_AXIS_ANGLE_AXIS_NORM_SQR_ASSERT_TOLERANCE;
}

/**
 * @brief Checks if the rotation angle near-enough to a no-op.
 */
static inline bool se3_axis_angle_negligible(se3_axis_angle_t aa) {
    return fabsf(se3_angle_wrap_pi(aa.angle)) < SE3_AXIS_ANGLE_NEGLIGIBLE_ANGLE;
}

static inline se3_axis_angle_t se3_axis_angle_normalize(se3_axis_angle_t aa) {
    float wrapped_angle = se3_angle_wrap_pi(aa.angle);

    if (fabsf(wrapped_angle) < SE3_AXIS_ANGLE_NEGLIGIBLE_ANGLE) {
        // zero rotation angle, arbitrary axis
        return se3_axis_angle_identity();
    }

    float len_sqr = se3_vec3_norm_sqr(aa.axis);
    if (len_sqr < SE3_AXIS_ANGLE_DIV_MIN_SQR) {
        // zero-length axis with nonzero angle is undefined
        return SE3_AXIS_ANGLE_DIV_FALLBACK;
    }

    float inv_len = 1.0f / sqrtf(len_sqr);
    return se3_axis_angle(se3_vec3_mul_scalar(aa.axis, inv_len), wrapped_angle);
}

#endif /* SE3KIT_AXIS_ANGLE_H */
