/**
 * @file quat.h
 * @brief Rigidbody transformations for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_QUAT_H
#define SE3KIT_QUAT_H

#include <float.h>
#include <math.h>
#include "axis_angle.h"
#include "rpy.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_QUAT_DIV_MIN        (1e-6f)
#define SE3_QUAT_DIV_MIN_SQR    (SE3_QUAT_DIV_MIN * SE3_QUAT_DIV_MIN)
#define SE3_QUAT_DIV_FALLBACK   (se3_quat_identity())

#define SE3_QUAT_NORM_SQR_TOLERANCE    (2e-4f)
#define SE3_QUAT_GIMBAL_LOCK_THRESHOLD (0.9995f)

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Hamilton quaternion representation: q = w + xi + yj + zk
 * Scalar-first convention.
 */
typedef struct {
    float w;
    float x;
    float y;
    float z;
} se3_quat_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_quat_t se3_quat(float w, float x, float y, float z);  // {w, x, y, z}
static inline se3_quat_t se3_quat_zero(void);                           // {0, 0, 0, 0}
static inline se3_quat_t se3_quat_identity(void);                       // {1, 0, 0, 0}

// Basic binary operations
static inline se3_quat_t se3_quat_add(se3_quat_t a, se3_quat_t b);      // a + b
static inline se3_quat_t se3_quat_sub(se3_quat_t a, se3_quat_t b);      // a - b
static inline se3_quat_t se3_quat_mul_scalar(se3_quat_t q, float s);    // q * s
static inline se3_quat_t se3_quat_div_scalar(se3_quat_t q, float s);    // q / s

// Advanced binary operations (products, etc.)
static inline float se3_quat_dot(se3_quat_t a, se3_quat_t b);       // a ⋅ b
static inline se3_quat_t se3_quat_mul(se3_quat_t a, se3_quat_t b);  // a * b

// Unary operations
static inline bool se3_quat_is_valid(se3_quat_t q);         // ‖q‖ == 1
static inline float se3_quat_norm_sqr(se3_quat_t q);        // q ⋅ q
static inline float se3_quat_norm(se3_quat_t q);            // √(q ⋅ q)
static inline se3_quat_t se3_quat_negate(se3_quat_t q);     // q * -1
static inline se3_quat_t se3_quat_conjugate(se3_quat_t q);  // q*
static inline se3_quat_t se3_quat_inverse(se3_quat_t q);    // q* / ‖q‖²
static inline se3_quat_t se3_quat_normalize(se3_quat_t q);  // q / ‖q‖

// Transforms
static inline se3_vec3_t se3_quat_rotate_vec3(se3_quat_t q, se3_vec3_t v); // q * v * q*

// Interpolation, time-stepping
static inline se3_quat_t se3_quat_nlerp(se3_quat_t a, se3_quat_t b, float blend); // normalized linear interpolation (fast)
static inline se3_quat_t se3_quat_slerp(se3_quat_t a, se3_quat_t b, float blend); // spherical linear interpolation (constant-velocity)

// Conversions
static inline se3_quat_t se3_quat_from_vec3(se3_vec3_t v);              // vector to pure quaternion {0, x, y, z}
static inline se3_quat_t se3_quat_from_axis_angle(se3_axis_angle_t aa); // axis angle to quaternion
static inline se3_quat_t se3_quat_from_rpy(se3_rpy_t rpy);              // RPY to quaternion

static inline se3_vec3_t se3_quat_to_vec3(se3_quat_t q);                // quaternion imaginary part to vector {x, y, z}
static inline se3_axis_angle_t se3_quat_to_axis_angle(se3_quat_t q);    // quaternion to axis angle
static inline se3_rpy_t se3_quat_to_rpy(se3_quat_t q);                  // quaternion to RPY

// Decomposition
static inline void se3_quat_decompose_twist_swing(se3_quat_t q, se3_vec3_t twist_axis, float *out_twist_angle, se3_quat_t *out_remaining);

// =============================================================================
// Helpers --------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Returns @p flippable signed to form the short-arc path from @p base.
 *
 * @param flippable Quaternion to potentially negate (+q or -q).
 * @param base      Reference quaternion defining the target hemisphere.
 * @return Either @p flippable or -@p flippable, guaranteeing (base ⋅ result) >= 0.
 */
static inline se3_quat_t se3_quat_short_arc(se3_quat_t flippable, se3_quat_t base) {
    return (se3_quat_dot(base, flippable) < 0.0f) ? se3_quat_negate(flippable) : flippable;
}

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_quat_t se3_quat(float w, float x, float y, float z) {
    return (se3_quat_t){.w = w, .x = x, .y = y, .z = z};
}

static inline se3_quat_t se3_quat_zero(void) { 
    return se3_quat(0.0f, 0.0f, 0.0f, 0.0f);
}

static inline se3_quat_t se3_quat_identity(void) {
    return se3_quat(1.0f, 0.0f, 0.0f, 0.0f);
}

// =============================================================================
// Basic binary operation definitions -----------------------------------------|
// =============================================================================

static inline se3_quat_t se3_quat_add(se3_quat_t a, se3_quat_t b) {
    return se3_quat(a.w + b.w, a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline se3_quat_t se3_quat_sub(se3_quat_t a, se3_quat_t b) {
    return se3_quat(a.w - b.w, a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline se3_quat_t se3_quat_mul_scalar(se3_quat_t q, float s) {
    return se3_quat(q.w * s, q.x * s, q.y * s, q.z * s);
}

static inline se3_quat_t se3_quat_div_scalar(se3_quat_t q, float s) {
    if (fabsf(s) < SE3_QUAT_DIV_MIN) {
        return SE3_QUAT_DIV_FALLBACK;
    }

    float inv_s = 1.0f / s;
    return se3_quat_mul_scalar(q, inv_s);
}

// =============================================================================
// Advanced binary operation (products, etc.) definitions ---------------------|
// =============================================================================

static inline float se3_quat_dot(se3_quat_t a, se3_quat_t b) {
    return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

static inline se3_quat_t se3_quat_mul(se3_quat_t a, se3_quat_t b) {
    return se3_quat(
        (a.w * b.w) - (a.x * b.x) - (a.y * b.y) - (a.z * b.z),
        (a.w * b.x) + (a.x * b.w) + (a.y * b.z) - (a.z * b.y),
        (a.w * b.y) - (a.x * b.z) + (a.y * b.w) + (a.z * b.x),
        (a.w * b.z) + (a.x * b.y) - (a.y * b.x) + (a.z * b.w)
    );
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

/**
 * @brief Checks if a quaternion represents a valid rotation in SO(3) (‖q‖ == 1).
 */
static inline bool se3_quat_is_valid(se3_quat_t q) {
    float norm_sq = se3_quat_norm_sqr(q);
    return fabsf(norm_sq - 1.0f) <= SE3_QUAT_NORM_SQR_TOLERANCE;
}

static inline float se3_quat_norm_sqr(se3_quat_t q) {
    return se3_quat_dot(q, q);
}

static inline float se3_quat_norm(se3_quat_t q) {
    return sqrtf(se3_quat_norm_sqr(q));
}

static inline se3_quat_t se3_quat_negate(se3_quat_t q) {
    return se3_quat(-q.w, -q.x, -q.y, -q.z);
}

static inline se3_quat_t se3_quat_conjugate(se3_quat_t q) {
    return se3_quat(q.w, -q.x, -q.y, -q.z);
}

static inline se3_quat_t se3_quat_inverse(se3_quat_t q) {
    float norm_sqr = se3_quat_norm_sqr(q);

    if (norm_sqr < SE3_QUAT_DIV_MIN_SQR) {
        return SE3_QUAT_DIV_FALLBACK;
    }

    float inv_norm_sqr = 1.0f / norm_sqr;
    return se3_quat_mul_scalar(se3_quat_conjugate(q), inv_norm_sqr);
}

static inline se3_quat_t se3_quat_normalize(se3_quat_t q) {
    float norm_sqr = se3_quat_norm_sqr(q);

    if (norm_sqr < SE3_QUAT_DIV_MIN_SQR) {
        return SE3_QUAT_DIV_FALLBACK;
    }

    float inv_norm = 1.0f / sqrtf(norm_sqr);
    return se3_quat_mul_scalar(q, inv_norm);
}

// =============================================================================
// Transform definitions ------------------------------------------------------|
// =============================================================================

/**
 * @brief Rotates a 3D vector by a unit quaternion (q * v * q*).
 * @param q Unit rotation quaternion.
 * @param v 3D vector to rotate.
 * @return Rotated vector.
 */
static inline se3_vec3_t se3_quat_rotate_vec3(se3_quat_t q, se3_vec3_t v) {
    // tx = 2 * (q_v × v)
    se3_vec3_t qv = se3_quat_to_vec3(q);
    se3_vec3_t t = se3_vec3_mul_scalar(se3_vec3_cross(qv, v), 2.0f);
    // v' = v + q.w * tx + (q_v × tx)
    se3_vec3_t qwt = se3_vec3_mul_scalar(t, q.w);
    return se3_vec3_add(v, se3_vec3_add(qwt, se3_vec3_cross(qv, t)));
}

// =============================================================================
// Interpolation, time-stepping definitions -----------------------------------|
// =============================================================================

/**
 * @brief Normalized linear interpolation along the shortest arc.
 * Faster than SLERP, but with variable velocity across large arcs.
 * @param a     Start orientation (blend = 0.0f).
 * @param b     End orientation (blend = 1.0f).
 * @param blend Interpolation factor [0.0f, 1.0f].
 * @return Normalized interpolated quaternion.
 */
static inline se3_quat_t se3_quat_nlerp(se3_quat_t a, se3_quat_t b, float blend) {
    b = se3_quat_short_arc(b, a);

    se3_quat_t result = se3_quat(
        a.w + (b.w - a.w) * blend,
        a.x + (b.x - a.x) * blend,
        a.y + (b.y - a.y) * blend,
        a.z + (b.z - a.z) * blend
    );
    return se3_quat_normalize(result);
}

/**
 * @brief Spherical linear interpolation along the shortest arc.
 * Constant interpolation velocity, falls back to NLERP at short arcs.
 * @param a     Start orientation (blend = 0.0f).
 * @param b     End orientation (blend = 1.0f).
 * @param blend Interpolation factor [0.0f, 1.0f].
 * @return Interpolated unit quaternion.
 */
static inline se3_quat_t se3_quat_slerp(se3_quat_t a, se3_quat_t b, float blend) {
    b = se3_quat_short_arc(b, a);

    float dot = se3_quat_dot(a, b);
    if (dot > 0.9995f) {
        // small-angle nlerp fallback (cause: sin(theta) -> 0)
        return se3_quat_nlerp(a, b, blend);
    }

    float theta_0 = acosf(dot);
    float theta = theta_0 * blend;
    float sin_theta_0 = sinf(theta_0);

    float s0 = sinf(theta_0 - theta) / sin_theta_0;
    float s1 = sinf(theta) / sin_theta_0;

    return se3_quat(
        (a.w * s0) + (b.w * s1),
        (a.x * s0) + (b.x * s1),
        (a.y * s0) + (b.y * s1),
        (a.z * s0) + (b.z * s1)
    );
}

// =============================================================================
// Conversion definitions -----------------------------------------------------|
// =============================================================================


/**
 * @brief Constructs a pure quaternion (w = 0) from a 3D vector.
 */
static inline se3_quat_t se3_quat_from_vec3(se3_vec3_t v) {
    return se3_quat(0.0f, v.x, v.y, v.z);
}

/**
 * @brief Extracts the 3D imaginary vector component (x, y, z) of a quaternion.
 */
static inline se3_vec3_t se3_quat_to_vec3(se3_quat_t q) {
    return se3_vec3(q.x, q.y, q.z);
}

/**
 * @brief Constructs a unit quaternion from an axis and rotation angle.
 * @param aa Axis-angle to construct quaternion from, will be normalized internally.
 */
static inline se3_quat_t se3_quat_from_axis_angle(se3_axis_angle_t aa) {
    if (se3_axis_angle_negligible(aa)) {
        return se3_quat_identity();
    }

    aa = se3_axis_angle_normalize(aa);
    float half_angle = aa.angle * 0.5f;
    float cos_ha = cosf(half_angle);
    float sin_ha = sinf(half_angle);

    se3_vec3_t qv = se3_vec3_mul_scalar(aa.axis, sin_ha);
    return se3_quat(cos_ha, qv.x, qv.y, qv.z);
}

/**
 * @brief Constructs an axis-angle rotation equivalent to a quaternion
 * @param q Quaternion to convert to axis-angle.
 * @return Equivalent axis-angle; if the rotation angle is zero, an arbitrary axis is returned.
 */
static inline se3_axis_angle_t se3_quat_to_axis_angle(se3_quat_t q) {
    se3_quat_t uq = se3_quat_normalize(q);
    if (uq.w < 0.0f) { uq = se3_quat_negate(uq); }

    se3_vec3_t qv = se3_quat_to_vec3(uq);
    float s = se3_vec3_norm(qv);

    if (s < SE3_QUAT_DIV_MIN) {
        // effectively zero rotation angle
        return se3_axis_angle_identity();
    }

    float inv_s = 1.0f / s;
    return se3_axis_angle(se3_vec3_mul_scalar(qv, inv_s), 2.0f * atan2f(s, uq.w));
}

/**
 * @brief Constructs a unit quaternion from Tait-Bryan RPY angles (Z-Y-X sequence).
 * Computes q = q_yaw * q_pitch * q_roll.
 */
static inline se3_quat_t se3_quat_from_rpy(se3_rpy_t rpy) {
    float hr = rpy.roll  * 0.5f;
    float hp = rpy.pitch * 0.5f;
    float hy = rpy.yaw   * 0.5f;

    float cr = cosf(hr);
    float sr = sinf(hr);
    float cp = cosf(hp);
    float sp = sinf(hp);
    float cy = cosf(hy);
    float sy = sinf(hy);

    float cy_cp = cy * cp;
    float sy_sp = sy * sp;
    float cy_sp = cy * sp;
    float sy_cp = sy * cp;

    return se3_quat(
        (cy_cp * cr) + (sy_sp * sr),
        (cy_cp * sr) - (sy_sp * cr),
        (cy_sp * cr) + (sy_cp * sr),
        (sy_cp * cr) - (cy_sp * sr)
    );
}

/**
 * @brief Extracts Tait-Bryan RPY angles (Z-Y-X sequence) from a unit quaternion.
 * Resolves gimbal lock at pitch = +/- 90 degrees by locking yaw to zero.
 */
static inline se3_rpy_t se3_quat_to_rpy(se3_quat_t q) {
    se3_quat_t uq = se3_quat_normalize(q);

    // sin(pitch) = 2 * (w*y - z*x)
    float sin_pitch = 2.0f * ((uq.w * uq.y) - (uq.z * uq.x));

    se3_rpy_t rpy;

    if (fabsf(sin_pitch) >= SE3_QUAT_GIMBAL_LOCK_THRESHOLD) {
        rpy.pitch = copysignf(SE3_HALF_PI, sin_pitch);
        rpy.yaw   = 0.0f;
        rpy.roll  = 2.0f * atan2f(uq.x, uq.w);
    } else {
        rpy.pitch = asinf(sin_pitch);

        // roll = atan2(2*(w*x + y*z), 1 - 2*(x^2 + y^2))
        float r_num = 2.0f * ((uq.w * uq.x) + (uq.y * uq.z));
        float r_den = 1.0f - (2.0f * ((uq.x * uq.x) + (uq.y * uq.y)));
        rpy.roll = atan2f(r_num, r_den);

        // yaw = atan2(2*(w*z + x*y), 1 - 2*(y^2 + z^2))
        float y_num = 2.0f * ((uq.w * uq.z) + (uq.x * uq.y));
        float y_den = 1.0f - (2.0f * ((uq.y * uq.y) + (uq.z * uq.z)));
        rpy.yaw = atan2f(y_num, y_den);
    }

    return rpy;
}

// =============================================================================
// Decomposition definitions --------------------------------------------------|
// =============================================================================

/**
 * @brief Decomposes a rotation into a twist around a specific normalized axis 
 *        and a remaining swing. 
 *        Result satisfies: q = remaining * twist (swing-then-twist).
 *
 * @param q               The input rotation quaternion.
 * @param twist_axis      The normalized axis of allowed rotation (e.g., a hinge joint).
 * @param out_twist_angle Output signed angle of rotation around twist_axis (radians).
 * @param out_remaining   Output remaining rotation.
 */
static inline void se3_quat_decompose_twist_swing(
    se3_quat_t q, 
    se3_vec3_t twist_axis, 
    float *out_twist_angle, 
    se3_quat_t *out_remaining
) {
    se3_vec3_t q_vec = se3_quat_to_vec3(q);
    
    // Project the vector part of the quaternion onto the constraint axis
    float proj = se3_vec3_dot(q_vec, twist_axis);

    // Singularity check: 180-degree rotation strictly orthogonal to the twist axis.
    // In this state, the twist around our target axis is effectively zero.
    if (fabsf(q.w) < SE3_QUAT_DIV_MIN && fabsf(proj) < SE3_QUAT_DIV_MIN) {
        if (out_twist_angle) *out_twist_angle = 0.0f;
        if (out_remaining)   *out_remaining = q;
        return;
    }

    // atan2f(y, x) elegantly recovers the half-angle without requiring normalization or square roots.
    float half_angle = atan2f(proj, q.w);
    
    if (out_twist_angle) {
        *out_twist_angle = 2.0f * half_angle;
    }

    if (out_remaining) {
        // Construct the conjugate of the twist quaternion to extract the remainder
        float c = cosf(half_angle);
        float s = sinf(half_angle);
        se3_quat_t twist_conj = se3_quat(c, -twist_axis.x * s, -twist_axis.y * s, -twist_axis.z * s);
        
        // remaining = q * twist^-1
        *out_remaining = se3_quat_mul(q, twist_conj);
    }
}

#endif /* SE3KIT_QUAT_H */
