
/**
 * @file quat.h
 * @brief Rigidbody transformations for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_DUAL_QUAT_H
#define SE3KIT_DUAL_QUAT_H

#include "axis_angle.h"
#include "quat.h"

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Unit dual quaternion representing a rigid body transformation in SE(3).
 * dq = real + ε * dual, where ε² = 0 and ε != 0.
 */
typedef struct {
    se3_quat_t real; // Rotation component (unit quaternion)
    se3_quat_t dual; // Translation component (0.5f * (0, t) * real)
} se3_dual_quat_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_dual_quat_t se3_dual_quat(se3_quat_t real, se3_quat_t dual);  // {real, dual}
static inline se3_dual_quat_t se3_dual_quat_zero(void);                         // {zero, zero}
static inline se3_dual_quat_t se3_dual_quat_identity(void);                     // {identity, zero}

// Basic binary operations
static inline se3_dual_quat_t se3_dual_quat_add(se3_dual_quat_t a, se3_dual_quat_t b); // a + b
static inline se3_dual_quat_t se3_dual_quat_sub(se3_dual_quat_t a, se3_dual_quat_t b); // a - b
static inline se3_dual_quat_t se3_dual_quat_mul_scalar(se3_dual_quat_t dq, float s);   // dq * s

// Advanced binary operations (dual quaternion product, transformation chaining)
static inline se3_dual_quat_t se3_dual_quat_mul(se3_dual_quat_t a, se3_dual_quat_t b); // a * b

// Unary operations
static inline se3_dual_quat_t se3_dual_quat_negate(se3_dual_quat_t dq);
static inline se3_dual_quat_t se3_dual_quat_conjugate_quat(se3_dual_quat_t dq);     // {r*, d*}
static inline se3_dual_quat_t se3_dual_quat_conjugate_dual(se3_dual_quat_t dq);     // {r, -d}
static inline se3_dual_quat_t se3_dual_quat_conjugate_total(se3_dual_quat_t dq);    // {r*, -d*}
static inline se3_dual_quat_t se3_dual_quat_normalize(se3_dual_quat_t dq);          // {r / ‖r‖, r ⋅ d = 0}

// Transforms
static inline se3_vec3_t se3_dual_quat_transform_vec3(se3_dual_quat_t dq, se3_vec3_t v);    // R * v + t
static inline se3_vec3_t se3_dual_quat_rotate_vec3(se3_dual_quat_t dq, se3_vec3_t v);       // R * v (no translation)

// Conversions
static inline se3_dual_quat_t se3_dual_quat_from_pivot_rotation_translation(se3_vec3_t world_trans, se3_quat_t rot, se3_vec3_t pivot, se3_vec3_t local_trans);
static inline se3_dual_quat_t se3_dual_quat_from_rotation_translation(se3_vec3_t world_trans, se3_quat_t rot, se3_vec3_t local_trans);
static inline se3_dual_quat_t se3_dual_quat_from_rotation(se3_quat_t rot);
static inline se3_dual_quat_t se3_dual_quat_from_translation(se3_vec3_t trans);

static inline se3_quat_t se3_dual_quat_to_rotation(se3_dual_quat_t dq);
static inline se3_vec3_t se3_dual_quat_to_translation(se3_dual_quat_t dq);

#endif /* SE3KIT_DUAL_QUAT_H */
