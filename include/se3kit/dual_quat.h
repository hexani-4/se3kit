
/**
 * @file dual_quat.h
 * @brief Rigidbody transformations for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_DUAL_QUAT_H
#define SE3KIT_DUAL_QUAT_H

#include "axis_angle.h"
#include "quat.h"
#include "line.h"
#include "plane.h"
#include "cone.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_DUAL_QUAT_SCLERP_NEGLIGIBLE_ANGLE   (1e-3f)
#define SE3_DUAL_QUAT_ORTHOGONALITY_TOLERANCE   (1e-4f)

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
static inline bool se3_dual_quat_is_valid(se3_dual_quat_t dq);
static inline se3_dual_quat_t se3_dual_quat_negate(se3_dual_quat_t dq);             // {-r, -d}
static inline se3_dual_quat_t se3_dual_quat_conjugate_quat(se3_dual_quat_t dq);     // {r*, d*}
static inline se3_dual_quat_t se3_dual_quat_conjugate_dual(se3_dual_quat_t dq);     // {r, -d}
static inline se3_dual_quat_t se3_dual_quat_conjugate_total(se3_dual_quat_t dq);    // {r*, -d*}
static inline se3_dual_quat_t se3_dual_quat_normalize(se3_dual_quat_t dq);          // {r / ‖r‖, r ⋅ d = 0}

// Transforms
static inline se3_vec3_t se3_dual_quat_transform_vec3(se3_dual_quat_t dq, se3_vec3_t v);    // R * v + t
static inline se3_vec3_t se3_dual_quat_rotate_vec3(se3_dual_quat_t dq, se3_vec3_t v);       // R * v (no translation)
static inline se3_line_t  se3_dual_quat_transform_line(se3_dual_quat_t dq, se3_line_t line);
static inline se3_plane_t se3_dual_quat_transform_plane(se3_dual_quat_t dq, se3_plane_t plane);
static inline se3_cone_t  se3_dual_quat_transform_cone(se3_dual_quat_t dq, se3_cone_t cone);

// Interpolation, time-stepping
static inline se3_dual_quat_t se3_dual_quat_sclerp(se3_dual_quat_t a, se3_dual_quat_t b, float blend);  // screw linear interpolation

// Conversions
static inline se3_dual_quat_t se3_dual_quat_from_pivot_rotation_translation(se3_vec3_t world_trans, se3_quat_t rot, se3_vec3_t pivot, se3_vec3_t local_trans);
static inline se3_dual_quat_t se3_dual_quat_from_rotation_translation(se3_vec3_t world_trans, se3_quat_t rot, se3_vec3_t local_trans);
static inline se3_dual_quat_t se3_dual_quat_from_rotation(se3_quat_t rot);
static inline se3_dual_quat_t se3_dual_quat_from_translation(se3_vec3_t trans);

static inline se3_quat_t se3_dual_quat_to_rotation(se3_dual_quat_t dq);
static inline se3_vec3_t se3_dual_quat_to_translation(se3_dual_quat_t dq);

// =============================================================================
// Helpers --------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Constructs a unit dual quaternion from orientation and resolved net translation.
 */
static inline se3_dual_quat_t se3_dual_quat_from_rot_net_trans_(se3_quat_t rot, se3_vec3_t net_trans) {
    se3_quat_t t_quat = se3_quat_from_vec3(net_trans);
    se3_quat_t dual = se3_quat_mul_scalar(se3_quat_mul(t_quat, rot), 0.5f);
    return se3_dual_quat(rot, dual);
}

/**
 * @brief Returns @p flippable signed to form short-arc path from @p base.
 * @param flippable Dual quaternion to potentially negate (+dq or -dq).
 * @param base      Reference dual quaternion defining the target hemisphere.
 * @return Either @p flippable or -@p flippable, guaranteeing (base.real ⋅ result.real) >= 0
 */
static inline se3_dual_quat_t se3_dual_quat_short_arc(se3_dual_quat_t flippable, se3_dual_quat_t base) {
    return (se3_quat_dot(base.real, flippable.real) < 0.0f) ? se3_dual_quat_negate(flippable) : flippable;
}

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_dual_quat_t se3_dual_quat(se3_quat_t real, se3_quat_t dual) {
    return (se3_dual_quat_t){.real = real, .dual = dual};
}

static inline se3_dual_quat_t se3_dual_quat_zero(void) {
    return se3_dual_quat(se3_quat_zero(), se3_quat_zero());
}

static inline se3_dual_quat_t se3_dual_quat_identity(void) {
    return se3_dual_quat(se3_quat_identity(), se3_quat_zero());
}

// =============================================================================
// Basic binary operation definitions -----------------------------------------|
// =============================================================================

static inline se3_dual_quat_t se3_dual_quat_add(se3_dual_quat_t a, se3_dual_quat_t b) {
    return se3_dual_quat(
        se3_quat_add(a.real, b.real),
        se3_quat_add(a.dual, b.dual)
    );
}

static inline se3_dual_quat_t se3_dual_quat_sub(se3_dual_quat_t a, se3_dual_quat_t b) {
    return se3_dual_quat(
        se3_quat_sub(a.real, b.real),
        se3_quat_sub(a.dual, b.dual)
    );
}

static inline se3_dual_quat_t se3_dual_quat_mul_scalar(se3_dual_quat_t dq, float s) {
    return se3_dual_quat(
        se3_quat_mul_scalar(dq.real, s),
        se3_quat_mul_scalar(dq.dual, s)
    );
}

// =============================================================================
// Advanced binary operation definitions --------------------------------------|
// =============================================================================

static inline se3_dual_quat_t se3_dual_quat_mul(se3_dual_quat_t a, se3_dual_quat_t b) {
    se3_quat_t real = se3_quat_mul(a.real, b.real);
    se3_quat_t dual = se3_quat_add(
        se3_quat_mul(a.real, b.dual),
        se3_quat_mul(a.dual, b.real)
    );
    return se3_dual_quat(real, dual);
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

/**
 * @brief Checks if a dual quaternion satisfies unit SE(3) invariants:
 *        1. ||real|| == 1 (valid rotation quaternion).
 *        2. real ⋅ dual == 0 (orthogonality / Plücker condition).
 */
static inline bool se3_dual_quat_is_valid(se3_dual_quat_t dq) {
    if (!se3_quat_is_valid(dq.real)) {
        return false;
    }

    float dot_rd = se3_quat_dot(dq.real, dq.dual);
    return fabsf(dot_rd) <= SE3_DUAL_QUAT_ORTHOGONALITY_TOLERANCE;
}

static inline se3_dual_quat_t se3_dual_quat_negate(se3_dual_quat_t dq) {
    return se3_dual_quat(se3_quat_negate(dq.real), se3_quat_negate(dq.dual));
}

/**
 * @brief Quaternion conjugate: qr* + ε qd*. Forms SE(3) kinematic inverse for unit dual quaternions.
 */
static inline se3_dual_quat_t se3_dual_quat_conjugate_quat(se3_dual_quat_t dq) {
    return se3_dual_quat(se3_quat_conjugate(dq.real), se3_quat_conjugate(dq.dual));
}

/**
 * @brief Dual conjugate: qr - ε qd. Negates translation component.
 */
static inline se3_dual_quat_t se3_dual_quat_conjugate_dual(se3_dual_quat_t dq) {
    return se3_dual_quat(dq.real, se3_quat_negate(dq.dual));
}

/**
 * @brief Total conjugate: qr* - ε qd*. Used for Plücker line sandwich products.
 */
static inline se3_dual_quat_t se3_dual_quat_conjugate_total(se3_dual_quat_t dq) {
    return se3_dual_quat(se3_quat_conjugate(dq.real), se3_quat_negate(se3_quat_conjugate(dq.dual)));
}

/**
 * @brief Normalizes a dual quaternion to satisfy ||r|| = 1 and r ⋅ d = 0.
 */
static inline se3_dual_quat_t se3_dual_quat_normalize(se3_dual_quat_t dq) {
    float mag_sqr = se3_quat_norm_sqr(dq.real);
    if (mag_sqr < SE3_QUAT_DIV_MIN_SQR) {
        return se3_dual_quat_identity();
    }

    float inv_mag = 1.0f / sqrtf(mag_sqr);
    se3_quat_t r_norm = se3_quat_mul_scalar(dq.real, inv_mag);

    // Enforce Plücker condition (r ⋅ d = 0) by removing parallel projection
    float dot_rd = se3_quat_dot(r_norm, dq.dual);
    se3_quat_t d_clean = se3_quat_sub(dq.dual, se3_quat_mul_scalar(r_norm, dot_rd));
    se3_quat_t d_norm = se3_quat_mul_scalar(d_clean, inv_mag);

    return se3_dual_quat(r_norm, d_norm);
}

// =============================================================================
// Transform definitions ------------------------------------------------------|
// =============================================================================

/**
 * @brief Rotates a 3D vector without applying translation (p' = R * v).
 */
static inline se3_vec3_t se3_dual_quat_rotate_vec3(se3_dual_quat_t dq, se3_vec3_t v) {
    return se3_quat_rotate_vec3(dq.real, v);
}

/**
 * @brief Transforms a 3D point in SE(3): p' = R * p + t.
 */
static inline se3_vec3_t se3_dual_quat_transform_vec3(se3_dual_quat_t dq, se3_vec3_t p) {
    se3_vec3_t rot_v = se3_quat_rotate_vec3(dq.real, p);
    se3_vec3_t trans = se3_dual_quat_to_translation(dq);
    return se3_vec3_add(rot_v, trans);
}

/**
 * @brief Transforms a Plücker line in SE(3).
 *        d' = R*d
 *        m' = R*m + t × d'
 */
static inline se3_line_t se3_dual_quat_transform_line(se3_dual_quat_t dq, se3_line_t line) {
    se3_vec3_t d_prime = se3_quat_rotate_vec3(dq.real, line.dir);
    se3_vec3_t m_prime = se3_quat_rotate_vec3(dq.real, line.moment);
    se3_vec3_t t = se3_dual_quat_to_translation(dq);
    
    return se3_line(d_prime, se3_vec3_add(m_prime, se3_vec3_cross(t, d_prime)));
}

/**
 * @brief Transforms a plane in SE(3).
 *        n' = R*n
 *        d' = d - (n' ⋅ t)
 */
static inline se3_plane_t se3_dual_quat_transform_plane(se3_dual_quat_t dq, se3_plane_t plane) {
    se3_vec3_t n_prime = se3_quat_rotate_vec3(dq.real, plane.normal);
    se3_vec3_t t = se3_dual_quat_to_translation(dq);
    
    return se3_plane(n_prime, plane.d - se3_vec3_dot(n_prime, t));
}

/**
 * @brief Transforms a cone in SE(3).
 */
static inline se3_cone_t se3_dual_quat_transform_cone(se3_dual_quat_t dq, se3_cone_t cone) {
    se3_vec3_t d_prime = se3_quat_rotate_vec3(dq.real, cone.axis.dir);
    se3_vec3_t m_prime = se3_quat_rotate_vec3(dq.real, cone.axis.moment);
    se3_vec3_t t = se3_dual_quat_to_translation(dq);
    
    se3_line_t new_axis = se3_line(d_prime, se3_vec3_add(m_prime, se3_vec3_cross(t, d_prime)));
    
    return (se3_cone_t){
        .axis = new_axis,
        .apex_coord = cone.apex_coord + se3_vec3_dot(t, d_prime),
        .cos_half_angle = cone.cos_half_angle
    };
}

// =============================================================================
// Interpolation definitions --------------------------------------------------|
// =============================================================================

/**
 * @brief Screw Linear Interpolation (ScLERP) along the shortest geodesic path in SE(3).
 * Constant screw pitch and angular velocity, falls back to linear displacement for pure translations.
 * @param a     Start transformation (blend = 0.0f).
 * @param b     End transformation (blend = 1.0f).
 * @param blend Interpolation factor [0.0f, 1.0f].
 * @return Interpolated unit dual quaternion.
 */
static inline se3_dual_quat_t se3_dual_quat_sclerp(se3_dual_quat_t a, se3_dual_quat_t b, float blend) {
    b = se3_dual_quat_short_arc(b, a);

    // Relative displacement: delta = a* * b
    se3_dual_quat_t a_inv = se3_dual_quat_conjugate_quat(a);
    se3_dual_quat_t delta = se3_dual_quat_mul(a_inv, b);

    // Extract rotation angle and axis
    se3_vec3_t qv = se3_quat_to_vec3(delta.real);
    float sin_half = se3_vec3_norm(qv);

    se3_dual_quat_t delta_pow;
    if (sin_half < SE3_DUAL_QUAT_SCLERP_NEGLIGIBLE_ANGLE) {
        // Pure translation: Delta^t = 1 + ε * (blend * delta.dual)
        delta_pow = se3_dual_quat(
            se3_quat_identity(),
            se3_quat_mul_scalar(delta.dual, blend)
        );
    } else {
        float cos_half = delta.real.w;
        float half_angle = atan2f(sin_half, cos_half);
        float inv_sin = 1.0f / sin_half;

        // Unit screw axis direction vector
        se3_vec3_t n = se3_vec3_mul_scalar(qv, inv_sin);

        // Pitch displacement along screw axis: d/2 = -delta_d.w / sin(theta/2)
        float half_d = -delta.dual.w * inv_sin;

        // Moment vector: m = (delta_d.v - half_d * cos(theta/2) * n) / sin(theta/2)
        se3_vec3_t qd_v = se3_quat_to_vec3(delta.dual);
        se3_vec3_t m = se3_vec3_mul_scalar(
            se3_vec3_sub(qd_v, se3_vec3_mul_scalar(n, half_d * cos_half)),
            inv_sin
        );

        // Scale angle and pitch by blend factor
        float t_half_angle = blend * half_angle;
        float t_half_d = blend * half_d;

        float sin_t = sinf(t_half_angle);
        float cos_t = cosf(t_half_angle);

        // Real component: cos(t*theta/2) + sin(t*theta/2) * n
        se3_vec3_t r_v = se3_vec3_mul_scalar(n, sin_t);
        se3_quat_t r_pow = se3_quat(cos_t, r_v.x, r_v.y, r_v.z);

        // Dual component: -t_half_d * sin(t*theta/2) + sin(t*theta/2) * m + t_half_d * cos(t*theta/2) * n
        float d_w = -t_half_d * sin_t;
        se3_vec3_t d_v = se3_vec3_add(
            se3_vec3_mul_scalar(m, sin_t),
            se3_vec3_mul_scalar(n, t_half_d * cos_t)
        );
        se3_quat_t d_pow = se3_quat(d_w, d_v.x, d_v.y, d_v.z);

        delta_pow = se3_dual_quat(r_pow, d_pow);
    }

    return se3_dual_quat_mul(a, delta_pow);
}

// =============================================================================
// Conversion definitions -----------------------------------------------------|
// =============================================================================

/**
 * @brief Constructs a dual quaternion from compound hierarchical placement:
 *        p' = R * (p + local_trans - pivot) + pivot + world_trans
 */
static inline se3_dual_quat_t se3_dual_quat_from_pivot_rotation_translation(
    se3_vec3_t world_trans,
    se3_quat_t rot,
    se3_vec3_t pivot,
    se3_vec3_t local_trans
) {
    se3_vec3_t offset = se3_vec3_sub(local_trans, pivot);
    se3_vec3_t rot_offset = se3_quat_rotate_vec3(rot, offset);
    se3_vec3_t net_trans = se3_vec3_add(world_trans, se3_vec3_add(pivot, rot_offset));
    return se3_dual_quat_from_rot_net_trans_(rot, net_trans);
}

/**
 * @brief Constructs a dual quaternion from compound hierarchical placement:
 *        p' = R * (p + local_trans) + world_trans
 */
static inline se3_dual_quat_t se3_dual_quat_from_rotation_translation(
    se3_vec3_t world_trans,
    se3_quat_t rot,
    se3_vec3_t local_trans
) {
    se3_vec3_t rot_local_trans = se3_quat_rotate_vec3(rot, local_trans);
    se3_vec3_t net_trans = se3_vec3_add(world_trans, rot_local_trans);
    return se3_dual_quat_from_rot_net_trans_(rot, net_trans);
}

/**
 * @brief Constructs a dual quaternion representing pure rotation (zero translation).
 */
static inline se3_dual_quat_t se3_dual_quat_from_rotation(se3_quat_t rot) {
    return se3_dual_quat(rot, se3_quat_zero());
}

/**
 * @brief Constructs a dual quaternion representing pure translation (identity rotation).
 */
static inline se3_dual_quat_t se3_dual_quat_from_translation(se3_vec3_t trans) {
    se3_quat_t t_quat = se3_quat_from_vec3(trans);
    return se3_dual_quat(se3_quat_identity(), se3_quat_mul_scalar(t_quat, 0.5f));
}

/**
 * @brief Extracts the rotation quaternion from a dual quaternion.
 */
static inline se3_quat_t se3_dual_quat_to_rotation(se3_dual_quat_t dq) {
    return dq.real;
}

/**
 * @brief Extracts the net reference/world translation vector: t = 2 * dual * real*.
 */
static inline se3_vec3_t se3_dual_quat_to_translation(se3_dual_quat_t dq) {
    se3_quat_t t_quat = se3_quat_mul(
        se3_quat_mul_scalar(dq.dual, 2.0f),
        se3_quat_conjugate(dq.real)
    );
    return se3_quat_to_vec3(t_quat);
}

#endif /* SE3KIT_DUAL_QUAT_H */
