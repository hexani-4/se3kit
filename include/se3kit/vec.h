/**
 * @file vec.h
 * @brief Rigidbody transformations for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_VEC_H
#define SE3KIT_VEC_H

#include <math.h>

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_VEC3_DIV_MIN         (1e-6f)
#define SE3_VEC3_DIV_MIN_SQR     (SE3_VEC3_DIV_MIN * SE3_VEC3_DIV_MIN)
#define SE3_VEC3_DIV_FALLBACK    (se3_vec3_zero())

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Vector of three floats.
 */
typedef struct {
    float x;
    float y;
    float z;
} se3_vec3_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_vec3_t se3_vec3(float x, float y, float z); // {x, y, z}
static inline se3_vec3_t se3_vec3_zero(void);                 // {0, 0, 0}

// Basic binary operations
static inline se3_vec3_t se3_vec3_add(se3_vec3_t a, se3_vec3_t b);    // a + b
static inline se3_vec3_t se3_vec3_sub(se3_vec3_t a, se3_vec3_t b);    // a - b
static inline se3_vec3_t se3_vec3_mul_scalar(se3_vec3_t v, float s);  // v * s
static inline se3_vec3_t se3_vec3_div_scalar(se3_vec3_t v, float s);  // v / s

// Advanced binary operations (products, etc.)
static inline float se3_vec3_dot(se3_vec3_t a, se3_vec3_t b);         // a ⋅ b
static inline se3_vec3_t se3_vec3_cross(se3_vec3_t a, se3_vec3_t b);  // a × b

// Unary operations
static inline float se3_vec3_norm_sqr(se3_vec3_t v);        // v ⋅ v
static inline float se3_vec3_norm(se3_vec3_t v);            // √(v ⋅ v)
static inline se3_vec3_t se3_vec3_negate(se3_vec3_t v);     // v * -1
static inline se3_vec3_t se3_vec3_normalize(se3_vec3_t v);  // v / ‖v‖
static inline se3_vec3_t se3_vec3_orthogonal(se3_vec3_t v); // Returns a normalized vector orthogonal to v

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_vec3_t se3_vec3(float x, float y, float z) {
    return (se3_vec3_t){.x = x, .y = y, .z = z};
}

static inline se3_vec3_t se3_vec3_zero(void) {
    return se3_vec3(0.0f, 0.0f, 0.0f);
}

// =============================================================================
// Basic binary operation definitions -----------------------------------------|
// =============================================================================

static inline se3_vec3_t se3_vec3_add(se3_vec3_t a, se3_vec3_t b) {
    return se3_vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}

static inline se3_vec3_t se3_vec3_sub(se3_vec3_t a, se3_vec3_t b) {
    return se3_vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}

static inline se3_vec3_t se3_vec3_mul_scalar(se3_vec3_t v, float s) {
    return se3_vec3(v.x * s, v.y * s, v.z * s);
}

static inline se3_vec3_t se3_vec3_div_scalar(se3_vec3_t v, float s) {
    if (fabsf(s) < SE3_VEC3_DIV_MIN) {
        return SE3_VEC3_DIV_FALLBACK;
    }

    float inv_s = 1.0f / s;
    return se3_vec3_mul_scalar(v, inv_s);
}

// =============================================================================
// Advanced binary operation definitions --------------------------------------|
// =============================================================================

static inline float se3_vec3_dot(se3_vec3_t a, se3_vec3_t b) {
    return (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

static inline se3_vec3_t se3_vec3_cross(se3_vec3_t a, se3_vec3_t b) {
    return se3_vec3(
        (a.y * b.z) - (a.z * b.y),
        (a.z * b.x) - (a.x * b.z),
        (a.x * b.y) - (a.y * b.x)
    );
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

static inline float se3_vec3_norm_sqr(se3_vec3_t v) {
    return se3_vec3_dot(v, v);
}

static inline float se3_vec3_norm(se3_vec3_t v) {
    return sqrtf(se3_vec3_norm_sqr(v));
}

static inline se3_vec3_t se3_vec3_negate(se3_vec3_t v) {
    return se3_vec3(-v.x, -v.y, -v.z);
}

static inline se3_vec3_t se3_vec3_normalize(se3_vec3_t v) {
    float len_sqr = se3_vec3_norm_sqr(v);

    if (len_sqr < SE3_VEC3_DIV_MIN_SQR) {
        return SE3_VEC3_DIV_FALLBACK;
    }

    float inv_len = 1.0f / sqrtf(len_sqr);
    return se3_vec3_mul_scalar(v, inv_len);
}

/**
 * @brief Computes an arbitrary normalized vector orthogonal to @p v.
 *        Assumes @p v is non-zero.
 */
static inline se3_vec3_t se3_vec3_orthogonal(se3_vec3_t v) {
    if (fabsf(v.x) < 0.9f) {
        // v is not strongly aligned with X. Cross X axis (1,0,0) with v.
        // i x v = (0, -v.z, v.y)
        float inv_len = 1.0f / sqrtf(v.y * v.y + v.z * v.z);
        return se3_vec3(0.0f, -v.z * inv_len, v.y * inv_len);
    } else {
        // v is strongly aligned with X. Cross Y axis (0,1,0) with v.
        // j x v = (v.z, 0, -v.x)
        float inv_len = 1.0f / sqrtf(v.x * v.x + v.z * v.z);
        return se3_vec3(v.z * inv_len, 0.0f, -v.x * inv_len);
    }
}

#endif /* SE3KIT_VEC_H */
