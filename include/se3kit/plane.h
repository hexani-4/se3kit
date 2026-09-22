/**
 * @file plane.h
 * @brief Plane geometry for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_PLANE_H
#define SE3KIT_PLANE_H

#include <stdbool.h>
#include <math.h>
#include "line.h"
#include "vec.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_PLANE_DIV_MIN                   (1e-6f)
#define SE3_PLANE_DIV_MIN_SQR               (SE3_PLANE_DIV_MIN * SE3_PLANE_DIV_MIN)
#define SE3_PLANE_NORM_SQR_TOLERANCE        (2e-4f)
#define SE3_PLANE_CONTAINMENT_TOLERANCE     (1e-4f)
#define SE3_PLANE_DIV_FALLBACK              (se3_plane(se3_vec3(0.0f, 0.0f, 1.0f), 0.0f)) // XY Plane

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Canonical 3D plane in Hessian Normal Form: n ⋅ p + d = 0.
 * - normal: Unit normal vector (‖normal‖ == 1).
 * - d:      Signed shortest distance from the coordinate origin to the plane.
 */
typedef struct {
    se3_vec3_t normal;
    float      d;
} se3_plane_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_plane_t se3_plane(se3_vec3_t normal, float d);
static inline se3_plane_t se3_plane_from_normal_and_point(se3_vec3_t normal, se3_vec3_t p);
static inline se3_plane_t se3_plane_from_points(se3_vec3_t p1, se3_vec3_t p2, se3_vec3_t p3);

// Unary operations
static inline bool        se3_plane_is_valid(se3_plane_t plane);
static inline se3_plane_t se3_plane_normalize(se3_plane_t plane);   // normal / ‖normal‖
static inline se3_plane_t se3_plane_flip(se3_plane_t plane);        // {-normal, -d} (Flips the active half-space)

// Geometric queries
static inline float       se3_plane_distance_to_point(se3_plane_t plane, se3_vec3_t p); // Signed distance (n ⋅ p + d)
static inline bool        se3_plane_contains_point(se3_plane_t plane, se3_vec3_t p);    // Checks if distance is within tolerance
static inline se3_vec3_t  se3_plane_project_point(se3_plane_t plane, se3_vec3_t p);
static inline se3_vec3_t  se3_plane_project_origin(se3_plane_t plane);                  // Evaluates to -d * normal directly

// Intersections
static inline se3_vec3_t  se3_plane_intersect_line(se3_plane_t plane, se3_line_t line); // Evaluates to SE3_VEC3_DIV_FALLBACK if parallel

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_plane_t se3_plane(se3_vec3_t normal, float d) {
    return (se3_plane_t){.normal = normal, .d = d};
}

static inline se3_plane_t se3_plane_from_normal_and_point(se3_vec3_t normal, se3_vec3_t p) {
    se3_vec3_t n = se3_vec3_normalize(normal);
    return (se3_plane_t){
        .normal = n,
        .d = -se3_vec3_dot(n, p)
    };
}

static inline se3_plane_t se3_plane_from_points(se3_vec3_t p1, se3_vec3_t p2, se3_vec3_t p3) {
    se3_vec3_t v12 = se3_vec3_sub(p2, p1);
    se3_vec3_t v13 = se3_vec3_sub(p3, p1);
    se3_vec3_t n = se3_vec3_normalize(se3_vec3_cross(v12, v13));
    
    return (se3_plane_t){
        .normal = n,
        .d = -se3_vec3_dot(n, p1)
    };
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

static inline bool se3_plane_is_valid(se3_plane_t plane) {
    float norm_sq = se3_vec3_norm_sqr(plane.normal);
    return fabsf(norm_sq - 1.0f) <= SE3_PLANE_NORM_SQR_TOLERANCE && !isnan(plane.d);
}

static inline se3_plane_t se3_plane_normalize(se3_plane_t plane) {
    float len_sq = se3_vec3_norm_sqr(plane.normal);
    
    if (len_sq < SE3_PLANE_DIV_MIN_SQR) {
        return SE3_PLANE_DIV_FALLBACK;
    }
    
    float inv_len = 1.0f / sqrtf(len_sq);
    return se3_plane(se3_vec3_mul_scalar(plane.normal, inv_len), plane.d * inv_len);
}

static inline se3_plane_t se3_plane_flip(se3_plane_t plane) {
    return se3_plane(se3_vec3_negate(plane.normal), -plane.d);
}

// =============================================================================
// Geometric query definitions ------------------------------------------------|
// =============================================================================

static inline float se3_plane_distance_to_point(se3_plane_t plane, se3_vec3_t p) {
    return se3_vec3_dot(plane.normal, p) + plane.d;
}

static inline bool se3_plane_contains_point(se3_plane_t plane, se3_vec3_t p) {
    return fabsf(se3_plane_distance_to_point(plane, p)) <= SE3_PLANE_CONTAINMENT_TOLERANCE;
}

static inline se3_vec3_t se3_plane_project_point(se3_plane_t plane, se3_vec3_t p) {
    float dist = se3_plane_distance_to_point(plane, p);
    return se3_vec3_sub(p, se3_vec3_mul_scalar(plane.normal, dist));
}

static inline se3_vec3_t se3_plane_project_origin(se3_plane_t plane) {
    return se3_vec3_mul_scalar(plane.normal, -plane.d);
}

// =============================================================================
// Intersection definitions ---------------------------------------------------|
// =============================================================================

static inline se3_vec3_t se3_plane_intersect_line(se3_plane_t plane, se3_line_t line) {
    float dot_dir = se3_vec3_dot(plane.normal, line.dir);
    
    if (fabsf(dot_dir) < SE3_PLANE_DIV_MIN) {
        return SE3_VEC3_DIV_FALLBACK;
    }

    se3_vec3_t p0 = se3_line_project_origin(line);
    float dist_p0 = se3_plane_distance_to_point(plane, p0);
    float t = -dist_p0 / dot_dir;
    
    return se3_vec3_add(p0, se3_vec3_mul_scalar(line.dir, t));
}

#endif /* SE3KIT_PLANE_H */
