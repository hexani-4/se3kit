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

// TODO: check, dedupe, extend, implement

// Initializers
static inline se3_plane_t se3_plane(se3_vec3_t normal, float d);
static inline se3_plane_t se3_plane_from_normal_and_point(se3_vec3_t normal, se3_vec3_t p);
static inline se3_plane_t se3_plane_from_points(se3_vec3_t p1, se3_vec3_t p2, se3_vec3_t p3);

// Unary operations
static inline bool        se3_plane_is_valid(se3_plane_t plane);
static inline se3_plane_t se3_plane_normalize(se3_plane_t plane);   // Enforces ‖normal‖ == 1
static inline se3_plane_t se3_plane_flip(se3_plane_t plane);        // {-normal, -d} (Flips the active half-space)

// Geometric queries
static inline float       se3_plane_distance_to_point(se3_plane_t plane, se3_vec3_t p); // Signed distance (n ⋅ p + d)
static inline bool        se3_plane_contains_point(se3_plane_t plane, se3_vec3_t p);    // Checks if distance is within tolerance
static inline se3_vec3_t  se3_plane_project_point(se3_plane_t plane, se3_vec3_t p);
static inline se3_vec3_t  se3_plane_project_origin(se3_plane_t plane);                  // Evaluates to -d * normal directly

// Intersections
static inline se3_vec3_t  se3_plane_intersect_line(se3_plane_t plane, se3_line_t line); // Evaluates to SE3_VEC3_DIV_FALLBACK if parallel

#endif /* SE3KIT_PLANE_H */
