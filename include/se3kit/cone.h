/**
 * @file cone.h
 * @brief Spherical cone and friction cone geometry for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_CONE_H #define SE3KIT_CONE_H

#include <stdbool.h>
#include <math.h>
#include "line.h"
#include "quat.h"
#include "vec.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_CONE_DIV_MIN                    (1e-6f)
#define SE3_CONE_DIV_MIN_SQR                (SE3_CONE_DIV_MIN * SE3_CONE_DIV_MIN)
#define SE3_CONE_NORM_SQR_TOLERANCE         (2e-4f)
#define SE3_CONE_ORTHOGONALITY_TOLERANCE    (1e-4f)
#define SE3_CONE_DISTANCE_TOLERANCE         (1e-4f)

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Cone (conical sector) defined by a central Plücker axis line,
 *        an apex point, a half-angle aperture, and spherical radial cutoff.
 */
typedef struct {
    se3_line_t axis;            // Central symmetry axis
    float      apex_coord;      // Apex coordinate (on the axis)
    float      cos_half_angle;  // Aperture (as cosine of the half-angle)
    float      range;           // Spherical radial cutoff (or +infinity)
} se3_cone_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// TODO: check, dedupe, extend, implement

// Initializers
static inline se3_cone_t se3_cone(se3_vec3_t apex, se3_vec3_t axis_dir, float half_angle_rad, float range);
static inline se3_cone_t se3_cone_from_line(se3_line_t axis_line, se3_vec3_t apex, float half_angle_rad, float range);
static inline se3_cone_t se3_cone_friction(se3_vec3_t contact_point, se3_vec3_t normal, float mu); // Unbounded (range = INFINITY)

// Unary operations
static inline bool       se3_cone_is_valid(se3_cone_t cone);

// Geometric containment queries
static inline bool       se3_cone_contains_point(se3_cone_t cone, se3_vec3_t p);       // Spherical sector test: ||p - apex|| <= range && angle <= half_angle
static inline bool       se3_cone_contains_direction(se3_cone_t cone, se3_vec3_t dir); // Angular aperture test only (ignores range)
static inline bool       se3_cone_contains_line(se3_cone_t cone, se3_line_t line);     // Line passes through apex within angular aperture

// Plücker line & generator queries
static inline se3_line_t se3_cone_axis_line(se3_cone_t cone);                                           // Central Plücker axis line
static inline se3_line_t se3_cone_generator_line(se3_cone_t cone, se3_vec3_t tangent_hint, float phi); // Plücker boundary generator at azimuth phi

// Rotational alignment queries
static inline bool       se3_cone_rotation_to_contain_point(se3_cone_t cone, se3_vec3_t p, se3_quat_t *out_rot);       // Minimal rotation around apex
static inline bool       se3_cone_rotation_to_contain_direction(se3_cone_t cone, se3_vec3_t dir, se3_quat_t *out_rot); // Minimal rotation toward direction

// Conversions & Metrics
static inline float      se3_cone_half_angle(se3_cone_t cone);
static inline float      se3_cone_solid_angle(se3_cone_t cone); // 2pi * (1 - cos_angle) steradians

#endif /* SE3KIT_CONE_H */
