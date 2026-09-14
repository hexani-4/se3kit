/**
 * @file line.h
 * @brief Plücker line geometry for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_LINE_H
#define SE3KIT_LINE_H

#include <math.h>
#include <stdbool.h>
#include "vec.h"

// =============================================================================
// Constant defines -----------------------------------------------------------|
// =============================================================================

#define SE3_LINE_DIV_MIN            (1e-6f)
#define SE3_LINE_DIV_MIN_SQR        (SE3_LINE_DIV_MIN * SE3_LINE_DIV_MIN)
#define SE3_LINE_NORM_SQ_TOLERANCE  (2e-4f)
#define SE3_LINE_ORTHO_TOLERANCE    (1e-4f)

// =============================================================================
// Types ----------------------------------------------------------------------|
// =============================================================================

/**
 * @brief Directed 3D line represented by normalized Plücker coordinates.
 * - dir:    Unit direction vector (‖dir‖ == 1).
 * - moment: Moment vector about origin (p × dir, where dir ⋅ moment == 0).
 */
typedef struct {
    se3_vec3_t dir;
    se3_vec3_t moment;
} se3_line_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_line_t se3_line(se3_vec3_t dir, se3_vec3_t moment);

// Unary operations
static inline bool       se3_line_is_valid(se3_line_t line);
static inline se3_line_t se3_line_normalize(se3_line_t line);   // Reprojects such that ‖dir‖ == 1 and dir ⋅ moment == 0

// Geometric queries
static inline se3_vec3_t se3_line_project_origin(se3_line_t line);                  // Projection of the origin onto a line (faster than project_point)
static inline se3_vec3_t se3_line_project_point(se3_line_t line, se3_vec3_t p);     // Projection of a point onto a line
static inline float      se3_line_distance_to_point(se3_line_t line, se3_vec3_t p); // Distance between a line and a point
static inline float      se3_line_reciprocal_product(se3_line_t a, se3_line_t b);
static inline float      se3_line_distance_to_line(se3_line_t a, se3_line_t b);     // Distance between two lines

// Conversions
static inline se3_line_t se3_line_from_points(se3_vec3_t a, se3_vec3_t b);
static inline se3_line_t se3_line_from_point_dir(se3_vec3_t point, se3_vec3_t dir);

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_line_t se3_line(se3_vec3_t dir, se3_vec3_t moment) {
    return (se3_line_t){.dir = dir, .moment = moment};
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

/**
 * @brief Verifies that a line satisfies Plücker invariants:
 *        1. ‖dir‖ == 1
 *        2. dir ⋅ moment == 0 (Klein quadric condition)
 *
 * Scales the moment orthogonality tolerance relative to ‖moment‖ to prevent
 * float precision induced rejections when lines pass far from the coordinate origin.
 */
static inline bool se3_line_is_valid(se3_line_t line) {
    float norm_sq = se3_vec3_norm_sqr(line.dir);
    if (fabsf(norm_sq - 1.0f) > SE3_LINE_NORM_SQ_TOLERANCE) {
        return false;
    }

    float dot = fabsf(se3_vec3_dot(line.dir, line.moment));
    float m_len = se3_vec3_norm(line.moment);
    float max_allowed = SE3_LINE_ORTHO_TOLERANCE * fmaxf(1.0f, m_len);

    return dot <= max_allowed;
}

/**
 * @brief Re-projects a drifted line onto the Klein quadric:
 *        - Normalizes direction: dir' = dir / ‖dir‖
 *        - Strips parallel pitch from moment: moment' = moment - (dir' ⋅ moment) * dir'
 */
static inline se3_line_t se3_line_normalize(se3_line_t line) {
    float len_sq = se3_vec3_norm_sqr(line.dir);
    if (len_sq < SE3_LINE_DIV_MIN_SQR) {
        return se3_line(se3_vec3(1.0f, 0.0f, 0.0f), se3_vec3_zero());
    }

    float inv_len = 1.0f / sqrtf(len_sq);
    se3_vec3_t d_norm = se3_vec3_mul_scalar(line.dir, inv_len);

    float parallel_pitch = se3_vec3_dot(d_norm, line.moment);
    se3_vec3_t m_proj = se3_vec3_sub(line.moment, se3_vec3_mul_scalar(d_norm, parallel_pitch));

    return se3_line(d_norm, m_proj);
}

// =============================================================================
// Geometric query definitions ------------------------------------------------|
// =============================================================================

/**
 * @brief Returns the unique point on the line closest to the coordinate origin.
 * Evaluates to dir × moment directly (no parameter solve required).
 */
static inline se3_vec3_t se3_line_project_origin(se3_line_t line) {
    return se3_vec3_cross(line.dir, line.moment);
}

/**
 * @brief Projects an arbitrary point @p p onto the line.
 * Because project_origin(line) ⋅ dir == 0, this simplifies to:
 * p_proj = project_origin(line) + (p ⋅ dir) * dir
 */
static inline se3_vec3_t se3_line_project_point(se3_line_t line, se3_vec3_t p) {
    se3_vec3_t p0 = se3_line_project_origin(line);
    float t = se3_vec3_dot(p, line.dir);
    return se3_vec3_add(p0, se3_vec3_mul_scalar(line.dir, t));
}

/**
 * @brief Computes the perpendicular Euclidean distance from point @p p to the line.
 * Formula: ‖(p × dir) - moment‖
 */
static inline float se3_line_distance_to_point(se3_line_t line, se3_vec3_t p) {
    se3_vec3_t p_cross_d = se3_vec3_cross(p, line.dir);
    return se3_vec3_norm(se3_vec3_sub(p_cross_d, line.moment));
}

/**
 * @brief Computes the Klein reciprocal product (mutual moment) of two lines:
 *        recip(a, b) = a.dir ⋅ b.moment + b.dir ⋅ a.moment
 *
 * Evaluates to 0 if and only if lines @p a and @p b are coplanar (intersecting or parallel).
 */
static inline float se3_line_reciprocal_product(se3_line_t a, se3_line_t b) {
    return se3_vec3_dot(a.dir, b.moment) + se3_vec3_dot(b.dir, a.moment);
}

/**
 * @brief Computes the shortest perpendicular distance between two lines.
 */
static inline float se3_line_distance_to_line(se3_line_t a, se3_line_t b) {
    se3_vec3_t cross_dir = se3_vec3_cross(a.dir, b.dir);
    float cross_norm = se3_vec3_norm(cross_dir);

    if (cross_norm < SE3_LINE_DIV_MIN) {
        // Parallel lines: distance from the canonical origin point of b to line a
        return se3_line_distance_to_point(a, se3_line_project_origin(b));
    }

    // Skew lines: |reciprocal_product| / ‖a.dir × b.dir‖
    return fabsf(se3_line_reciprocal_product(a, b)) / cross_norm;
}

// =============================================================================
// Conversion definitions -----------------------------------------------------|
// =============================================================================

/**
 * @brief Constructs a line from a support point and a direction vector.
 *        Normalizes the direction and falls back to the X-axis if dir is degenerate.
 */
static inline se3_line_t se3_line_from_point_dir(se3_vec3_t point, se3_vec3_t dir) {
    float len_sq = se3_vec3_norm_sqr(dir);

    if (len_sq < SE3_LINE_DIV_MIN_SQR) {
        return se3_line(se3_vec3(1.0f, 0.0f, 0.0f), se3_vec3_zero());
    }

    float inv_len = 1.0f / sqrtf(len_sq);
    se3_vec3_t udir = se3_vec3_mul_scalar(dir, inv_len);
    se3_vec3_t moment = se3_vec3_cross(point, udir);
    return se3_line(udir, moment);
}

/**
 * @brief Constructs a line passing through point @p a toward point @p b.
 *        Refer to se3_line_from_point_dir for degenerate argument behaviour
 */
static inline se3_line_t se3_line_from_points(se3_vec3_t a, se3_vec3_t b) {
    return se3_line_from_point_dir(a, se3_vec3_sub(b, a));
}

#endif /* SE3KIT_LINE_H */
