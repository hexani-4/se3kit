/**
 * @file cone.h
 * @brief Cone geometry for embedded C11.
 * Part of se3kit.
 */

#ifndef SE3KIT_CONE_H
#define SE3KIT_CONE_H

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
 *        an apex point, and half-angle aperture.
 */
typedef struct {
    se3_line_t axis;            // Central symmetry axis
    float      apex_coord;      // Apex coordinate (on the axis)
    float      cos_half_angle;  // Aperture (as cosine of the half-angle)
} se3_cone_t;

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// Initializers
static inline se3_cone_t se3_cone(se3_vec3_t apex, se3_vec3_t axis_dir, float half_angle_rad);

// Unary operations
static inline bool se3_cone_is_valid(se3_cone_t c);
static inline se3_cone_t se3_cone_normalize(se3_cone_t c);

// Geometric queries
static inline bool se3_cone_contains_point(se3_cone_t c, se3_vec3_t p);     // Point lies within aperture
static inline bool se3_cone_contains_line(se3_cone_t c, se3_line_t l);      // Line passes through the apex and lies within the aperture
static inline bool se3_cone_intercepts_line(se3_cone_t c, se3_line_t l);    // Line defines points within aperture

// Assisting geometry queries
static inline se3_line_t se3_cone_axis_line(se3_cone_t c);              // Central axis line
static inline se3_vec3_t se3_cone_apex(se3_cone_t c);                   // Apex point
static inline float se3_cone_apex_distance(se3_cone_t c, se3_vec3_t p); // Distance of the apex point to p
static inline float se3_cone_half_angle(se3_cone_t c);                  // Aperture half angle
static inline se3_line_t se3_cone_generator_line(se3_cone_t c, se3_vec3_t tangent_hint, float phi);

// Rotational alignment queries
static inline se3_quat_t se3_cone_rotation_to_contain_point(se3_cone_t c, se3_vec3_t p, float aperture_slop);   // Minimal rotation around apex to contain point, ensuring at most an aperture_slop angle to axis line

// Conversions & Deconstructors
static inline se3_cone_t se3_cone_from_line(se3_line_t axis_line, se3_vec3_t apex, float half_angle);
static inline se3_cone_t se3_cone_from_friction(se3_vec3_t contact_point, se3_vec3_t normal, float mu);

// =============================================================================
// Initializer definitions ----------------------------------------------------|
// =============================================================================

static inline se3_cone_t se3_cone(se3_vec3_t apex, se3_vec3_t axis_dir, float half_angle_rad) {
    se3_line_t axis = se3_line_from_point_dir(apex, axis_dir);
    se3_vec3_t p0 = se3_line_project_origin(axis);
    
    return (se3_cone_t){
        .axis = axis,
        .apex_coord = se3_vec3_dot(se3_vec3_sub(apex, p0), axis.dir),
        .cos_half_angle = cosf(half_angle_rad)
    };
}

// =============================================================================
// Unary operation definitions ------------------------------------------------|
// =============================================================================

static inline bool se3_cone_is_valid(se3_cone_t c) {
    return se3_line_is_valid(c.axis) && 
           c.cos_half_angle >= -1.0f && 
           c.cos_half_angle <= 1.0f && 
           !isnan(c.apex_coord);
}

static inline se3_cone_t se3_cone_normalize(se3_cone_t c) {
    return (se3_cone_t){
        .axis = se3_line_normalize(c.axis),
        .apex_coord = c.apex_coord,
        .cos_half_angle = fmaxf(-1.0f, fminf(1.0f, c.cos_half_angle))
    };
}

// =============================================================================
// Geometric query definitions ------------------------------------------------|
// =============================================================================

static inline bool se3_cone_contains_point(se3_cone_t c, se3_vec3_t p) {
    se3_vec3_t v = se3_vec3_sub(p, se3_cone_apex(c));
    float len_sq = se3_vec3_norm_sqr(v);
    
    if (len_sq < SE3_CONE_DIV_MIN_SQR) {
        return true; // Apex is trivially inside
    }

    float z = se3_vec3_dot(v, c.axis.dir);
    float c_ha = c.cos_half_angle;

    // Acute vs Obtuse cone geometric branching
    if (c_ha >= 0.0f) {
        if (z <= 0.0f) return false;
        return (z * z) >= (c_ha * c_ha * len_sq);
    } else {
        if (z >= 0.0f) return true;
        return (z * z) <= (c_ha * c_ha * len_sq);
    }
}

static inline bool se3_cone_contains_line(se3_cone_t c, se3_line_t l) {
    // A bundle line of action must originate precisely from the apex
    if (se3_line_distance_to_point(l, se3_cone_apex(c)) > SE3_CONE_DISTANCE_TOLERANCE) {
        return false;
    }
    
    // Evaluate if the direction falls within the aperture (||l.dir|| == 1)
    float z = se3_vec3_dot(l.dir, c.axis.dir);
    float c_ha = c.cos_half_angle;

    if (c_ha >= 0.0f) {
        if (z <= 0.0f) return false;
        return (z * z) >= (c_ha * c_ha); 
    } else {
        if (z >= 0.0f) return true;
        return (z * z) <= (c_ha * c_ha);
    }
}

/**
 * @brief Evaluates projective intersection between the cone's quadratic complex 
 *        and the given line using an exact, branch-optimized algebraic solver.
 */
static inline bool se3_cone_intercepts_line(se3_cone_t c, se3_line_t l) {
    se3_vec3_t apex = se3_cone_apex(c);
    se3_vec3_t w = se3_vec3_sub(se3_line_project_point(l, apex), apex);
    float W = se3_vec3_norm_sqr(w);

    if (W < SE3_CONE_DIV_MIN_SQR) {
        return true; // Line pierces the apex exactly
    }

    float c_ha = c.cos_half_angle;
    float c2 = c_ha * c_ha;
    float Du = se3_vec3_dot(l.dir, c.axis.dir);
    float Dw = se3_vec3_dot(w, c.axis.dir);

    float A = Du * Du - c2;
    
    // If the line is steeper than the cone aperture, an infinite line 
    // will always eventually enter the bounding cone volume.
    if (A > 0.0f) {
        return true; 
    }

    // Discriminant delta condition for intersection with the double-cone:
    // delta = c2 * (W * Du^2 + Dw^2 - c2 * W) >= 0
    float delta_scaled = W * Du * Du + Dw * Dw - c2 * W;
    if (delta_scaled < 0.0f) {
        return false; // Misses the mathematical double-cone entirely
    }

    // It hits the double cone. Find the parameter t of maximum penetration 
    // (A < 0) or the single intersection point (A == 0) to evaluate containment.
    float t_eval;
    if (A < -SE3_CONE_DIV_MIN_SQR) {
        t_eval = -(Dw * Du) / A;
    } else {
        float B = Dw * Du;
        if (fabsf(B) < SE3_CONE_DIV_MIN) {
            // Line is perfectly parallel to the boundary edge
            return Dw > 0.0f || c_ha < 0.0f;
        }
        float C = Dw * Dw - c2 * W;
        t_eval = -C / (2.0f * B);
    }

    // Reconstruct the point of deepest intersection and verify it lies on the correct half
    se3_vec3_t p_eval = se3_vec3_add(se3_vec3_add(apex, w), se3_vec3_mul_scalar(l.dir, t_eval));
    return se3_cone_contains_point(c, p_eval);
}

// =============================================================================
// Assisting geometry query definitions ---------------------------------------|
// =============================================================================

static inline se3_line_t se3_cone_axis_line(se3_cone_t c) {
    return c.axis;
}

static inline se3_vec3_t se3_cone_apex(se3_cone_t c) {
    se3_vec3_t p0 = se3_line_project_origin(c.axis);
    return se3_vec3_add(p0, se3_vec3_mul_scalar(c.axis.dir, c.apex_coord));
}

static inline float se3_cone_apex_distance(se3_cone_t c, se3_vec3_t p) {
    return se3_vec3_norm(se3_vec3_sub(p, se3_cone_apex(c)));
}

static inline float se3_cone_half_angle(se3_cone_t c) {
    return acosf(fmaxf(-1.0f, fminf(1.0f, c.cos_half_angle)));
}

/**
 * @brief Evaluates an exterior generator line at azimuth @p phi (0 to 2PI)
 *        to construct discrete polyhedral envelopes (e.g. friction pyramids).
 */
static inline se3_line_t se3_cone_generator_line(se3_cone_t c, se3_vec3_t tangent_hint, float phi) {
    se3_vec3_t d = c.axis.dir;
    
    // Project the hint to the plane orthogonal to the axis
    se3_vec3_t t_proj = se3_vec3_sub(tangent_hint, se3_vec3_mul_scalar(d, se3_vec3_dot(tangent_hint, d)));
    float t_len_sq = se3_vec3_norm_sqr(t_proj);

    se3_vec3_t u;
    if (t_len_sq < SE3_CONE_DIV_MIN_SQR) {
        // Fallback robust orthogonal basis if the hint was collinear
        u = fabsf(d.x) < 0.9f ? se3_vec3(1.0f, 0.0f, 0.0f) : se3_vec3(0.0f, 1.0f, 0.0f);
        u = se3_vec3_normalize(se3_vec3_sub(u, se3_vec3_mul_scalar(d, se3_vec3_dot(u, d))));
    } else {
        u = se3_vec3_mul_scalar(t_proj, 1.0f / sqrtf(t_len_sq));
    }

    se3_vec3_t v = se3_vec3_cross(d, u);
    se3_vec3_t radial = se3_vec3_add(se3_vec3_mul_scalar(u, cosf(phi)), se3_vec3_mul_scalar(v, sinf(phi)));
    float sin_ha = sqrtf(fmaxf(0.0f, 1.0f - c.cos_half_angle * c.cos_half_angle));
    
    se3_vec3_t gen_dir = se3_vec3_add(se3_vec3_mul_scalar(d, c.cos_half_angle), se3_vec3_mul_scalar(radial, sin_ha));
    return se3_line_from_point_dir(se3_cone_apex(c), gen_dir);
}

// =============================================================================
// Rotational alignment definitions -------------------------------------------|
// =============================================================================

static inline se3_quat_t se3_cone_rotation_to_contain_point(se3_cone_t c, se3_vec3_t p, float aperture_slop) {
    se3_vec3_t v = se3_vec3_sub(p, se3_cone_apex(c));
    float len_sq = se3_vec3_norm_sqr(v);

    if (len_sq < SE3_CONE_DIV_MIN_SQR) {
        return se3_quat_identity();
    }

    se3_vec3_t u_v = se3_vec3_mul_scalar(v, 1.0f / sqrtf(len_sq));
    float current_cos = se3_vec3_dot(c.axis.dir, u_v);
    float target_cos = cosf(fmaxf(0.0f, aperture_slop));

    // Target is already deeper inside the cone than the requested slop
    if (current_cos >= target_cos) {
        return se3_quat_identity();
    }

    se3_vec3_t rot_axis = se3_vec3_cross(c.axis.dir, u_v);
    float rot_len_sq = se3_vec3_norm_sqr(rot_axis);

    if (rot_len_sq < SE3_CONE_DIV_MIN_SQR) {
        // Antiparallel fallback
        rot_axis = se3_vec3_orthogonal(c.axis.dir);
    } else {
        rot_axis = se3_vec3_mul_scalar(rot_axis, 1.0f / sqrtf(rot_len_sq));
    }

    float current_angle = acosf(fmaxf(-1.0f, fminf(1.0f, current_cos)));
    float target_angle = acosf(fmaxf(-1.0f, fminf(1.0f, target_cos)));
    
    se3_axis_angle_t aa = se3_axis_angle(rot_axis, current_angle - target_angle);
    return se3_quat_from_axis_angle(aa);
}

// =============================================================================
// Conversions & Deconstructors -----------------------------------------------|
// =============================================================================

static inline se3_cone_t se3_cone_from_line(se3_line_t axis_line, se3_vec3_t apex, float half_angle) {
    se3_vec3_t p0 = se3_line_project_origin(axis_line);
    return (se3_cone_t){
        .axis = axis_line,
        .apex_coord = se3_vec3_dot(se3_vec3_sub(apex, p0), axis_line.dir),
        .cos_half_angle = cosf(half_angle)
    };
}

/**
 * @brief Constructs an unbounded Coulomb friction cone bounding valid lines of action.
 *        cos(alpha) = 1 / sqrt(1 + mu^2)
 */
static inline se3_cone_t se3_cone_from_friction(se3_vec3_t contact_point, se3_vec3_t normal, float mu) {
    se3_line_t axis = se3_line_from_point_dir(contact_point, normal);
    se3_vec3_t p0 = se3_line_project_origin(axis);
    
    return (se3_cone_t){
        .axis = axis,
        .apex_coord = se3_vec3_dot(se3_vec3_sub(contact_point, p0), axis.dir),
        .cos_half_angle = 1.0f / sqrtf(1.0f + mu * mu) 
    };
}

#endif /* SE3KIT_CONE_H */
