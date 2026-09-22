/**
 * @file batch.h
 * @brief Batch processing shorthands for arrays of geometry primitives.
 * Part of se3kit.
 */

#ifndef SE3KIT_BATCH_H
#define SE3KIT_BATCH_H

#include <stddef.h>
#include "dual_quat.h"
#include "quat.h"
#include "vec.h"

// =============================================================================
// Compiler Compatibility -----------------------------------------------------|
// =============================================================================

#ifndef SE3_RESTRICT
    #if defined(__cplusplus) || defined(_MSC_VER)
        #define SE3_RESTRICT __restrict
    #else
        #define SE3_RESTRICT restrict
    #endif
#endif

// =============================================================================
// API Declarations -----------------------------------------------------------|
// =============================================================================

// TODO: check, dedupe, extend, implement

// Vector array transforms
static inline void se3_batch_transform_vec3(se3_dual_quat_t dq, const se3_vec3_t *SE3_RESTRICT in_pts, se3_vec3_t *SE3_RESTRICT out_pts, size_t count);
static inline void se3_batch_rotate_vec3(se3_quat_t q, const se3_vec3_t *SE3_RESTRICT in_pts, se3_vec3_t *SE3_RESTRICT out_pts, size_t count);
static inline void se3_batch_transform_line(se3_dual_quat_t dq, const se3_line_t *SE3_RESTRICT in_lines, se3_line_t *SE3_RESTRICT out_lines, size_t count);

static inline void se3_batch_transform_vec3_in_place(se3_dual_quat_t dq, se3_vec3_t *pts, size_t count);
static inline void se3_batch_rotate_vec3_in_place(se3_quat_t q, se3_vec3_t *pts, size_t count);
static inline void se3_batch_transform_line_in_place(se3_dual_quat_t dq, se3_line_t *lines, size_t count);

// =============================================================================
// Implementation -------------------------------------------------------------|
// =============================================================================

static inline void se3_batch_transform_vec3(
    se3_dual_quat_t dq, 
    const se3_vec3_t *SE3_RESTRICT in_pts, 
    se3_vec3_t *SE3_RESTRICT out_pts, 
    size_t count
) {
    for (size_t i = 0; i < count; ++i) {
        out_pts[i] = se3_dual_quat_transform_vec3(dq, in_pts[i]);
    }
}

static inline void se3_batch_rotate_vec3(
    se3_quat_t q, 
    const se3_vec3_t *SE3_RESTRICT in_pts, 
    se3_vec3_t *SE3_RESTRICT out_pts, 
    size_t count
) {
    for (size_t i = 0; i < count; ++i) {
        out_pts[i] = se3_quat_rotate_vec3(q, in_pts[i]);
    }
}

static inline void se3_batch_transform_vec3_in_place(se3_dual_quat_t dq, se3_vec3_t *pts, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        pts[i] = se3_dual_quat_transform_vec3(dq, pts[i]);
    }
}

static inline void se3_batch_rotate_vec3_in_place(se3_quat_t q, se3_vec3_t *pts, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        pts[i] = se3_quat_rotate_vec3(q, pts[i]);
    }
}

static inline void se3_batch_transform_line(
    se3_dual_quat_t dq, 
    const se3_line_t *SE3_RESTRICT in_lines, 
    se3_line_t *SE3_RESTRICT out_lines, 
    size_t count
) {
    for (size_t i = 0; i < count; ++i) {
        out_lines[i] = se3_dual_quat_transform_line(dq, in_lines[i]);
    }
}

static inline void se3_batch_transform_line_in_place(se3_dual_quat_t dq, se3_line_t *lines, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        lines[i] = se3_dual_quat_transform_line(dq, lines[i]);
    }
}

#endif /* SE3KIT_BATCH_H */
