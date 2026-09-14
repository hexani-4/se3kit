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

#endif /* SE3KIT_BATCH_H */
