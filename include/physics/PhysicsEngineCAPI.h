#pragma once

#include <stdint.h>

#if defined(_WIN32) && defined(SPACE_PHYSICS_BUILD_SHARED)
#if defined(space_physics_EXPORTS)
#define SPACE_PHYSICS_API __declspec(dllexport)
#else
#define SPACE_PHYSICS_API __declspec(dllimport)
#endif
#else
#define SPACE_PHYSICS_API
#endif

#ifdef __cplusplus
extern "C" {
#endif

typedef void* space_physics_handle;

typedef struct space_physics_config {
    uint64_t body_count;
    double softening_meters;
    uint64_t thread_count;
} space_physics_config;

typedef struct space_physics_sector {
    int64_t x;
    int64_t y;
    int64_t z;
} space_physics_sector;

typedef struct space_physics_vector3d {
    double x;
    double y;
    double z;
} space_physics_vector3d;

typedef enum space_physics_result {
    SPACE_PHYSICS_OK = 0,
    SPACE_PHYSICS_INVALID_ARGUMENT = 1,
    SPACE_PHYSICS_OUT_OF_RANGE = 2,
    SPACE_PHYSICS_ALLOCATION_FAILURE = 3,
    SPACE_PHYSICS_INTERNAL_ERROR = 4
} space_physics_result;

SPACE_PHYSICS_API space_physics_handle space_physics_create(space_physics_config config);
SPACE_PHYSICS_API void space_physics_destroy(space_physics_handle handle);
SPACE_PHYSICS_API uint64_t space_physics_body_count(space_physics_handle handle);
SPACE_PHYSICS_API space_physics_result space_physics_set_body(
    space_physics_handle handle, uint64_t index, space_physics_sector sector,
    space_physics_vector3d position_meters, space_physics_vector3d velocity_meters_per_second,
    double mass_kg);
SPACE_PHYSICS_API space_physics_result space_physics_apply_thrust(
    space_physics_handle handle, uint64_t index, space_physics_vector3d acceleration_meters_per_second_squared);
SPACE_PHYSICS_API space_physics_result space_physics_clear_thrust(space_physics_handle handle, uint64_t index);
SPACE_PHYSICS_API space_physics_result space_physics_compute_accelerations(space_physics_handle handle);
SPACE_PHYSICS_API space_physics_result space_physics_step(space_physics_handle handle, double dt_seconds);
SPACE_PHYSICS_API space_physics_result space_physics_copy_positions(
    space_physics_handle handle, double* out_local_positions, int64_t* out_sectors, uint64_t element_count);
SPACE_PHYSICS_API space_physics_result space_physics_copy_velocities(
    space_physics_handle handle, double* out_velocities, uint64_t element_count);

#ifdef __cplusplus
}
#endif