#include "physics/PhysicsEngineCAPI.h"

#include "physics/PhysicsEngine.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <new>

namespace {

space_physics::PhysicsEngine* engineFromHandle(space_physics_handle handle) noexcept {
    return static_cast<space_physics::PhysicsEngine*>(handle);
}

bool validFinite(double value) noexcept {
    return std::isfinite(value);
}

bool validVector(space_physics_vector3d value) noexcept {
    return validFinite(value.x) && validFinite(value.y) && validFinite(value.z);
}

space_physics_result validBody(space_physics_handle handle, uint64_t index) noexcept {
    const auto* engine = engineFromHandle(handle);
    if (engine == nullptr) {
        return SPACE_PHYSICS_INVALID_ARGUMENT;
    }
    if (index >= engine->bodyCount() || index > std::numeric_limits<std::size_t>::max()) {
        return SPACE_PHYSICS_OUT_OF_RANGE;
    }
    return SPACE_PHYSICS_OK;
}

} // namespace

space_physics_handle space_physics_create(space_physics_config config) {
    if (config.body_count > std::numeric_limits<std::size_t>::max()
        || !validFinite(config.softening_meters) || config.softening_meters < 0.0
        || config.thread_count > std::numeric_limits<std::size_t>::max()) {
        return nullptr;
    }
    try {
        return new space_physics::PhysicsEngine(
            static_cast<std::size_t>(config.body_count), config.softening_meters,
            static_cast<std::size_t>(config.thread_count));
    } catch (const std::bad_alloc&) {
        return nullptr;
    } catch (...) {
        return nullptr;
    }
}

void space_physics_destroy(space_physics_handle handle) {
    delete engineFromHandle(handle);
}

uint64_t space_physics_body_count(space_physics_handle handle) {
    const auto* engine = engineFromHandle(handle);
    return engine == nullptr ? 0 : static_cast<uint64_t>(engine->bodyCount());
}

space_physics_result space_physics_set_body(
    space_physics_handle handle, uint64_t index, space_physics_sector sector,
    space_physics_vector3d position, space_physics_vector3d velocity, double mass) {
    const space_physics_result result = validBody(handle, index);
    if (result != SPACE_PHYSICS_OK || !validVector(position) || !validVector(velocity)
        || !validFinite(mass) || mass < 0.0) {
        return result == SPACE_PHYSICS_OK ? SPACE_PHYSICS_INVALID_ARGUMENT : result;
    }
    engineFromHandle(handle)->setBody(
        static_cast<std::size_t>(index), {sector.x, sector.y, sector.z},
        {position.x, position.y, position.z}, {velocity.x, velocity.y, velocity.z}, mass);
    return SPACE_PHYSICS_OK;
}

space_physics_result space_physics_apply_thrust(
    space_physics_handle handle, uint64_t index, space_physics_vector3d acceleration) {
    const space_physics_result result = validBody(handle, index);
    if (result != SPACE_PHYSICS_OK) {
        return result;
    }
    if (!validVector(acceleration)) {
        return SPACE_PHYSICS_INVALID_ARGUMENT;
    }
    engineFromHandle(handle)->applyThrust(
        static_cast<std::size_t>(index), {acceleration.x, acceleration.y, acceleration.z});
    return SPACE_PHYSICS_OK;
}

space_physics_result space_physics_clear_thrust(space_physics_handle handle, uint64_t index) {
    const space_physics_result result = validBody(handle, index);
    if (result != SPACE_PHYSICS_OK) {
        return result;
    }
    engineFromHandle(handle)->clearThrust(static_cast<std::size_t>(index));
    return SPACE_PHYSICS_OK;
}

space_physics_result space_physics_compute_accelerations(space_physics_handle handle) {
    if (engineFromHandle(handle) == nullptr) {
        return SPACE_PHYSICS_INVALID_ARGUMENT;
    }
    engineFromHandle(handle)->computeAccelerations();
    return SPACE_PHYSICS_OK;
}

space_physics_result space_physics_step(space_physics_handle handle, double dt_seconds) {
    if (engineFromHandle(handle) == nullptr || !validFinite(dt_seconds) || dt_seconds < 0.0) {
        return SPACE_PHYSICS_INVALID_ARGUMENT;
    }
    engineFromHandle(handle)->step(dt_seconds);
    return SPACE_PHYSICS_OK;
}

space_physics_result space_physics_copy_positions(
    space_physics_handle handle, double* out_local_positions, int64_t* out_sectors, uint64_t element_count) {
    const auto* engine = engineFromHandle(handle);
    if (engine == nullptr || out_local_positions == nullptr || out_sectors == nullptr) {
        return SPACE_PHYSICS_INVALID_ARGUMENT;
    }
    if (element_count < engine->bodyCount() || element_count > std::numeric_limits<std::size_t>::max()) {
        return SPACE_PHYSICS_OUT_OF_RANGE;
    }
    engine->copyPositionsTo(out_local_positions, out_sectors);
    return SPACE_PHYSICS_OK;
}

space_physics_result space_physics_copy_velocities(
    space_physics_handle handle, double* out_velocities, uint64_t element_count) {
    const auto* engine = engineFromHandle(handle);
    if (engine == nullptr || out_velocities == nullptr) {
        return SPACE_PHYSICS_INVALID_ARGUMENT;
    }
    if (element_count < engine->bodyCount() || element_count > std::numeric_limits<std::size_t>::max()) {
        return SPACE_PHYSICS_OUT_OF_RANGE;
    }
    engine->copyVelocitiesTo(out_velocities);
    return SPACE_PHYSICS_OK;
}