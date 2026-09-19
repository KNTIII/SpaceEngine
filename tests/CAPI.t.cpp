#include "physics/PhysicsEngineCAPI.h"

#include <cassert>
#include <cmath>
#include <cstdint>

int main() {
    const space_physics_config config{2, 1.0, 2};
    const space_physics_handle handle = space_physics_create(config);
    assert(handle != nullptr);
    assert(space_physics_body_count(handle) == 2);

    assert(space_physics_set_body(handle, 0, {0, 0, 0}, {0.0, 0.0, 0.0}, {}, 1.0e10)
           == SPACE_PHYSICS_OK);
    assert(space_physics_set_body(handle, 1, {0, 0, 0}, {10.0, 0.0, 0.0}, {}, 1.0)
           == SPACE_PHYSICS_OK);
    assert(space_physics_apply_thrust(handle, 1, {2.0, 0.0, 0.0}) == SPACE_PHYSICS_OK);
    assert(space_physics_compute_accelerations(handle) == SPACE_PHYSICS_OK);
    assert(space_physics_step(handle, 0.5) == SPACE_PHYSICS_OK);

    double positions[6]{};
    int64_t sectors[6]{};
    double velocities[6]{};
    assert(space_physics_copy_positions(handle, positions, sectors, 2) == SPACE_PHYSICS_OK);
    assert(space_physics_copy_velocities(handle, velocities, 2) == SPACE_PHYSICS_OK);
    assert(std::isfinite(positions[0]));
    assert(std::isfinite(velocities[3]));
    assert(space_physics_copy_positions(handle, positions, sectors, 1) == SPACE_PHYSICS_OUT_OF_RANGE);
    assert(space_physics_step(handle, -1.0) == SPACE_PHYSICS_INVALID_ARGUMENT);
    assert(space_physics_set_body(handle, 2, {}, {}, {}, 1.0) == SPACE_PHYSICS_OUT_OF_RANGE);
    space_physics_destroy(handle);
}