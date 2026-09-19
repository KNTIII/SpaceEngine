#include "physics/PhysicsEngine.hpp"

#include <cassert>
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <thread>
#include <vector>

using namespace space_physics;

static void initialize(PhysicsEngine& engine) {
    for (std::size_t i = 0; i < engine.bodyCount(); ++i) {
        engine.setBody(i, {static_cast<std::int64_t>(i / 3), static_cast<std::int64_t>(i % 3), 0},
                       {1000.0 * static_cast<double>(i), 2000.0, -3000.0},
                       {0.0, 100.0 + static_cast<double>(i), 0.0}, 1.0e12 + static_cast<double>(i));
    }
    engine.computeAccelerations();
}

int main() {
    PhysicsEngine engine(2, 0.0);
    engine.setBody(0, {0, 0, 0}, {999999999.0, -1.0, 0.0}, {}, 1.0e10);
    engine.setBody(1, {1, 2, 0}, {1.0, 2.0, 3.0}, {}, 1.0e10);

    const Vector3d delta = engine.relativeVector(0, 1);
    assert(std::abs(delta.x - 2.0) < 1.0e-9);
    assert(std::abs(delta.y - 2000000003.0) < 1.0e-3);
    assert(std::abs(delta.z - 3.0) < 1.0e-9);

    engine.computeAccelerations();
    assert(engine.accelerationX()[0] > 0.0);
    assert(engine.accelerationY()[0] > 0.0);
    assert(engine.accelerationX()[1] < 0.0);

    PhysicsEngine crossing(1);
    crossing.setBody(0, {0, 0, 0}, {-1.0, PhysicsEngine::SectorSize + 2.0, 0.0}, {}, 1.0);
    crossing.computeAccelerations();
    crossing.step(0.0);
    assert(crossing.sectorX()[0] == -1);
    assert(crossing.sectorY()[0] == 1);
    assert(crossing.positionX()[0] >= 0.0 && crossing.positionX()[0] < PhysicsEngine::SectorSize);
    assert(crossing.positionY()[0] >= 0.0 && crossing.positionY()[0] < PhysicsEngine::SectorSize);

    PhysicsEngine clockEngine(1);
    clockEngine.setBody(0, {}, {}, {}, 1.0);
    clockEngine.computeAccelerations();
    FixedTimestep clock(clockEngine, 0.1, 8);
    assert(clock.advance(0.25) == 2);
    assert(std::abs(clock.accumulator() - 0.05) < 1.0e-12);

    PhysicsEngine thrust(1, 1.0, 1);
    thrust.setBody(0, {}, {}, {}, 1.0);
    thrust.computeAccelerations();
    thrust.applyThrust(0, {2.0, -3.0, 4.0});
    thrust.step(0.5);
    assert(std::abs(thrust.velocityX()[0] - 1.0) < 1.0e-12);
    assert(std::abs(thrust.velocityY()[0] + 1.5) < 1.0e-12);
    assert(std::abs(thrust.velocityZ()[0] - 2.0) < 1.0e-12);
    thrust.clearThrust(0);
    assert(std::abs(thrust.accelerationX()[0]) < 1.0e-30);

    constexpr std::size_t deterministicBodies = 16;
    const std::size_t parallelThreads = std::max<std::size_t>(2, std::thread::hardware_concurrency());
    PhysicsEngine serial(deterministicBodies, 100.0, 1);
    PhysicsEngine parallel(deterministicBodies, 100.0, parallelThreads);
    initialize(serial);
    initialize(parallel);
    for (int step = 0; step < 1000; ++step) {
        serial.step(0.01);
        parallel.step(0.01);
    }
    assert(serial.positionX() == parallel.positionX());
    assert(serial.positionY() == parallel.positionY());
    assert(serial.positionZ() == parallel.positionZ());
    assert(serial.velocityX() == parallel.velocityX());
    assert(serial.velocityY() == parallel.velocityY());
    assert(serial.velocityZ() == parallel.velocityZ());
    assert(serial.accelerationX() == parallel.accelerationX());
    assert(serial.accelerationY() == parallel.accelerationY());
    assert(serial.accelerationZ() == parallel.accelerationZ());

    std::vector<double> positions(3 * deterministicBodies);
    std::vector<double> velocities(3 * deterministicBodies);
    std::vector<std::int64_t> sectors(3 * deterministicBodies);
    serial.copyPositionsTo(positions.data(), sectors.data());
    serial.copyVelocitiesTo(velocities.data());
    assert(positions[0] == serial.positionX()[0]);
    assert(positions[1] == serial.positionY()[0]);
    assert(positions[2] == serial.positionZ()[0]);
    assert(velocities[0] == serial.velocityX()[0]);
    assert(sectors[0] == serial.sectorX()[0]);
}