#include "physics/PhysicsEngine.hpp"

#include <cmath>

namespace space_physics {

PhysicsEngine::PhysicsEngine(std::size_t bodyCount, double softeningMeters, std::size_t threadCount)
    : softeningSquared_(softeningMeters * softeningMeters),
      sectorX_(bodyCount), sectorY_(bodyCount), sectorZ_(bodyCount),
      positionX_(bodyCount), positionY_(bodyCount), positionZ_(bodyCount),
      velocityX_(bodyCount), velocityY_(bodyCount), velocityZ_(bodyCount),
      accelerationX_(bodyCount), accelerationY_(bodyCount), accelerationZ_(bodyCount),
      externalAccelX_(bodyCount), externalAccelY_(bodyCount), externalAccelZ_(bodyCount),
      masses_(bodyCount), threadCount_(threadCount == 0 ? 1 : threadCount) {
    if (threadCount_ > bodyCount && bodyCount > 0) {
        threadCount_ = bodyCount;
    }
    workers_.reserve(threadCount_ > 1 ? threadCount_ - 1 : 0);
    for (std::size_t workerIndex = 1; workerIndex < threadCount_; ++workerIndex) {
        workers_.emplace_back(&PhysicsEngine::workerLoop, this, workerIndex);
    }
}

PhysicsEngine::~PhysicsEngine() {
    {
        std::lock_guard lock(workMutex_);
        stopWorkers_ = true;
        ++workGeneration_;
    }
    workCondition_.notify_all();
    for (std::thread& worker : workers_) {
        worker.join();
    }
}

void PhysicsEngine::setBody(std::size_t index, Sector sector, Vector3d positionMeters,
                            Vector3d velocityMetersPerSecond, double massKg) {
    sectorX_[index] = sector.x;
    sectorY_[index] = sector.y;
    sectorZ_[index] = sector.z;
    positionX_[index] = positionMeters.x;
    positionY_[index] = positionMeters.y;
    positionZ_[index] = positionMeters.z;
    velocityX_[index] = velocityMetersPerSecond.x;
    velocityY_[index] = velocityMetersPerSecond.y;
    velocityZ_[index] = velocityMetersPerSecond.z;
    externalAccelX_[index] = 0.0;
    externalAccelY_[index] = 0.0;
    externalAccelZ_[index] = 0.0;
    masses_[index] = massKg;
}

void PhysicsEngine::applyThrust(std::size_t index, Vector3d acceleration) noexcept {
    accelerationX_[index] += acceleration.x - externalAccelX_[index];
    accelerationY_[index] += acceleration.y - externalAccelY_[index];
    accelerationZ_[index] += acceleration.z - externalAccelZ_[index];
    externalAccelX_[index] = acceleration.x;
    externalAccelY_[index] = acceleration.y;
    externalAccelZ_[index] = acceleration.z;
}

void PhysicsEngine::clearThrust(std::size_t index) noexcept {
    applyThrust(index, {});
}

Vector3d PhysicsEngine::relativeVector(std::size_t bodyA, std::size_t bodyB) const noexcept {
    const long double x = (static_cast<long double>(sectorX_[bodyB]) - sectorX_[bodyA]) * SectorSize
        + static_cast<long double>(positionX_[bodyB]) - positionX_[bodyA];
    const long double y = (static_cast<long double>(sectorY_[bodyB]) - sectorY_[bodyA]) * SectorSize
        + static_cast<long double>(positionY_[bodyB]) - positionY_[bodyA];
    const long double z = (static_cast<long double>(sectorZ_[bodyB]) - sectorZ_[bodyA]) * SectorSize
        + static_cast<long double>(positionZ_[bodyB]) - positionZ_[bodyA];
    return {static_cast<double>(x), static_cast<double>(y), static_cast<double>(z)};
}

void PhysicsEngine::computeAccelerationRange(std::size_t begin, std::size_t end) noexcept {
    for (std::size_t i = begin; i < end; ++i) {
        long double ax = 0.0L;
        long double ay = 0.0L;
        long double az = 0.0L;
    #if defined(_MSC_VER)
    #pragma loop(ivdep)
    #elif defined(__GNUC__) || defined(__clang__)
    #pragma GCC ivdep
    #endif
        for (std::size_t j = 0; j < bodyCount(); ++j) {
            if (i == j) {
                continue;
            }
            const long double dx = (static_cast<long double>(sectorX_[j]) - sectorX_[i]) * SectorSize
                + static_cast<long double>(positionX_[j]) - positionX_[i];
            const long double dy = (static_cast<long double>(sectorY_[j]) - sectorY_[i]) * SectorSize
                + static_cast<long double>(positionY_[j]) - positionY_[i];
            const long double dz = (static_cast<long double>(sectorZ_[j]) - sectorZ_[i]) * SectorSize
                + static_cast<long double>(positionZ_[j]) - positionZ_[i];
            const long double distanceSquared = dx * dx + dy * dy + dz * dz + softeningSquared_;
            const long double inverseDistance = 1.0L / std::sqrt(distanceSquared);
            const long double scale = static_cast<long double>(GravitationalConstant) * masses_[j]
                * inverseDistance * inverseDistance * inverseDistance;
            ax += dx * scale;
            ay += dy * scale;
            az += dz * scale;
        }
        accelerationX_[i] = static_cast<double>(ax) + externalAccelX_[i];
        accelerationY_[i] = static_cast<double>(ay) + externalAccelY_[i];
        accelerationZ_[i] = static_cast<double>(az) + externalAccelZ_[i];
    }
}

void PhysicsEngine::workerLoop(std::size_t workerIndex) noexcept {
    std::size_t observedGeneration = 0;
    for (;;) {
        {
            std::unique_lock lock(workMutex_);
            workCondition_.wait(lock, [this, &observedGeneration] {
                return stopWorkers_ || workGeneration_ != observedGeneration;
            });
            if (stopWorkers_) {
                return;
            }
            observedGeneration = workGeneration_;
        }

        const std::size_t chunk = bodyCount() / threadCount_;
        const std::size_t remainder = bodyCount() % threadCount_;
        const std::size_t begin = workerIndex * chunk + (workerIndex < remainder ? workerIndex : remainder);
        const std::size_t end = begin + chunk + (workerIndex < remainder ? 1 : 0);
        computeAccelerationRange(begin, end);

        {
            std::lock_guard lock(workMutex_);
            ++completedWorkers_;
            if (completedWorkers_ == workers_.size()) {
                completionCondition_.notify_one();
            }
        }
    }
}

void PhysicsEngine::computeAccelerations() noexcept {
    if (threadCount_ == 1) {
        computeAccelerationRange(0, bodyCount());
        return;
    }

    {
        std::lock_guard lock(workMutex_);
        completedWorkers_ = 0;
        ++workGeneration_;
    }
    workCondition_.notify_all();

    const std::size_t chunk = bodyCount() / threadCount_;
    const std::size_t remainder = bodyCount() % threadCount_;
    const std::size_t mainEnd = chunk + (remainder > 0 ? 1 : 0);
    computeAccelerationRange(0, mainEnd);

    std::unique_lock lock(workMutex_);
    completionCondition_.wait(lock, [this] { return completedWorkers_ == workers_.size(); });
}

void PhysicsEngine::normalizePositions() noexcept {
    for (std::size_t i = 0; i < bodyCount(); ++i) {
        const auto shiftX = static_cast<std::int64_t>(std::floor(positionX_[i] / SectorSize));
        const auto shiftY = static_cast<std::int64_t>(std::floor(positionY_[i] / SectorSize));
        const auto shiftZ = static_cast<std::int64_t>(std::floor(positionZ_[i] / SectorSize));
        sectorX_[i] += shiftX;
        sectorY_[i] += shiftY;
        sectorZ_[i] += shiftZ;
        positionX_[i] -= static_cast<double>(shiftX) * SectorSize;
        positionY_[i] -= static_cast<double>(shiftY) * SectorSize;
        positionZ_[i] -= static_cast<double>(shiftZ) * SectorSize;
    }
}

void PhysicsEngine::step(double dtSeconds) noexcept {
    const double halfDt = 0.5 * dtSeconds;
    for (std::size_t i = 0; i < bodyCount(); ++i) {
        velocityX_[i] += accelerationX_[i] * halfDt;
        velocityY_[i] += accelerationY_[i] * halfDt;
        velocityZ_[i] += accelerationZ_[i] * halfDt;
        positionX_[i] += velocityX_[i] * dtSeconds;
        positionY_[i] += velocityY_[i] * dtSeconds;
        positionZ_[i] += velocityZ_[i] * dtSeconds;
    }
    normalizePositions();
    computeAccelerations();
    for (std::size_t i = 0; i < bodyCount(); ++i) {
        velocityX_[i] += accelerationX_[i] * halfDt;
        velocityY_[i] += accelerationY_[i] * halfDt;
        velocityZ_[i] += accelerationZ_[i] * halfDt;
    }
}

void PhysicsEngine::copyPositionsTo(double* outLocalPositions, std::int64_t* outSectors) const noexcept {
    for (std::size_t i = 0; i < bodyCount(); ++i) {
        outLocalPositions[3 * i] = positionX_[i];
        outLocalPositions[3 * i + 1] = positionY_[i];
        outLocalPositions[3 * i + 2] = positionZ_[i];
        outSectors[3 * i] = sectorX_[i];
        outSectors[3 * i + 1] = sectorY_[i];
        outSectors[3 * i + 2] = sectorZ_[i];
    }
}

void PhysicsEngine::copyVelocitiesTo(double* outVelocities) const noexcept {
    for (std::size_t i = 0; i < bodyCount(); ++i) {
        outVelocities[3 * i] = velocityX_[i];
        outVelocities[3 * i + 1] = velocityY_[i];
        outVelocities[3 * i + 2] = velocityZ_[i];
    }
}

FixedTimestep::FixedTimestep(PhysicsEngine& engine, double fixedDtSeconds,
                             std::size_t maxStepsPerAdvance) noexcept
    : engine_(engine), fixedDtSeconds_(fixedDtSeconds), maxStepsPerAdvance_(maxStepsPerAdvance) {}

std::size_t FixedTimestep::advance(double elapsedSeconds) noexcept {
    accumulatorSeconds_ += elapsedSeconds;
    std::size_t steps = 0;
    while (accumulatorSeconds_ >= fixedDtSeconds_ && steps < maxStepsPerAdvance_) {
        engine_.step(fixedDtSeconds_);
        accumulatorSeconds_ -= fixedDtSeconds_;
        ++steps;
    }
    if (steps == maxStepsPerAdvance_ && accumulatorSeconds_ >= fixedDtSeconds_) {
        accumulatorSeconds_ = std::fmod(accumulatorSeconds_, fixedDtSeconds_);
    }
    return steps;
}

} // namespace space_physics