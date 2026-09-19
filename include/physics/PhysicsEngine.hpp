#pragma once

#include <cstddef>
#include <cstdint>
#include <condition_variable>
#include <mutex>
#include <thread>
#include <vector>

namespace space_physics {

struct Sector {
    std::int64_t x{};
    std::int64_t y{};
    std::int64_t z{};
};

struct Vector3d {
    double x{};
    double y{};
    double z{};
};

class PhysicsEngine {
public:
    static constexpr double SectorSize = 1.0e9;
    static constexpr double GravitationalConstant = 6.67430e-11;

    explicit PhysicsEngine(std::size_t bodyCount, double softeningMeters = 1.0,
                           std::size_t threadCount = 1);
    ~PhysicsEngine();

    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;

    std::size_t bodyCount() const noexcept { return masses_.size(); }
    void setBody(std::size_t index, Sector sector, Vector3d positionMeters,
                 Vector3d velocityMetersPerSecond, double massKg);
    void applyThrust(std::size_t index, Vector3d accelerationMetersPerSecondSquared) noexcept;
    void clearThrust(std::size_t index) noexcept;
    Vector3d relativeVector(std::size_t bodyA, std::size_t bodyB) const noexcept;
    void computeAccelerations() noexcept;
    void step(double dtSeconds) noexcept;

    std::size_t threadCount() const noexcept { return threadCount_; }

    // Layout: three consecutive values per body: x, y, z.
    void copyPositionsTo(double* outLocalPositions, std::int64_t* outSectors) const noexcept;
    void copyVelocitiesTo(double* outVelocities) const noexcept;

    const std::vector<std::int64_t>& sectorX() const noexcept { return sectorX_; }
    const std::vector<std::int64_t>& sectorY() const noexcept { return sectorY_; }
    const std::vector<std::int64_t>& sectorZ() const noexcept { return sectorZ_; }
    const std::vector<double>& positionX() const noexcept { return positionX_; }
    const std::vector<double>& positionY() const noexcept { return positionY_; }
    const std::vector<double>& positionZ() const noexcept { return positionZ_; }
    const std::vector<double>& velocityX() const noexcept { return velocityX_; }
    const std::vector<double>& velocityY() const noexcept { return velocityY_; }
    const std::vector<double>& velocityZ() const noexcept { return velocityZ_; }
    const std::vector<double>& accelerationX() const noexcept { return accelerationX_; }
    const std::vector<double>& accelerationY() const noexcept { return accelerationY_; }
    const std::vector<double>& accelerationZ() const noexcept { return accelerationZ_; }
    const std::vector<double>& externalAccelerationX() const noexcept { return externalAccelX_; }
    const std::vector<double>& externalAccelerationY() const noexcept { return externalAccelY_; }
    const std::vector<double>& externalAccelerationZ() const noexcept { return externalAccelZ_; }
    const std::vector<double>& masses() const noexcept { return masses_; }

private:
    void workerLoop(std::size_t workerIndex) noexcept;
    void computeAccelerationRange(std::size_t begin, std::size_t end) noexcept;
    void normalizePositions() noexcept;

    double softeningSquared_;
    std::vector<std::int64_t> sectorX_;
    std::vector<std::int64_t> sectorY_;
    std::vector<std::int64_t> sectorZ_;
    std::vector<double> positionX_;
    std::vector<double> positionY_;
    std::vector<double> positionZ_;
    std::vector<double> velocityX_;
    std::vector<double> velocityY_;
    std::vector<double> velocityZ_;
    std::vector<double> accelerationX_;
    std::vector<double> accelerationY_;
    std::vector<double> accelerationZ_;
    std::vector<double> externalAccelX_;
    std::vector<double> externalAccelY_;
    std::vector<double> externalAccelZ_;
    std::vector<double> masses_;

    std::size_t threadCount_;
    std::vector<std::thread> workers_;
    std::mutex workMutex_;
    std::condition_variable workCondition_;
    std::condition_variable completionCondition_;
    bool stopWorkers_{};
    std::size_t workGeneration_{};
    std::size_t completedWorkers_{};
};

class FixedTimestep {
public:
    FixedTimestep(PhysicsEngine& engine, double fixedDtSeconds,
                  std::size_t maxStepsPerAdvance = 8) noexcept;
    std::size_t advance(double elapsedSeconds) noexcept;
    double accumulator() const noexcept { return accumulatorSeconds_; }

private:
    PhysicsEngine& engine_;
    double fixedDtSeconds_;
    double accumulatorSeconds_{};
    std::size_t maxStepsPerAdvance_;
};

} // namespace space_physics