#include "physics/PhysicsEngine.hpp"

#include <chrono>
#include <cmath>
#include <cstddef>
#include <iomanip>
#include <iostream>
#include <random>
#include <thread>

using namespace space_physics;

int main() {
    constexpr std::size_t bodyCount = 2000;
    constexpr int measuredSteps = 10;
    const unsigned int hardwareThreads = std::thread::hardware_concurrency();
    const std::size_t threads = hardwareThreads == 0 ? 1 : hardwareThreads;
    PhysicsEngine engine(bodyCount, 1000.0, threads);
    std::mt19937_64 generator(0xC0FFEEULL);
    std::uniform_real_distribution<double> position(-5.0e8, 5.0e8);
    std::uniform_real_distribution<double> velocity(-10.0, 10.0);

    for (std::size_t i = 0; i < bodyCount; ++i) {
        engine.setBody(i,
                       {static_cast<std::int64_t>(i % 5), static_cast<std::int64_t>((i / 5) % 5), 0},
                       {position(generator), position(generator), position(generator)},
                       {velocity(generator), velocity(generator), velocity(generator)},
                       1.0e20 + static_cast<double>(i) * 1.0e15);
    }
    engine.computeAccelerations();

    const auto start = std::chrono::steady_clock::now();
    for (int step = 0; step < measuredSteps; ++step) {
        engine.step(1.0);
    }
    const auto elapsed = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - start).count();
    const double averageMilliseconds = elapsed / static_cast<double>(measuredSteps);
    const double stepsPerSecond = 1000.0 / averageMilliseconds;

    std::cout << std::fixed << std::setprecision(3)
              << "bodies=" << bodyCount
              << " threads=" << engine.threadCount()
              << " average_step_ms=" << averageMilliseconds
              << " steps_per_second=" << stepsPerSecond << '\n';
    if (!std::isfinite(averageMilliseconds)) {
        return 1;
    }
}