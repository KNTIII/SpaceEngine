#include "physics/PhysicsEngine.hpp"

#include <iomanip>
#include <iostream>

using space_physics::FixedTimestep;
using space_physics::PhysicsEngine;

int main() {
    constexpr double sunMass = 1.98847e30;
    constexpr double earthMass = 5.9722e24;
    constexpr double moonMass = 7.342e22;
    constexpr double craftMass = 1.0e3;
    constexpr double earthOrbitRadius = 1.495978707e11;
    constexpr double moonOrbitRadius = 3.844e8;
    constexpr double secondsPerHour = 3600.0;
    const double earthLocalX = earthOrbitRadius - 149.0 * PhysicsEngine::SectorSize;

    PhysicsEngine engine(4, 1.0e3);
    engine.setBody(0, {0, 0, 0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, sunMass);
    engine.setBody(1, {149, 0, 0}, {earthLocalX, 0.0, 0.0}, {0.0, 29780.0, 0.0}, earthMass);
    engine.setBody(2, {149, 0, 0}, {earthLocalX + moonOrbitRadius, 0.0, 0.0},
                   {0.0, 29780.0 + 1022.0, 0.0}, moonMass);
    engine.setBody(3, {149, 0, 0}, {earthLocalX + 7.0e6, 0.0, 0.0},
                   {0.0, 29780.0 + 7700.0, 0.0}, craftMass);

    engine.computeAccelerations();
    FixedTimestep clock(engine, secondsPerHour);
    for (int step = 0; step < 1000; ++step) {
        clock.advance(secondsPerHour);
    }

    const char* names[] = {"Sun", "Earth", "Moon", "Spacecraft"};
    std::cout << std::setprecision(17);
    std::cout << "Completed 1000 fixed physics steps\n";
    for (std::size_t i = 0; i < engine.bodyCount(); ++i) {
        std::cout << names[i] << ": sector=(" << engine.sectorX()[i] << ", "
                  << engine.sectorY()[i] << ", " << engine.sectorZ()[i] << ")"
                  << " local_m=(" << engine.positionX()[i] << ", "
                  << engine.positionY()[i] << ", " << engine.positionZ()[i] << ")\n";
    }
}