#include <cmath>
#include <iostream>
#include <vector>

#include "chrono_dem/physics/ChSystemDem.h"

int main() {
    using chrono::ChVector3f;
    using chrono::dem::CHDEM_FRICTION_MODE;
    using chrono::dem::CHDEM_TIME_INTEGRATOR;
    using chrono::dem::ChSystemDem;

    constexpr float radius = 0.5f;
    ChSystemDem system(radius, 2.5f, ChVector3f(20.0f, 20.0f, 20.0f), ChVector3f(10.0f, 10.0f, 10.0f));

    std::vector<ChVector3f> particles;
    particles.reserve(400);
    constexpr float spacing = 2.01f * radius;
    for (int layer = 0; layer < 4; ++layer) {
        for (int row = 0; row < 10; ++row) {
            for (int column = 0; column < 10; ++column) {
                particles.emplace_back(5.0f + column * spacing,
                                       5.0f + row * spacing,
                                       0.6f + layer * spacing);
            }
        }
    }
    system.SetParticles(particles);
    system.SetPsiFactors(32, 16);
    system.SetBDFixed(true);
    system.SetGravitationalAcceleration(ChVector3f(0.0f, 0.0f, -980.0f));
    system.SetKn_SPH2SPH(1.0e7f);
    system.SetKn_SPH2WALL(1.0e7f);
    system.SetGn_SPH2SPH(1.0e3f);
    system.SetGn_SPH2WALL(1.0e3f);
    system.SetKt_SPH2SPH(2.0e6f);
    system.SetKt_SPH2WALL(1.0e6f);
    system.SetGt_SPH2SPH(50.0f);
    system.SetGt_SPH2WALL(50.0f);
    system.SetStaticFrictionCoeff_SPH2SPH(0.5f);
    system.SetStaticFrictionCoeff_SPH2WALL(0.5f);
    system.SetFrictionMode(CHDEM_FRICTION_MODE::MULTI_STEP);
    system.SetTimeIntegrator(CHDEM_TIME_INTEGRATOR::FORWARD_EULER);
    system.SetFixedStepSize(1.0e-4f);
    system.Initialize();
    system.AdvanceSimulation(2.0e-3f);

    const float kinetic_energy = system.GetParticlesKineticEnergy();
    const ChVector3f first_position = system.GetParticlePosition(0);
    const bool finite = std::isfinite(kinetic_energy) && std::isfinite(first_position.x()) &&
                        std::isfinite(first_position.y()) && std::isfinite(first_position.z());

    if (!finite || system.GetNumParticles() != particles.size()) {
        std::cerr << "FAIL  invalid DEM state" << std::endl;
        return 1;
    }

    std::cout << "PASS  installed Chrono DEM package"
              << " particles=" << system.GetNumParticles()
              << " kinetic_energy=" << kinetic_energy
              << " first_z=" << first_position.z() << std::endl;
    return 0;
}
