#include "constants.hpp"
#include "satellite.hpp"
#include "satellite_swarm.hpp"
#include "simulator.hpp"
#include "transmission_channel.hpp"
#include "utils.hpp"

#include <iostream>

int main()
{
    constexpr auto simulation_origin = Point3D{.x = 0, .y = 0, .z = earth_radius};
    constexpr auto simulation_radius = 500'000.f;
    constexpr auto leo_altitude = 2'000'000.f;
    constexpr auto leo_rotation = Rotation3D{.pitch = 90.f, .yaw = 0.f, .roll = 90.f};
    constexpr auto signal_amplitude = 1.f;
    constexpr auto signal_frequency = 9e6f;
    constexpr auto signal_duration = 1;
    constexpr auto sample_rate = 10.f;
    const auto satellite_swarm_config = SatelliteSwarm::ElsaConfig{50, 1.f, 0.1f};
    const auto satellite_config = Satellite::UraConfig{4, 4, 16.f, 16.f};
    const auto transmission_channel = TransmissionChannel{2.2e9f, 30.f, 25.f, 20.f, 20e6f, 3.f, 0.3f, 1.5f, 1.f, 15.f};

    Simulator simulator;

    simulator.setup(simulation_origin,
                    simulation_radius,
                    leo_altitude,
                    leo_rotation,
                    satellite_swarm_config,
                    satellite_config,
                    signal_amplitude,
                    signal_frequency,
                    signal_duration,
                    sample_rate,
                    transmission_channel);

    
    const auto results = simulator.run();

    save_results_to_csv("/Users/tymoteuszpilarz/Desktop/satellite_swarm_simulator/src/visualizations/results.csv",
                        results);
    std::cout << results.size() * results[0].second.size();

    return 0;
}
