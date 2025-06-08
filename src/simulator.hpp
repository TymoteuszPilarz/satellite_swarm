#pragma once

#include "satellite_swarm.hpp"
#include "transmission_channel.hpp"
#include "utils.hpp"

#include <complex>
#include <memory>
#include <vector>
#include <utility>

class Simulator
{
public:
    void setup(Point3D simulation_origin,
               float simulation_radius,
               float leo_altitude,
               Rotation3D leo_rotation,
               SatelliteSwarm::Config satellite_swarm_config,
               Satellite::Config satellite_config,
               float signal_amplitude,
               float signal_frequency,
               float signal_duration,
               float sample_rate,
               TransmissionChannel transmission_channel);

    std::vector<std::pair<Point3D, std::vector<std::pair<Point3D, float>>>> run();

private:
    static constexpr int _num_of_earth_points{1'000'000};
    static constexpr int _num_of_leo_points{4};

    float _signal_amplitude{};
    float _signal_frequency{};
    float _signal_duration{};
    float _sample_rate{};
    int _num_of_samples{};
    TransmissionChannel _transmission_channel{1.f};

    std::vector<Point3D> _simulation_points;
    std::vector<Point3D> _leo_points;

    std::unique_ptr<SatelliteSwarm> _satellite_swarm;

    [[nodiscard]] std::vector<std::complex<float>> _generate_signal(float phase_shift) const;
};
