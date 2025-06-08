#include "simulator.hpp"

#include "constants.hpp"
#include "satellite_swarm.hpp"
#include "transmission_channel.hpp"
#include "utils.hpp"
#include <Eigen/Dense>

#include <functional>
#include <numbers>

void Simulator::setup(const Point3D simulation_origin,
                      const float simulation_radius,
                      const float leo_altitude,
                      const Rotation3D leo_rotation,
                      SatelliteSwarm::Config satellite_swarm_config,
                      Satellite::Config satellite_config,
                      const float signal_amplitude,
                      const float signal_frequency,
                      const float signal_duration,
                      const float sample_rate,
                      TransmissionChannel transmission_channel)
{
    _signal_amplitude = signal_amplitude;
    _signal_frequency = signal_frequency;
    _signal_duration = signal_duration;
    _sample_rate = sample_rate;
    _num_of_samples = static_cast<int>(_signal_duration * _sample_rate);
    _transmission_channel = transmission_channel;

    const auto earth_points = gen_sphere_points(earth_radius, _num_of_earth_points);

    _simulation_points = find_nearby_points(earth_points, simulation_origin, simulation_radius);
    _leo_points = gen_circle_points(earth_radius + leo_altitude, _num_of_leo_points);

    const auto leo_rotation_X =
      Eigen::AngleAxisf{leo_rotation.pitch * std::numbers::pi_v<float> / 180, Eigen::Vector3f::UnitX()};
    const auto leo_rotation_Z =
      Eigen::AngleAxisf{leo_rotation.yaw * std::numbers::pi_v<float> / 180, Eigen::Vector3f::UnitZ()};
    const auto leo_rotation_Y =
      Eigen::AngleAxisf{leo_rotation.roll * std::numbers::pi_v<float> / 180, Eigen::Vector3f::UnitY()};

    const auto leo_rotation_mat =
      Eigen::Matrix3f{(leo_rotation_Y * leo_rotation_Z * leo_rotation_X).toRotationMatrix()};

    for (auto& leo_point : _leo_points)
    {
        auto leo_point_vec = Eigen::Vector3f{leo_point.x, leo_point.y, leo_point.z};
        leo_point_vec = leo_rotation_mat * leo_point_vec;
        leo_point.x = leo_point_vec.x();
        leo_point.y = leo_point_vec.y();
        leo_point.z = leo_point_vec.z();
    }
    
    _satellite_swarm = std::make_unique<SatelliteSwarm>(std::move(satellite_swarm_config),
                                                        std::move(satellite_config),
                                                        _signal_frequency);
    _satellite_swarm->set_target_position(simulation_origin);
}

std::vector<std::pair<Point3D, std::vector<std::pair<Point3D, float>>>> Simulator::run()
{
    auto results = std::vector<std::pair<Point3D, std::vector<std::pair<Point3D, float>>>>{};
    results.reserve(_leo_points.size());

    for (const auto leo_point : std::views::counted(_leo_points.begin() + 2, 1))
    {
        _satellite_swarm->set_position(leo_point);

        auto results_for_leo_point = std::vector<std::pair<Point3D, float>>{};
        results_for_leo_point.reserve(_simulation_points.size());

        for (const auto simulation_point : _simulation_points)
        {
            const auto antennas_positions = _satellite_swarm->get_antennas_positions();
            const auto antennas_phase_shifts = _satellite_swarm->get_antennas_phase_shifts();

            auto received_signal = std::vector<std::complex<float>>(_num_of_samples);

            for (const auto&& [antenna_position, antenna_phase_shift] :
                 std::views::zip(antennas_positions, antennas_phase_shifts))
            {
                const auto signal = _generate_signal(antenna_phase_shift);

                const auto processed_signal =
                  _transmission_channel.process_signal(signal, antenna_position, simulation_point);

                std::ranges::transform(received_signal, processed_signal, received_signal.begin(), std::plus{});
            };

            const auto power = received_signal | std::views::transform([](const auto sample) {
                                   return std::norm(sample);
                               });
            const auto avg_power = std::ranges::fold_left(power, 0, std::plus{}) / static_cast<float>(power.size());

            results_for_leo_point.emplace_back(simulation_point, avg_power);
        }

        results.emplace_back(leo_point, std::move(results_for_leo_point));
    }

    return results;
}

std::vector<std::complex<float>> Simulator::_generate_signal(const float phase_shift) const
{
    using namespace std::complex_literals;

    const auto times = linspace(0, _signal_duration, _num_of_samples);

    std::vector<std::complex<float>> signal;
    signal.reserve(_num_of_samples);

    for (const auto& time : times)
    {
        const auto sample =
          std::polar(_signal_amplitude, (2 * std::numbers::pi_v<float> * _signal_frequency * time) + phase_shift);
        signal.push_back(sample);
    }

    return signal;
}
