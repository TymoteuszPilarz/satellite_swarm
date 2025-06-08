#include "satellite_swarm.hpp"

#include "constants.hpp"
#include "utils.hpp"

#include <iostream>

SatelliteSwarm::SatelliteSwarm(Config swarm_config, Satellite::Config satellite_config, float frequency)

  : _config{std::move(swarm_config)},
    _wavelength{speed_of_light / frequency},
    _satellites(_config.satellites_positions.size(), Satellite{std::move(satellite_config), frequency})
{
}

std::vector<Point3D> SatelliteSwarm::get_antennas_positions() const
{
    return _satellites
           | std::views::transform([](const auto& satellite) {
                 return satellite.get_antennas_positions();
             })
           | std::views::join
           | std::ranges::to<std::vector>();
}

std::vector<float> SatelliteSwarm::get_antennas_phase_shifts() const
{
    return _satellites
           | std::views::transform([](const auto& satellite) {
                 return satellite.get_antennas_phase_shifts();
             })
           | std::views::join
           | std::ranges::to<std::vector>();
}

void SatelliteSwarm::set_position(const Point3D position)
{
    const auto satellites_positions =
      rotate_plane_to_face_target(_config.satellites_positions, position, {.x = 0, .y = 0, .z = 0});

    for (auto&& [satellite, position] : std::views::zip(_satellites, satellites_positions))
    {
        satellite.set_position(position);
    }

    _update_phase_shifts();
}

void SatelliteSwarm::set_target_position(const Point3D target_position)
{
    _target_position = target_position;

    for (auto& satellite : _satellites)
    {
        satellite.set_target_position(target_position);
    }
}

void SatelliteSwarm::_update_phase_shifts()
{
    const auto wavenumber = 2.0f * std::numbers::pi_v<float> / _wavelength;

    const auto ref_pos = _satellites[0].get_position();
    const auto dx_ref = _target_position.x - ref_pos.x;
    const auto dy_ref = _target_position.y - ref_pos.y;
    const auto dz_ref = _target_position.z - ref_pos.z;
    const auto ref_distance = std::sqrt((dx_ref * dx_ref) + (dy_ref * dy_ref) + (dz_ref * dz_ref));

    for (auto& satellite : _satellites)
    {
        const auto sat_pos = satellite.get_position();

        const auto dx = _target_position.x - sat_pos.x;
        const auto dy = _target_position.y - sat_pos.y;
        const auto dz = _target_position.z - sat_pos.z;

        const auto distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
        const auto relative_distance = distance - ref_distance;

        const auto phase_shift = wrap_phase(-wavenumber * relative_distance);

        satellite.set_phase_shift(phase_shift);
    }
}
