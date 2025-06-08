#include "satellite.hpp"

#include "constants.hpp"
#include "utils.hpp"

#include <numbers>
#include <ranges>

#include <iostream>

Satellite::Satellite(Config config, const float frequency)
  : _config{std::move(config)},
    _wavelength{static_cast<float>(speed_of_light) / frequency},
    _antennas(_config.antennas_positions.size())
{
}

Point3D Satellite::get_position() const
{
    return _position;
}

std::vector<Point3D> Satellite::get_antennas_positions() const
{
    return _antennas
           | std::views::transform([](const auto& antenna) {
                 return antenna.get_position();
             })
           | std::ranges::to<std::vector>();
}

std::vector<float> Satellite::get_antennas_phase_shifts() const
{
    return _antennas
           | std::views::transform([](const auto& antenna) {
                 return antenna.get_phase_shift();
             })
           | std::ranges::to<std::vector>();
}

void Satellite::set_position(const Point3D position)
{
    _position = position;

    const auto antennas_positions =
      rotate_plane_to_face_target(_config.antennas_positions, position, {.x = 0, .y = 0, .z = 0});

    for (auto&& [antenna, position] : std::views::zip(_antennas, antennas_positions))
    {
        antenna.set_position(position);
    }

    _update_phase_shifts();
}

void Satellite::set_target_position(const Point3D target_position)
{
    _target_position = target_position;

    _update_phase_shifts();
}

void Satellite::set_phase_shift(const float phase_shift)
{
    for (auto& antenna : _antennas)
    {
        antenna.set_phase_shift(wrap_phase(antenna.get_phase_shift() + phase_shift));
    }
}

void Satellite::_update_phase_shifts()
{
    const auto wavenumber = 2.0f * std::numbers::pi_v<float> / _wavelength;

    const auto ref_pos = _antennas[0].get_position();
    const auto dx_ref = _target_position.x - ref_pos.x;
    const auto dy_ref = _target_position.y - ref_pos.y;
    const auto dz_ref = _target_position.z - ref_pos.z;
    const auto ref_distance = std::sqrt((dx_ref * dx_ref) + (dy_ref * dy_ref) + (dz_ref * dz_ref));

    for (auto& antenna : _antennas)
    {
        const auto antenna_pos = antenna.get_position();
        const auto dx = _target_position.x - antenna_pos.x;
        const auto dy = _target_position.y - antenna_pos.y;
        const auto dz = _target_position.z - antenna_pos.z;

        const auto distance = std::sqrt((dx * dx) + (dy * dy) + (dz * dz));
        const auto relative_distance = distance - ref_distance;

        const auto phase_shift = wrap_phase(-wavenumber * relative_distance);

        antenna.set_phase_shift(phase_shift);
    }
}
