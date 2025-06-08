#pragma once

#include "satellite.hpp"
#include "utils.hpp"

#include <vector>

class SatelliteSwarm
{
public:
    struct Config
    {
        std::vector<Point3D> satellites_positions;
    };

    struct cUraConfig : Config
    {
    };

    struct dUraConfig : Config
    {
    };

    struct LsaConfig : Config
    {
    };

    struct ElsaConfig : Config
    {
        ElsaConfig(int num_of_satellites, float d, float st_c)
        {
            satellites_positions.reserve(num_of_satellites);

            for (auto n = 1; n <= num_of_satellites; ++n)
            {
                const auto d_n = (d * (1 - st_c) * static_cast<float>(n)) + (d * st_c);
                const auto rho_n = std::sqrtf(d_n * std::numbers::pi_v<float> * std::sqrtf(static_cast<float>(n)));
                const auto phi_n = 2.f * std::numbers::pi_v<float> * std::numbers::phi_v<float> * static_cast<float>(n);
                satellites_positions.push_back({rho_n * cos(phi_n), rho_n * sin(phi_n), 0});
            }
        }
    };

    SatelliteSwarm(Config swarm_config, Satellite::Config satellite_config, float frequency);

    [[nodiscard]] std::vector<Point3D> get_antennas_positions() const;
    [[nodiscard]] std::vector<float> get_antennas_phase_shifts() const;

    void set_position(Point3D position);
    void set_target_position(Point3D target_position);

private:
    Config _config;
    float _wavelength{};

    std::vector<Satellite> _satellites;

    Point3D _target_position{};

    void _update_phase_shifts();
};
