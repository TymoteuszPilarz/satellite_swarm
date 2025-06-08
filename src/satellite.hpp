#pragma once

#include "antenna.hpp"
#include "utils.hpp"

#include <cstddef>
#include <vector>

class Satellite
{
public:
    struct Config
    {
        std::vector<Point3D> antennas_positions;
    };

    struct UraConfig : Config
    {
        UraConfig(int num_of_rows, int num_of_columns, float spacing_rows, float spacing_columns)
        {
            antennas_positions.reserve(static_cast<std::size_t>(num_of_rows) * num_of_columns);

            const auto x_offset = -((static_cast<float>(num_of_columns) - 1) * spacing_columns) / 2.0f;
            const auto y_offset = -((static_cast<float>(num_of_rows) - 1) * spacing_rows) / 2.0f;

            for (auto row = 0; row < num_of_rows; ++row)
            {
                for (auto col = 0; col < num_of_columns; ++col)
                {
                    const auto antenna_position = Point3D{.x = (static_cast<float>(col) * spacing_columns) + x_offset,
                                                          .y = (static_cast<float>(row) * spacing_rows) + y_offset,
                                                          .z = 0.f};

                    antennas_positions.push_back(antenna_position);
                }
            }
        }
    };

    Satellite(Config config, float frequency);

    [[nodiscard]] Point3D get_position() const;
    [[nodiscard]] std::vector<Point3D> get_antennas_positions() const;
    [[nodiscard]] std::vector<float> get_antennas_phase_shifts() const;

    void set_position(Point3D position);
    void set_target_position(Point3D target_position);
    void set_phase_shift(float phase_shift);

private:
    Config _config;
    float _wavelength{};

    std::vector<Antenna> _antennas;

    Point3D _position{};
    Point3D _target_position{};

    void _update_phase_shifts();
};
