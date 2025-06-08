#pragma once

#include "utils.hpp"

class Antenna
{
public:
    [[nodiscard]] Point3D get_position() const;
    [[nodiscard]] float get_phase_shift() const;

    void set_position(Point3D position);
    void set_phase_shift(float phase_shift);

private:
    float _phase_shift{};
    Point3D _position{};
};
