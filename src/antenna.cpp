#include "antenna.hpp"

Point3D Antenna::get_position() const
{
    return _position;
}

float Antenna::get_phase_shift() const
{
    return _phase_shift;
}

void Antenna::set_position(Point3D position)
{
    _position = position;
}

void Antenna::set_phase_shift(float phase_shift)
{
    _phase_shift = phase_shift;
}
