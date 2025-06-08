#pragma once

#include "utils.hpp"

#include <complex>
#include <random>
#include <vector>

class TransmissionChannel
{
public:
    explicit TransmissionChannel(float carrier_freq_hz = 2.2e9f, // Carrier frequency [Hz]
                                 float tx_power_dBW = 30.0f, // Transmit power [dBW]
                                 float tx_directivity_dB = 25.0f, // Transmit antenna directivity [dB]
                                 float rx_gain_dB = 20.0f, // Receive antenna gain [dB]
                                 float channel_bandwidth_hz = 20e6f, // Channel bandwidth [Hz]
                                 float noise_figure_dB = 3.0f, // Receiver noise figure [dB]
                                 float atmospheric_loss_dB = 0.3f, // Atmospheric losses [dB]
                                 float shadow_margin_dB = 1.5f, // Shadow fading margin [dB]
                                 float extra_system_loss_dB = 1.0f, // Extra system losses [dB]
                                 float rician_K_dB = 15.0f); // Rician K-factor [dB]

    std::vector<std::complex<float>> process_signal(const std::vector<std::complex<float>>& input_signal,
                                                    const Point3D& source_position,
                                                    const Point3D& destination_position);

private:
    float carrier_freq_hz;
    float tx_power_dBW;
    float tx_directivity_dB;
    float rx_gain_dB;
    float channel_bandwidth_hz;
    float noise_figure_dB;
    float atmospheric_loss_dB;
    float shadow_margin_dB;
    float extra_system_loss_dB;
    float rician_K_dB;

    std::default_random_engine generator;
};
