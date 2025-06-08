#include "transmission_channel.hpp"

#include "constants.hpp"
#include "utils.hpp"

#include <cmath>
#include <numbers>
#include <random>

TransmissionChannel::TransmissionChannel(const float carrier_freq_hz,
                                         const float tx_power_dBW,
                                         const float tx_directivity_dB,
                                         const float rx_gain_dB,
                                         const float channel_bandwidth_hz,
                                         const float noise_figure_dB,
                                         const float atmospheric_loss_dB,
                                         const float shadow_margin_dB,
                                         const float extra_system_loss_dB,
                                         const float rician_K_dB)
  : carrier_freq_hz{carrier_freq_hz},
    tx_power_dBW{tx_power_dBW},
    tx_directivity_dB{tx_directivity_dB},
    rx_gain_dB{rx_gain_dB},
    channel_bandwidth_hz{channel_bandwidth_hz},
    noise_figure_dB{noise_figure_dB},
    atmospheric_loss_dB{atmospheric_loss_dB},
    shadow_margin_dB{shadow_margin_dB},
    extra_system_loss_dB{extra_system_loss_dB},
    rician_K_dB{rician_K_dB},
    generator{std::random_device{}()}
{
}

std::vector<std::complex<float>>
TransmissionChannel::process_signal(const std::vector<std::complex<float>>& input_signal,
                                    const Point3D& source_position,
                                    const Point3D& destination_position)
{
    // if (is_segment_intersecting_sphere(earth_radius, source_position, destination_position))
    // {
    //     return {};
    // }

    const auto distance_m = distance(source_position, destination_position);

    // Free space path loss
    const auto fspl_dB = (20.f * std::log10f(4.f * std::numbers::pi_v<float> * carrier_freq_hz / speed_of_light))
                         + (20.f * std::log10f(distance_m));

    // EIRP and received power
    const auto eirp = tx_power_dBW + tx_directivity_dB;
    const auto prx_dBW = eirp - fspl_dB - atmospheric_loss_dB - shadow_margin_dB - extra_system_loss_dB;

    // SNR
    const auto grt = rx_gain_dB - noise_figure_dB - (10.f * std::log10f(290.f));
    const auto snr_dB = eirp
                        + grt
                        - fspl_dB
                        - atmospheric_loss_dB
                        - shadow_margin_dB
                        - extra_system_loss_dB
                        - boltzmann_constant_dB
                        - (10.f * std::log10f(channel_bandwidth_hz));

    const auto snr_linear = std::powf(10.f, snr_dB / 10.f);
    const auto signal_power = std::powf(10.f, prx_dBW / 10.f);
    const auto noise_power = signal_power / snr_linear;

    // Distributions
    auto noise_dist = std::normal_distribution<float>(0.f, std::sqrtf(noise_power / 2.f));
    auto rician_dist = std::normal_distribution<float>(0.f, 1.f);

    // Rician K-factor
    const auto k_factor_linear = std::powf(10.f, rician_K_dB / 10.f);
    const auto los_ampl = std::sqrtf(k_factor_linear / (k_factor_linear + 1.f));
    const auto scatter_ampl = std::sqrtf(1.f / (k_factor_linear + 1.f));

    // Phase shift due to propagation distance
    const float wavelength = speed_of_light / carrier_freq_hz;
    const float phase_shift_rad = -2.f * std::numbers::pi_v<float> * distance_m / wavelength;
    const std::complex<float> phase_shift = std::exp(std::complex<float>(0.f, phase_shift_rad));

    auto output = std::vector<std::complex<float>>{};
    output.reserve(input_signal.size());

    for (const auto input_sample : input_signal)
    {
        // Apply phase shift from propagation delay
        const auto shifted_sample = input_sample * phase_shift;

        // Rician fading
        auto scatter = std::complex<float>(rician_dist(generator), rician_dist(generator));
        scatter /= std::numbers::sqrt2_v<float>; // unit power normalization

        const auto fading = los_ampl + scatter_ampl * scatter;

        // AWGN
        const auto noise = std::complex<float>(noise_dist(generator), noise_dist(generator));

        // Final received sample
        output.push_back(shifted_sample * fading + noise);
    }

    return output;
}
