#pragma once
#include "core/types.hpp"
#include "core/pendulum_n1.hpp"
#include "integrators/dopri5.hpp"
#include <stdexcept>
#include <cmath>

namespace pendulum {

template <int N>
class Engine;

template <>
class Engine<1> {
    static constexpr int Dim = 2;
    using Vec = Eigen::Matrix<double, Dim, 1>;

public:
    Engine(const SystemConfig& config, const State<1>& ic)
        : config_(config),
          state_(ic),
          coeffs_(compute_coeffs_n1(config.links[0])),
          E0_(total_energy_n1(coeffs_, config.g,
                              ic.theta[0], ic.theta_dot[0])),
          integrator_(make_rhs(), 1e-10, 1e-10) {
        config_.validate();
    }

    void step() {
        Vec y;
        y << state_.theta[0], state_.theta_dot[0];

        for (int attempts = 0; attempts < 200; ++attempts) {
            auto res = integrator_.step(state_.t, y, h_);
            double h_new = integrator_.suggest_h(h_, res.err_norm, prev_err_);

            if (res.accepted) {
                state_.theta[0] = res.y[0];
                state_.theta_dot[0] = res.y[1];
                state_.t = res.t;
                prev_err_ = res.err_norm;
                h_ = h_new;
                return;
            }
            h_ = h_new;
        }
        throw std::runtime_error("DOPRI5: step failed to converge");
    }

    void advance(double dt) {
        double t_end = state_.t + dt;
        while (state_.t < t_end - 1e-15) {
            double h_try = std::min(h_, t_end - state_.t);
            Vec y;
            y << state_.theta[0], state_.theta_dot[0];

            auto res = integrator_.step(state_.t, y, h_try);
            double h_new = integrator_.suggest_h(h_try, res.err_norm,
                                                  prev_err_);

            if (res.accepted) {
                state_.theta[0] = res.y[0];
                state_.theta_dot[0] = res.y[1];
                state_.t = res.t;
                prev_err_ = res.err_norm;
                h_ = h_new;
            } else {
                h_ = h_new;
            }
        }
    }

    double energy() const {
        return total_energy_n1(coeffs_, config_.g,
                               state_.theta[0], state_.theta_dot[0]);
    }

    double energy_drift() const {
        if (std::abs(E0_) < 1e-30)
            return std::abs(energy() - E0_);
        return std::abs((energy() - E0_) / E0_);
    }

    const State<1>& state() const { return state_; }
    double initial_energy() const { return E0_; }

private:
    SystemConfig config_;
    State<1> state_;
    CoeffsN1 coeffs_;
    double E0_;
    double h_ = 0.01;
    double prev_err_ = 0.0;
    Dopri5<Dim> integrator_;

    Dopri5<Dim>::RhsFn make_rhs() {
        return [this](double /*t*/, const Vec& y) -> Vec {
            Vec dy;
            dy[0] = y[1];
            dy[1] = acceleration_n1(coeffs_, config_.g, y[0]);
            return dy;
        };
    }
};

} // namespace pendulum
