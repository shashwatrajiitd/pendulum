#pragma once
#include "core/types.hpp"
#include "core/eom.hpp"
#include "integrators/dopri5.hpp"
#include <stdexcept>
#include <cmath>

namespace pendulum {

template <int N>
class Engine {
    static constexpr int Dim = 2 * N;
    using Vec = Eigen::Matrix<double, Dim, 1>;

public:
    Engine(const SystemConfig& cfg, const State<N>& ic)
        : config_(cfg),
          state_(ic),
          coeffs_(compute_coefficients<N>(cfg)),
          E0_(compute_total_energy<N>(coeffs_, cfg.g,
                                       ic.theta, ic.theta_dot)),
          integrator_(make_rhs(), 1e-10, 1e-10) {
        config_.validate();
    }

    void step() {
        Vec y = pack(state_);

        for (int attempts = 0; attempts < 200; ++attempts) {
            auto res = integrator_.step(state_.t, y, h_);
            double h_new = integrator_.suggest_h(h_, res.err_norm, prev_err_);

            if (res.accepted) {
                unpack(res.y, res.t, state_);
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
            Vec y = pack(state_);

            auto res = integrator_.step(state_.t, y, h_try);
            double h_new = integrator_.suggest_h(h_try, res.err_norm,
                                                  prev_err_);

            if (res.accepted) {
                unpack(res.y, res.t, state_);
                prev_err_ = res.err_norm;
                h_ = h_new;
            } else {
                h_ = h_new;
            }
        }
    }

    double energy() const {
        return compute_total_energy<N>(coeffs_, config_.g,
                                       state_.theta, state_.theta_dot);
    }

    double energy_drift() const {
        if (std::abs(E0_) < 1e-30)
            return std::abs(energy() - E0_);
        return std::abs((energy() - E0_) / E0_);
    }

    const State<N>& state() const { return state_; }
    double initial_energy() const { return E0_; }

private:
    SystemConfig config_;
    State<N> state_;
    Coefficients<N> coeffs_;
    double E0_;
    double h_ = 0.01;
    double prev_err_ = 0.0;
    Dopri5<Dim> integrator_;

    static Vec pack(const State<N>& s) {
        Vec y;
        y.template head<N>() = s.theta;
        y.template tail<N>() = s.theta_dot;
        return y;
    }

    static void unpack(const Vec& y, double t, State<N>& s) {
        s.theta = y.template head<N>();
        s.theta_dot = y.template tail<N>();
        s.t = t;
    }

    typename Dopri5<Dim>::RhsFn make_rhs() {
        return [this](double /*t*/, const Vec& y) -> Vec {
            VecN<N> theta = y.template head<N>();
            VecN<N> theta_dot = y.template tail<N>();
            VecN<N> theta_ddot = compute_acceleration<N>(
                coeffs_, config_.g, theta, theta_dot);

            Vec dy;
            dy.template head<N>() = theta_dot;
            dy.template tail<N>() = theta_ddot;
            return dy;
        };
    }
};

} // namespace pendulum
