#pragma once
#include <Eigen/Dense>
#include <functional>
#include <cmath>
#include <algorithm>

namespace pendulum {

template <int Dim>
struct IntegratorResult {
    Eigen::Matrix<double, Dim, 1> y;
    double t;
    double h_used;
    double err_norm;
    bool accepted;
};

template <int Dim>
class Dopri5 {
public:
    using Vec = Eigen::Matrix<double, Dim, 1>;
    using RhsFn = std::function<Vec(double, const Vec&)>;

    Dopri5(RhsFn rhs, double atol = 1e-10, double rtol = 1e-10)
        : rhs_(std::move(rhs)), atol_(atol), rtol_(rtol) {}

    IntegratorResult<Dim> step(double t, const Vec& y, double h) const {
        // Dormand-Prince 5(4) — 7 stages
        Vec k1 = rhs_(t, y);
        Vec k2 = rhs_(t + h * (1.0/5.0),
                       y + h * (1.0/5.0) * k1);
        Vec k3 = rhs_(t + h * (3.0/10.0),
                       y + h * (3.0/40.0 * k1 + 9.0/40.0 * k2));
        Vec k4 = rhs_(t + h * (4.0/5.0),
                       y + h * (44.0/45.0 * k1 - 56.0/15.0 * k2
                                + 32.0/9.0 * k3));
        Vec k5 = rhs_(t + h * (8.0/9.0),
                       y + h * (19372.0/6561.0 * k1 - 25360.0/2187.0 * k2
                                + 64448.0/6561.0 * k3 - 212.0/729.0 * k4));
        Vec k6 = rhs_(t + h,
                       y + h * (9017.0/3168.0 * k1 - 355.0/33.0 * k2
                                + 46732.0/5247.0 * k3 + 49.0/176.0 * k4
                                - 5103.0/18656.0 * k5));

        // 5th-order solution
        Vec y5 = y + h * (35.0/384.0 * k1
                          + 500.0/1113.0 * k3
                          + 125.0/192.0 * k4
                          - 2187.0/6784.0 * k5
                          + 11.0/84.0 * k6);

        Vec k7 = rhs_(t + h, y5);

        // 4th-order solution (for error estimate)
        Vec y4 = y + h * (5179.0/57600.0 * k1
                          + 7571.0/16695.0 * k3
                          + 393.0/640.0 * k4
                          - 92097.0/339200.0 * k5
                          + 187.0/2100.0 * k6
                          + 1.0/40.0 * k7);

        Vec err = y5 - y4;
        double en = error_norm(err, y, y5);

        return {y5, t + h, h, en, en <= 1.0};
    }

    double suggest_h(double h, double err, double prev_err) const {
        constexpr int p = 5;
        constexpr double alpha = 0.7 / p;
        constexpr double beta_c = 0.4 / p;
        constexpr double safety = 0.9;
        constexpr double fac_min = 0.2;
        constexpr double fac_max = 10.0;

        double factor;
        if (err <= 0.0) {
            factor = fac_max;
        } else if (prev_err > 0.0) {
            factor = safety * std::pow(err, -alpha)
                     * std::pow(prev_err, beta_c);
        } else {
            factor = safety * std::pow(err, -1.0 / (p + 1));
        }
        return h * std::clamp(factor, fac_min, fac_max);
    }

private:
    RhsFn rhs_;
    double atol_, rtol_;

    double error_norm(const Vec& err, const Vec& y, const Vec& y_new) const {
        double sum = 0.0;
        for (int i = 0; i < Dim; ++i) {
            double sc = atol_ + rtol_ * std::max(std::abs(y[i]),
                                                  std::abs(y_new[i]));
            double r = err[i] / sc;
            sum += r * r;
        }
        return std::sqrt(sum / Dim);
    }
};

} // namespace pendulum
