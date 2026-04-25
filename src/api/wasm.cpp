#include "engine.hpp"
#include <variant>
#include <cmath>
#include <memory>
#include <emscripten/emscripten.h>

using namespace pendulum;

struct EngineBase {
    virtual ~EngineBase() = default;
    virtual void advance(double dt) = 0;
    virtual double time() const = 0;
    virtual double energy() const = 0;
    virtual double energy_drift() const = 0;
    virtual double initial_energy() const = 0;
    virtual void get_state(double* theta, double* theta_dot) const = 0;
    virtual int n() const = 0;
};

template <int N>
struct EngineWrapper : EngineBase {
    Engine<N> engine;
    EngineWrapper(const SystemConfig& cfg, const State<N>& ic)
        : engine(cfg, ic) {}

    void advance(double dt) override { engine.advance(dt); }
    double time() const override { return engine.state().t; }
    double energy() const override { return engine.energy(); }
    double energy_drift() const override { return engine.energy_drift(); }
    double initial_energy() const override { return engine.initial_energy(); }
    int n() const override { return N; }

    void get_state(double* theta, double* theta_dot) const override {
        const auto& s = engine.state();
        for (int i = 0; i < N; ++i) {
            theta[i] = s.theta[i];
            theta_dot[i] = s.theta_dot[i];
        }
    }
};

static SystemConfig parse_config(int N, const double* params) {
    SystemConfig cfg;
    cfg.N = N;
    for (int i = 0; i < N; ++i) {
        cfg.links[i].L     = params[i * 5 + 0];
        cfg.links[i].m_rod = params[i * 5 + 1];
        cfg.links[i].m_bob = params[i * 5 + 2];
        cfg.links[i].r_bob = params[i * 5 + 3];
    }
    cfg.g = params[N * 5];
    return cfg;
}

extern "C" {

EMSCRIPTEN_KEEPALIVE
EngineBase* engine_create(int N, const double* params,
                           const double* theta0, const double* theta_dot0) {
    auto cfg = parse_config(N, params);
    cfg.validate();

    if (N == 1) {
        State<1> ic;
        ic.theta[0] = theta0[0];
        ic.theta_dot[0] = theta_dot0[0];
        return new EngineWrapper<1>(cfg, ic);
    } else if (N == 2) {
        State<2> ic;
        for (int i = 0; i < 2; ++i) {
            ic.theta[i] = theta0[i];
            ic.theta_dot[i] = theta_dot0[i];
        }
        return new EngineWrapper<2>(cfg, ic);
    } else {
        State<3> ic;
        for (int i = 0; i < 3; ++i) {
            ic.theta[i] = theta0[i];
            ic.theta_dot[i] = theta_dot0[i];
        }
        return new EngineWrapper<3>(cfg, ic);
    }
}

EMSCRIPTEN_KEEPALIVE
void engine_destroy(EngineBase* h) { delete h; }

EMSCRIPTEN_KEEPALIVE
void engine_advance(EngineBase* h, double dt) { h->advance(dt); }

EMSCRIPTEN_KEEPALIVE
double engine_time(EngineBase* h) { return h->time(); }

EMSCRIPTEN_KEEPALIVE
double engine_energy(EngineBase* h) { return h->energy(); }

EMSCRIPTEN_KEEPALIVE
double engine_energy_drift(EngineBase* h) { return h->energy_drift(); }

EMSCRIPTEN_KEEPALIVE
double engine_initial_energy(EngineBase* h) { return h->initial_energy(); }

EMSCRIPTEN_KEEPALIVE
int engine_get_n(EngineBase* h) { return h->n(); }

EMSCRIPTEN_KEEPALIVE
void engine_get_state(EngineBase* h, double* theta_out,
                       double* theta_dot_out) {
    h->get_state(theta_out, theta_dot_out);
}

EMSCRIPTEN_KEEPALIVE
void engine_get_positions(EngineBase* h, double* pos_out,
                           const double* lengths) {
    int N = h->n();
    double theta[3];
    double td[3];
    h->get_state(theta, td);

    double x = 0.0, y = 0.0;
    for (int i = 0; i < N; ++i) {
        x += lengths[i] * std::sin(theta[i]);
        y += -lengths[i] * std::cos(theta[i]);
        pos_out[2 * i + 0] = x;
        pos_out[2 * i + 1] = y;
    }
}

} // extern "C"
