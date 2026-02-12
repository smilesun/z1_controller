#include <cmath>
#include <iostream>
#include <vector>

#include "thirdparty/quadProgpp/QuadProg++.hh"

// Simple 1D MPC example (double integrator) solved via QuadProg++.
// This is a standalone demo; it does not connect to the robot.
//
// State: x = [position, velocity]^T
// Dynamics: x_{k+1} = A x_k + B u_k
// Cost: sum (x_k - x_ref)^T Q (x_k - x_ref) + u_k^T R u_k
// Constraints: u_min <= u_k <= u_max

struct MPCConfig {
    int horizon = 10;
    double dt = 0.01;
    double q_pos = 10.0;
    double q_vel = 1.0;
    double r_u = 0.1;
    double u_min = -2.0;
    double u_max = 2.0;
};

struct State {
    double pos;
    double vel;
};

int main() {
    MPCConfig cfg;

    State x0{0.0, 0.0};
    State xref{1.0, 0.0};

    const int N = cfg.horizon;
    const int nU = N;  // one control per step

    // Precompute A and B for the double integrator.
    const double dt = cfg.dt;
    const double A00 = 1.0;
    const double A01 = dt;
    const double A10 = 0.0;
    const double A11 = 1.0;
    const double B0 = 0.5 * dt * dt;
    const double B1 = dt;

    // Build QP: 0.5 * U^T G U + g0^T U
    quadprogpp::Matrix<double> G(nU, nU);
    quadprogpp::Vector<double> g0(nU);
    for (int i = 0; i < nU; ++i) {
        g0[i] = 0.0;
        for (int j = 0; j < nU; ++j) G[i][j] = 0.0;
    }

    // For each step i, compute linear mapping from U to x_i.
    // x_i = A^i x0 + sum_{j=0}^{i-1} A^{i-1-j} B u_j
    // We build contributions to G and g0 incrementally without explicit matrices.
    for (int i = 0; i < N; ++i) {
        // Compute A^i x0
        double Ai00 = 1.0;
        double Ai01 = 0.0;
        double Ai10 = 0.0;
        double Ai11 = 1.0;
        for (int k = 0; k < i; ++k) {
            const double n00 = Ai00 * A00 + Ai01 * A10;
            const double n01 = Ai00 * A01 + Ai01 * A11;
            const double n10 = Ai10 * A00 + Ai11 * A10;
            const double n11 = Ai10 * A01 + Ai11 * A11;
            Ai00 = n00; Ai01 = n01;
            Ai10 = n10; Ai11 = n11;
        }
        const double xi_pos_const = Ai00 * x0.pos + Ai01 * x0.vel;
        const double xi_vel_const = Ai10 * x0.pos + Ai11 * x0.vel;

        // For each control u_j, compute coefficient of u_j in x_i.
        for (int j = 0; j < i; ++j) {
            // Compute A^{i-1-j} * B
            double Aj00 = 1.0, Aj01 = 0.0, Aj10 = 0.0, Aj11 = 1.0;
            for (int k = 0; k < (i - 1 - j); ++k) {
                const double n00 = Aj00 * A00 + Aj01 * A10;
                const double n01 = Aj00 * A01 + Aj01 * A11;
                const double n10 = Aj10 * A00 + Aj11 * A10;
                const double n11 = Aj10 * A01 + Aj11 * A11;
                Aj00 = n00; Aj01 = n01;
                Aj10 = n10; Aj11 = n11;
            }
            const double cij_pos = Aj00 * B0 + Aj01 * B1;
            const double cij_vel = Aj10 * B0 + Aj11 * B1;

            // Cost contribution: (x_i - x_ref)^T Q (x_i - x_ref)
            // Q = diag(q_pos, q_vel)
            // Quadratic term contribution to G at (j, j)
            G[j][j] += 2.0 * (cfg.q_pos * cij_pos * cij_pos + cfg.q_vel * cij_vel * cij_vel);

            // Cross terms between u_j and u_k (k < i, k != j)
            for (int k = j + 1; k < i; ++k) {
                // Compute coefficient for u_k
                double Ak00 = 1.0, Ak01 = 0.0, Ak10 = 0.0, Ak11 = 1.0;
                for (int t = 0; t < (i - 1 - k); ++t) {
                    const double n00 = Ak00 * A00 + Ak01 * A10;
                    const double n01 = Ak00 * A01 + Ak01 * A11;
                    const double n10 = Ak10 * A00 + Ak11 * A10;
                    const double n11 = Ak10 * A01 + Ak11 * A11;
                    Ak00 = n00; Ak01 = n01;
                    Ak10 = n10; Ak11 = n11;
                }
                const double cik_pos = Ak00 * B0 + Ak01 * B1;
                const double cik_vel = Ak10 * B0 + Ak11 * B1;

                const double cross = 2.0 * (cfg.q_pos * cij_pos * cik_pos + cfg.q_vel * cij_vel * cik_vel);
                G[j][k] += cross;
                G[k][j] += cross;
            }

            // Linear term contribution (g0): 2 * (c^T Q (x_const - x_ref))
            const double err_pos = xi_pos_const - xref.pos;
            const double err_vel = xi_vel_const - xref.vel;
            g0[j] += 2.0 * (cfg.q_pos * cij_pos * err_pos + cfg.q_vel * cij_vel * err_vel);
        }
    }

    // Add control effort: sum r_u * u^2
    for (int i = 0; i < nU; ++i) {
        G[i][i] += 2.0 * cfg.r_u;
    }

    // Inequality constraints CI^T U + ci0 >= 0 for bounds
    const int nIneq = 2 * nU;
    quadprogpp::Matrix<double> CI(nU, nIneq);
    quadprogpp::Vector<double> ci0(nIneq);
    for (int j = 0; j < nIneq; ++j) {
        ci0[j] = 0.0;
        for (int i = 0; i < nU; ++i) CI[i][j] = 0.0;
    }
    for (int i = 0; i < nU; ++i) {
        // u_i >= u_min  ->  +1 * u_i + (-u_min) >= 0
        CI[i][i] = 1.0;
        ci0[i] = -cfg.u_min;
        // u_i <= u_max  ->  -1 * u_i + (u_max) >= 0
        CI[i][i + nU] = -1.0;
        ci0[i + nU] = cfg.u_max;
    }

    // No equality constraints
    quadprogpp::Matrix<double> CE(nU, 0);
    quadprogpp::Vector<double> ce0(0);

    quadprogpp::Vector<double> U(nU);
    for (int i = 0; i < nU; ++i) U[i] = 0.0;

    const double cost = quadprogpp::solve_quadprog(G, g0, CE, ce0, CI, ci0, U);
    if (std::isnan(cost)) {
        std::cerr << "QP failed to solve." << std::endl;
        return 1;
    }

    std::cout << "Solved MPC QP, cost=" << cost << "\n";
    std::cout << "First control input u0=" << U[0] << "\n";
    return 0;
}
