/*+-------------------------------------------------------------------------+
  |                       MultiVehicle simulator (libmvsim)                 |
  |                                                                         |
  | Copyright (C) 2014-2025  Jose Luis Blanco Claraco                       |
  | Copyright (C) 2017  Borys Tymchenko (Odessa Polytechnic University)     |
  | Distributed under 3-clause BSD License                                  |
  |   See COPYING                                                           |
  +-------------------------------------------------------------------------+ */

#include <cmath> 
#include <cstdio>
#include <mvsim/PID_Controller.h>

using namespace mvsim;

double PID_Controller::compute(double spVel, double actVel, double torque_slope, double dt)
{
    // --------- 0) Sanitizar dt ----------
    if (dt <= 0.0){
        dt = 1e-3;
    }
    // --------- 1) Filtro en la referencia (opcional) ----------
    // Gref(s)=1/(tau_f*s+1)^2  -> dos 1er orden en cascada
    if (enable_referencefilter)
    {
        const double a = std::exp(-dt / std::max(1e-6, tau_f));

        if (n_f <= 1)
        {
            // Filtro de primer orden: Gref(s) = 1 / (tau_f*s + 1)
            spVel_f_y1 = a * spVel_f_y1 + (1.0 - a) * spVel;
            spVel = spVel_f_y1;
        }
        else
        {
            // Filtro de segundo orden: Gref(s) = 1 / (tau_f*s + 1)^2
            spVel_f_y1 = a * spVel_f_y1 + (1.0 - a) * spVel;
            spVel_f_y2 = a * spVel_f_y2 + (1.0 - a) * spVel_f_y1;
            spVel = spVel_f_y2;
        }
    }


    // --------- 2) Error ----------
    const double e = spVel - actVel;

    // --------- 3) Feedforward (primer orden) ----------
    double ff_output = 0.0;
    if (enable_feedforward)
    {
        const double a_ff = std::exp(-dt / std::max(1e-6, tau_ff));
        // OJO: si avanzar en MVSim es par NEGATIVO, invierte el signo aquí si procede:
        const double torque_est = K_ff * torque_slope;  // escala
        ff_state  = a_ff * ff_state + (1.0 - a_ff) * torque_est;
        ff_output = ff_state;
    }

    // --------- 4) PID paralelo (posicional) ----------
    // 4.a) Integrador con antiwindup por back-calculation
    // (el término Kaw*Kp ≈ Ki para mantener magnitudes; ajusta si quieres)
    const double Kaw = (enable_antiwindup && KP > 1e-12) ? (KI / KP) : 0.0;
    // OJO: todavía no aplicamos la corrección de antiwindup; primero formamos u_unsat

    // 4.b) Filtered derivative of the error, D(s) = KD * N*s/(s + N) * E(s):
    // a first-order low-pass state x_d tracks the error, x_d_dot = N*(e - x_d),
    // and the filtered derivative is d = N*(e - x_d)  (i.e., d_dot + N*d = N*e_dot).
    // If N <= 0, the derivative action is disabled.
    double D = 0.0;
    if (N > 0.0)
    {
        const double d_filt = N * (e - d_state);  // filtered derivative of e
        D = KD * d_filt;
        d_state += dt * d_filt;  // forward-Euler update of the low-pass state
    }

    // 4.c) Integramos el I puro (sin back-calc por ahora):
    i_state += KI * e * dt;

    // 4.d) Salida no saturada:
    const double P = KP * e;
    double u_unsat = P + i_state + D + ff_output;

    // --------- 5) Saturación ----------
    double u_sat = u_unsat;
    if (max_out > 0.0)
    {
        if (u_sat >  max_out) u_sat =  max_out;
        if (u_sat < -max_out) u_sat = -max_out;
    }

    // --------- 6) Anti-windup (back-calculation) ----------
    if (enable_antiwindup && Kaw > 0.0)
    {
        const double error_aw = u_sat - u_unsat;  // negativo si nos hemos pasado
        i_state += Kaw * error_aw * dt;          // corrige el integrador
        // Recalcula u con el integrador corregido (sin tocar D ni FF):
        u_unsat = P + i_state + D + ff_output;
        // y re-satura:
        u_sat = u_unsat;
        if (max_out > 0.0)
        {
            if (u_sat >  max_out) u_sat =  max_out;
            if (u_sat < -max_out) u_sat = -max_out;
        }
    }

    lastOutput = u_sat;
    return u_sat;
}

void PID_Controller::reset()
{
    lastOutput = 0.0;
    // errores
    e_n = e_n_1 = e_n_2 = 0.0; // ya no se usan, pero no molesta
    // estados
    i_state = 0.0;
    d_state = 0.0;
    ff_state = 0.0;
    spVel_f_y1 = 0.0;
    spVel_f_y2 = 0.0;
}
