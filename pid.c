/* Name: pid.c */
/* Description: */
/* Author: John Houlihan */

/**
* MIT License
*
* Copyright (c) 2025 John Houlihan
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#include "pid.h"


/**
 * @brief Initialize pid_t structure passed by pointer.
 * @note FINAL in the implementation comments means it is the implementation
 *       of interest. All other member values are placeholders for eventual
 *       final implementation.
 *
 * @param self  An instance of pid_t
 * @return
 */
pid_status_t pid_init(pid_t* self)
{
    self->kp = 10.0;  /* proportional gain */
    self->ti = 0.0;  /* integration time constant */
    self->td = 0.0;  /* derivative time constant */
    self->ts = 0.001;  /* FINAL - 1 ms converted to units of seconds */
    self->alpha = 1.0;  /* low-pass filter smoothing factor (alpha) */
    self->sp = 35.0;  /* setpoint temperature in units of degC */
    self->pv = 0.0;  /* process variable (measured temperature in units of degC) */
    self->deadband = 0.0;  /* deadband */
    self->control_limit_max = 0.0;  /* maximum acceptable control output */
    self->control_limit_min = 0.0;  /* minimum acceptable control output */
    self->polarity = PID_POLARITY_DIRECT;
    self->error = 0.0;  /* difference between setpoint and process varible */
    self->error_last = 0.0;
    self->error_last2 = 0.0;
    self->error_lpf = 0.0;
    self->error_lpf_last = 0.0;
    self->error_lpf_last2 = 0.0;
    self->control = 0.0;  /* control function; output from the PID algorithm */
    return PID_SUCCESS;
}


/**
 * @brief Reset the PID loop
 *
 * @param[in] self  An instance of pid_t
 * @return
 */
pid_status_t pid_reset(pid_t* self)
{
    self->error = 0.0;  /* difference between setpoint and process varible */
    self->error_last = 0.0;
    self->error_last2 = 0.0;
    self->error_lpf = 0.0;
    self->error_lpf_last = 0.0;
    self->error_lpf_last2 = 0.0;
    self->control = 0.0;  /* control function; output from the PID algorithm */
    return PID_SUCCESS;
}


/**
 * @brief Set the proportional gain, Kp
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   kp  proportional gain
 * @return
 */
pid_status_t pid_set_proportional_gain(pid_t* self, pid_real_t kp)
{
    if (kp < 0.0)
    {
        self->kp = 0.0;
    }
    else
    {
        self->kp = kp;
    }
    return PID_SUCCESS;
}


/**
 * @brief Get the proportional gain, Kp
 *
 * @param[in]  self  An instance of pid_t
 * @param[out]   kp  proportional gain
 * @return
 */
pid_status_t pid_get_proportional_gain(pid_t* self, pid_real_t* kp)
{
    *kp = self->kp;
    return PID_SUCCESS;
}


/**
 * @brief Set the integral time, Ti
 * @note The integral term is zero when the integral time
 *       is set to zero
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   ti  integral time
 * @return
 */
pid_status_t pid_set_integral_time(pid_t* self, pid_real_t ti)
{
    if (ti < 0.0)
    {
        self->ti = 0.0;
    }
    else
    {
        self->ti = ti;
    }
    return PID_SUCCESS;
}


/**
 * @brief Get the intregral time, Ti
 *
 * @param[in] self  An instance of pid_t
 * @param[out]  ti  integral time
 * @return
 */
pid_status_t pid_get_integral_time(pid_t* self, pid_real_t* ti)
{
    *ti = self->ti;
    return PID_SUCCESS;
}


/**
 * @brief Set the derivative time, Td
 * @note The derivate term is disabled when the derivate
 *       time is set to zero.
 *
 *       If the user sets the sampling period to zero,
 *       then the derivate term is disabled.
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   td  derivative time
 * @return
 */
pid_status_t pid_set_derivative_time(pid_t* self, pid_real_t td)
{
    if (td < 0.0)
    {
        self->td = 0.0;
    }
    else
    {
        self->td = td;
    }
    return PID_SUCCESS;
}


/**
 * @brief Get the derivative time, Td
 *
 * @param[in]  self  An instance of pid_t
 * @param[out]   td  derivative time
 * @return
 */
pid_status_t pid_get_derivative_time(pid_t* self, pid_real_t* td)
{
    *td = self->td;
    return PID_SUCCESS;
}


/**
 * @brief Set the sampling period in units of seconds
 *
 * @param[in] self  An instance of pid_t
 * @param[in] kp  proportional gain
 * @param[in] ti  integral gain
 * @param[in] td  derivate gain
 * @return
 */
pid_status_t pid_set_gains(pid_t* self, pid_real_t kp, pid_real_t ti, pid_real_t td)
{
    pid_set_proportional_gain(self, kp);
    pid_set_integral_time(self, ti);
    pid_set_derivative_time(self, td);
    return PID_SUCCESS;
}


/**
 * @brief Set the sampling period in units of seconds
 *
 * @param[in] self  An instance of pid_t
 * @param[out] kp  proportional gain
 * @param[out] ti  integral gain
 * @param[out] td  derivate gain
 * @return
 */
pid_status_t pid_get_gains(pid_t* self, pid_real_t* kp, pid_real_t* ti, pid_real_t* td)
{
    *kp = self->kp;
    *ti = self->ti;
    *td = self->td;
    return PID_SUCCESS;
}


/**
 * @brief Set the sampling period in units of seconds
 * @note If the sampling period is set to zero, then the
 *       derivate term is disabled.
 *
 * @param[in] self  An instance of pid_t
 * @param[in] gain  the sampling period in units of seconds
 * @return
 */
pid_status_t pid_set_sampling_period(pid_t* self, pid_real_t period)
{
    if (period < 0.0)
    {
        self->ts = 0.0;
    }
    else
    {
        self->ts = period;
    }
    return PID_SUCCESS;
}


/**
 * @brief Get the sampling period in units of seconds
 *
 * @param[in]  self  An instance of pid_t
 * @param[out] gain  the sampling period in units of seconds
 * @return
 */
pid_status_t pid_get_sampling_period(pid_t* self, pid_real_t* period)
{
    *period = self->ts;
    return PID_SUCCESS;
}


/**
 * @brief Set the low-pass filter smoothing factor (alpha)
 *
 * @param[in]  self  An instance of pid_t
 * @param[in] alpha  the low-pass filter smoothing factor
 * @return
 */
pid_status_t pid_set_smoothing_factor(pid_t* self, pid_real_t alpha)
{
    self->alpha = alpha;
    return PID_SUCCESS;
}


/**
 * @brief Get the low-pass filter smoothing factor (alpha)
 *
 * @param[in]   self  An instance of pid_t
 * @param[out] alpha  the low-pass filter smoothing factor
 * @return
 */
pid_status_t pid_get_smoothing_factor(pid_t* self, pid_real_t* alpha)
{
    *alpha = self->alpha;
    return PID_SUCCESS;
}


/**
 * @brief Set the PID setpoint value
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   sp  the PID setpoint value
 * @return
 */
pid_status_t pid_set_setpoint(pid_t* self, pid_real_t sp)
{
    self->sp = sp;
    return PID_SUCCESS;
}


/**
 * @brief Get the PID setpoint value
 *
 * @param[in] self  An instance of pid_t
 * @param[out]  sp  the PID setpoint value
 * @return
 */
pid_status_t pid_get_setpoint(pid_t* self, pid_real_t* sp)
{
    *sp = self->sp;
    return PID_SUCCESS;
}


/**
 * @brief Set the PID deadband value
 * @note The correction to the control variable will not be computed
 *       when the measured error is within the deadband
 *
 * @param[in] self  An instance of pid_t
 * @param[in] deadband  the PID deadband value
 * @return
 */
pid_status_t pid_set_deadband(pid_t* self, pid_real_t deadband)
{
    self->deadband = deadband;
    return PID_SUCCESS;
}


/**
 * @brief Get the PID deadband value
 *
 * @param[in]      self  An instance of pid_t
 * @param[out] deadband  the PID deadband value
 * @return
 */
pid_status_t pid_get_deadband(pid_t* self, pid_real_t* deadband)
{
    *deadband = self->deadband;
    return PID_SUCCESS;
}


/**
 * @brief Set the PID control limits
 *
 * @param[in]   self  An instance of pid_t
 * @param[in] cl_max  the maximum PID control limit
 * @param[in] cl_min  the minimum PID control limit
 * @return
 */
pid_status_t pid_set_control_limits(pid_t* self, pid_real_t cl_max, pid_real_t cl_min)
{
    self->control_limit_max = cl_max;
    self->control_limit_min = cl_min;
    return PID_SUCCESS;
}


/**
 * @brief Get the PID control limits
 *
 * @param[in]    self  An instance of pid_t
 * @param[out] cl_max  the maximum PID control limit
 * @param[out] cl_min  the minimum PID control limit
 * @return
 */
pid_status_t pid_get_control_limits(pid_t* self, pid_real_t* cl_max, pid_real_t* cl_min)
{
    *cl_max = self->control_limit_max;
    *cl_min = self->control_limit_min;
    return PID_SUCCESS;
}


/**
 * @brief Set the PID polarity
 * @note
 *
 * 1) Control polarity determines whether a controller acts as negative feedback (corrective)
 * or positive feedback (runaway).
 *
 * 2) Negative Feedback (Standard): The controller output pushes the system in the opposite direction
 * of the error, reducing it towards a target setpoint.
 *
 * 3) Positive Feedback (Inverse): Pushes the system away from the setpoint. This is rarely used except in
 * specific oscillation applications (e.g., active magnetic bearings)
 *
 * 4) The core function of the controller is e(t) = Setpoint - Process Value. The polarity of this error
 * dictates how the Proportional (P), Integral (I), and Derivative (D) terms compute the final control output.
 *
 * 5) Direct Acting: As the Process Value exceeds the Setpoint, the controller output decreases.
 * (Common for cooling or motor speed).
 *
 * 6) Reverse Acting: As the Process Value exceeds the Setpoint, the controller output increases.
 * (Common for heating systems).
 *
 * @param[in]     self  An instance of pid_t
 * @param[in] polarity  the PID polarity
 * @return
 */
pid_status_t pid_set_polarity(pid_t* self, pid_polarity_t polarity)
{
    self->polarity = polarity;
    return PID_SUCCESS;
}


/**
 * @brief Get the PID polarity
 *
 * @param[in]      self  An instance of pid_t
 * @param[out] polarity  the PID polarity
 * @return
 */
pid_status_t pid_get_polarity(pid_t* self, pid_polarity_t* polarity)
{
    *polarity = self->polarity;
    return PID_SUCCESS;
}


/**
 * @brief Compute the correction term using the PID algorithm.
 *
 * TODO: More investigation of Discrete PID implementation using Laplace-transform
 * and/or Z-transform. Look for 'a discrete-time controller in the Z-domain' and
 * 'difference equation for controller output.'
 *
 * Explanation of the PID algorithm used in this function search for 'Discrete implementation' in:
 * https://en.wikipedia.org/wiki/Proportional%E2%80%93integral%E2%80%93derivative_controller
 *
 * Anti-wind-up via integrator clamping:
 * http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-reset-windup/
 *
 * Proportional of Measurement:
 * http://brettbeauregard.com/blog/2017/06/proportional-on-measurement-the-code/
 *
 * See following for potential implemention of discrete PID algorithm:
 * https://www.youtube.com/watch?v=zOByx3Izf5U&t=160s
 * https://github.com/pms67/PID/blob/master/PID.c
 * 
 * See for discrete PID implementation:
 * https://www.scilab.org/discrete-time-pid-controller-implementation
 *
 *
 * @param[in]     self  An instance of pid_t
 * @param[in]       pv  the process variable (i.e. the measured variable)
 * @param[out] control  control function; output from the PID algorithm
 * @return
 */
pid_status_t pid_compute(pid_t* self, pid_real_t pv, pid_real_t* control)
{
    pid_real_t control_kp = 0.0;
    pid_real_t control_ti = 0.0;
    pid_real_t control_td = 0.0;
    pid_real_t error_magnitude = 0.0;

    /* Compute the error value */
    if (self->polarity == PID_POLARITY_DIRECT)
    {
        self->error = self->sp - pv;;
    }
    else  /* self->polarity == PID_POLARITY_REVERSE */
    {
        self->error = -(self->sp - pv);
    }

    /* Deadband logic */
    error_magnitude = (self->error < 0.0) ? -self->error : self->error;
    if (error_magnitude < self->deadband)
    {
        *control = 0.0;
        return PID_MEASURED_ERROR_WITHIN_DEADBAND;
    }

    /* Apply a low-pass filter (LPF) to the error signal */
    self->error_lpf = self->alpha * self->error + (1 - self->alpha) * self->error_lpf_last;

    /* Compute the correction terms */
    control_kp = self->kp * (self->error - self->error_last);
    if (self->ti != 0.0)
    {
        control_ti = (self->kp / self->ti) * (self->error * self->ts);
    }
    if (self->ts != 0.0)
    {
        control_td = (
            (self->kp * self->td)
            * ( (self->error_lpf - 2 * self->error_lpf_last + self->error_lpf_last2) / self->ts )
        );
    }

    /* Compute Rest of PID Output */

    /**
     * Discrete PID so the previous control value is added to the new control value; therefore,
     * the control value is a running sum (TODO: confirm authenticity of this comment!!!)
     */
    self->control += control_kp + control_ti + control_td;

    /* Anti-wind-up via clamping */
    if (self->control > self->control_limit_max)
    {
        self->control = self->control_limit_max;
    }
    else if (self->control < self->control_limit_min)
    {
        self->control = self->control_limit_min;
    }

    *control = self->control;

    /* Remember some variables for next time */
    self->error_last2 = self->error_last;
    self->error_last = self->error;
    self->error_lpf_last2 = self->error_lpf_last;
    self->error_lpf_last = self->error_lpf;

    return PID_SUCCESS;
}


int main(int argc, char** argv)
{
    pid_t pid;  /* implement global PID object */

    pid_real_t kp = 10.0;  /* define proportional gain */
    pid_real_t ti = 0.0;   /* define integral time constant */
    pid_real_t td = 0.0;   /* define derivative time constant */

    pid_init(&pid);  /* initialize all members */
    pid_set_gains(&pid, kp, ti, td);  /* set all gains */

    return 0;
}
