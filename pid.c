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
#include <stdint.h>
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
uint8_t pid_init(pid_t* self) {
    self->kp = 10.0f;  /* proportional gain */
    self->ti = 1.0e15;  /* integration time constant */
    self->td = 0.0f;  /* derivative time constant */
    self->ts = 0.001f;  /* FINAL - 1 ms converted to units of seconds */
    self->alpha = 1.0f;  /* low-pass filter smoothing factor (alpha) */
    self->sp = 35.0f;  /* setpoint temperature in units of degC */
    self->pv = 0.0f;  /* process variable (measured temperature in units of degC) */
    self->deadband = 0.0f;  /* deadband */
    self->error = 0.0f;  /* difference between setpoint and process varible */
    self->error_last = 0.0f;
    self->error_last2 = 0.0f;
    self->error_lpf = 0.0f;
    self->error_lpf_last = 0.0f;
    self->error_lpf_last2 = 0.0f;
    self->control = 0.0f;  /* control function; output from the PID algorithm */
    self->control_limit_max = 0.0f;  /* maximum acceptable control output */
    self->control_limit_min = 0.0f;  /* minimum acceptable control output */
    return 0;
};


/**
 * @brief Reset the PID loop
 *
 * @param[in] self  An instance of pid_t
 * @return
 */
uint8_t pid_reset(pid_t* self) {
    self->error = 0.0f;  /* difference between setpoint and process varible */
    self->error_last = 0.0f;
    self->error_last2 = 0.0f;
    self->error_lpf = 0.0f;
    self->error_lpf_last = 0.0f;
    self->error_lpf_last2 = 0.0f;
    self->control = 0.0f;  /* control function; output from the PID algorithm */
    return 0;
}


/**
 * @brief Set the proportional gain, Kp
 *
 * @param[in] self  An instance of pid_t
 * @param[in] gain  proportional gain
 * @return
 */
uint8_t pid_set_proportional_gain(pid_t* self, float gain) {
    self->kp = gain;
    return 0;
};


/**
 * @brief Get the proportional gain, Kp
 *
 * @param[in]  self  An instance of pid_t
 * @param[out] gain  proportional gain
 * @return
 */
uint8_t pid_get_proportional_gain(pid_t* self, float* gain) {
    *gain = self->kp;
    return 0;
};


/**
 * @brief Set the integral gain, Ti
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   ti  integral time
 * @return
 */
uint8_t pid_set_integral_time(pid_t* self, float ti) {
    self->ti = ti;
    return 0;
};


/**
 * @brief Get the intregral gain, Ti
 *
 * @param[in] self  An instance of pid_t
 * @param[out]  ti  integral time
 * @return
 */
uint8_t pid_get_integral_time(pid_t* self, float* ti) {
    *ti = self->ti;
    return 0;
};


/**
 * @brief Set the derivative gain, Td
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   td  derivative time
 * @return
 */
uint8_t pid_set_derivative_time(pid_t* self, float td) {
    self->td = td;
    return 0;
};


/**
 * @brief Get the derivative gain, Td
 *
 * @param[in]  self  An instance of pid_t
 * @param[out]   td  derivative time
 * @return
 */
uint8_t pid_get_derivative_time(pid_t* self, float* td) {
    *td = self->td;
    return 0;
};


/**
 * @brief Set the sampling period in units of seconds
 *
 * @param[in] self  An instance of pid_t
 * @param[in] kp  proportional gain
 * @param[in] ti  integral gain
 * @param[in] td  derivate gain
 * @return
 */
uint8_t pid_set_gains(pid_t* self, float kp, float ti, float td) {
    self->kp = kp;
    self->ti = ti;
    self->td = td;
    return 0;
};


/**
 * @brief Set the sampling period in units of seconds
 *
 * @param[in] self  An instance of pid_t
 * @param[out] kp  proportional gain
 * @param[out] ti  integral gain
 * @param[out] td  derivate gain
 * @return
 */
uint8_t pid_get_gains(pid_t* self, float* kp, float* ti, float* td) {
    *kp = self->kp;
    *ti = self->ti;
    *td = self->td;
    return 0;
};


/**
 * @brief Set the sampling period in units of seconds
 *
 * @param[in] self  An instance of pid_t
 * @param[in] gain  the sampling period in units of seconds
 * @return
 */
uint8_t pid_set_sampling_period(pid_t* self, float period) {
    self->ts = period;
    return 0;
};


/**
 * @brief Get the sampling period in units of seconds
 *
 * @param[in]  self  An instance of pid_t
 * @param[out] gain  the sampling period in units of seconds
 * @return
 */
uint8_t pid_get_sampling_period(pid_t* self, float* period) {
    *period = self->ts;
    return 0;
};


/**
 * @brief Set the low-pass filter smoothing factor (alpha)
 *
 * @param[in]  self  An instance of pid_t
 * @param[in] alpha  the low-pass filter smoothing factor
 * @return
 */
uint8_t pid_set_smoothing_factor(pid_t* self, float alpha) {
    self->alpha = alpha;
    return 0;
};


/**
 * @brief Get the low-pass filter smoothing factor (alpha)
 *
 * @param[in]   self  An instance of pid_t
 * @param[out] alpha  the low-pass filter smoothing factor
 * @return
 */
uint8_t pid_get_smoothing_factor(pid_t* self, float* alpha) {
    *alpha = self->alpha;
    return 0;
};


/**
 * @brief Set the PID setpoint value
 *
 * @param[in] self  An instance of pid_t
 * @param[in]   sp  the PID setpoint value
 * @return
 */
uint8_t pid_set_setpoint(pid_t* self, float sp) {
    self->sp = sp;
    return 0;
};


/**
 * @brief Get the PID setpoint value
 *
 * @param[in] self  An instance of pid_t
 * @param[out]  sp  the PID setpoint value
 * @return
 */
uint8_t pid_get_setpoint(pid_t* self, float* sp) {
    *sp = self->sp;
    return 0;
};


/**
 * @brief Set the PID deadband value
 *
 * @param[in] self  An instance of pid_t
 * @param[in] deadband  the PID deadband value
 * @return
 */
uint8_t pid_set_deadband(pid_t* self, float deadband) {
    self->deadband = deadband;
    return 0;
}


/**
 * @brief Get the PID deadband value
 *
 * @param[in]      self  An instance of pid_t
 * @param[out] deadband  the PID deadband value
 * @return
 */
uint8_t pid_get_deadband(pid_t* self, float* deadband) {
    *deadband = self->deadband;
    return 0;
}


/**
 * @brief Set the PID control limits
 *
 * @param[in]   self  An instance of pid_t
 * @param[in] cl_max  the maximum PID control limit
 * @param[in] cl_min  the minimum PID control limit
 * @return
 */
uint8_t pid_set_control_limits(pid_t* self, float cl_max, float cl_min) {
    self->control_limit_max = cl_max;
    self->control_limit_min = cl_min;
    return 0;
}


/**
 * @brief Get the PID control limits
 *
 * @param[in]    self  An instance of pid_t
 * @param[out] cl_max  the maximum PID control limit
 * @param[out] cl_min  the minimum PID control limit
 * @return
 */
uint8_t pid_get_control_limits(pid_t* self, float* cl_max, float* cl_min) {
    *cl_max = self->control_limit_max;
    *cl_min = self->control_limit_min;
    return 0;
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
uint8_t pid_compute(pid_t* self, float pv, float* control) {
    float control_kp = 0.0f;
    float control_ti = 0.0f;
    float control_td = 0.0f;
    float error_magnitude = 0.0f;

    /* Compute the error value */
    self->error = self->sp - self->pv;
    // self->error = -(self->sp - pv);

    /* Apply a low-pass filter (LPF) to the error signal */
    self->error_lpf = self->alpha * self->error + (1 - self->alpha) * self->error_lpf_last;

    /* Compute the correction terms */
    control_kp = self->kp * (self->error - self->error_last);
    control_ti = (self->kp / self->ti) * (self->error * self->ts);
    control_td = (
        // (self->kp * self->td) * ( (self->error - 2 * self->error_last + self->error_last2) / self->ts )
        (self->kp * self->td) * ( (self->error_lpf - 2 * self->error_lpf_last + self->error_lpf_last2) / self->ts )
    );

    /* Compute Rest of PID Output */

    /**
     * Discrete PID so the previous control value is added to the new control value; therefore,
     * the control value is a running sum (TODO: confirm authenticity of this comment!!!)
     */
    self->control += control_kp + control_ti + control_td;

    /* Anti-wind-up via clamping */
    if (self->control > self->control_limit_max) {
        self->control = self->control_limit_max;
    } else if (self->control < self->control_limit_min) {
        self->control = self->control_limit_min;
    }

    /* Deadband logic */
    if (self->error < 0.0f) {
        error_magnitude = -1.0f * self->error;
    } else {
        error_magnitude = self->error;
    }

    if (error_magnitude < self->deadband) {
        *control = 0.0f;
    } else {
        *control = self->control;
    }

    /* Remember some variables for next time */
    self->error_last2 = self->error_last;
    self->error_last = self->error;
    self->error_lpf_last2 = self->error_lpf_last;
    self->error_lpf_last = self->error_lpf;

    return 0;
};


int main(int argc, char** argv) {
    pid_t pid;  /* implement global PID object */

    float kp = 10.0f;   /* define proportional gain */
    float ti = 1.0e15;  /* define integral time constant */
    float td = 0.0f;    /* define derivative time constant */

    pid_init(&pid);  /* initialize all members */
    pid_set_gains(&pid, kp, ti, td);  /* set all gains */

    return 0;
}
