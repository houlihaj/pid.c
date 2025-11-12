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


struct pid_t {
    float kp;  // proportional gain
    float ti;  // integral gain
    float td;  // derivate gain
    float ts;  // sampling period in units of seconds
    float alpha;  // low-pass filter smoothing factor (i.e. alpha)
    float sp;  // the setpoint value
    float pv;  // process variable (the measured value)
    float error;  // difference between setpoint and process varible
    float error_last;  // difference between setpoint and process varible
    float error_last2;  // difference between setpoint and process varible
    float error_lpf;  // difference between setpoint and process varible
    float error_lpf_last;  // difference between setpoint and process varible
    float error_lpf_last2;  // difference between setpoint and process varible
    float control;  // control function; output from the PID algorithm
    uint16_t control_max;  // maximum acceptable control output
    uint16_t control_min;  // minimum acceptable control output
};


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
    self->kp = 10.0;  /* proportional gain */
    self->ti = 1.0e15;  /* integration time constant */
    self->td = 0.0;  /* derivative time constant */
    self->ts = 0.001;  /* FINAL - 1 ms converted to units of seconds */
    self->alpha = 1.0;  /* low-pass filter smoothing factor (alpha) */
    self->sp = 35.0;  /* setpoint temperature in units of degC */
    self->pv = 0.0;  /* process variable (measured temperature in units of degC) */
    self->error = 0.0;  /* difference between setpoint and process varible */
    self->error_last = 0.0;  /* difference between setpoint and process varible */
    self->error_last2 = 0.0;  /* difference between setpoint and process varible */
    self->error_lpf = 0.0;  /* difference between setpoint and process varible */
    self->error_lpf_last = 0.0;  /* difference between setpoint and process varible */
    self->error_lpf_last2 = 0.0;  /* difference between setpoint and process varible */
    self->control = 0.0;  /* control function; output from the PID algorithm */
    self->control_max = 4095;  /* maximum acceptable control output */
    self->control_min = 0;  /* minimum acceptable control output */
    return 0;
};


/**
 * @brief Set the proportional gain, Kp
 *
 * @param self  An instance of pid_t
 * @param gain[in]  proportional gain
 * @return
 */
uint8_t pid_set_proportional_gain(pid_t* self, float gain) {
    self->kp = gain;
    return 0;
};


/**
 * @brief Get the proportional gain, Kp
 *
 * @param self  An instance of pid_t
 * @param gain[out]  proportional gain
 * @return
 */
uint8_t pid_get_proportional_gain(pid_t* self, float* gain) {
    *gain = self->kp;
    return 0;
};


/**
 * @brief Set the integral gain, Ti
 *
 * @param self  An instance of pid_t
 * @param gain[in]  integral gain
 * @return
 */
uint8_t pid_set_integral_gain(pid_t* self, float gain) {
    self->ti = gain;
    return 0;
};


/**
 * @brief Get the intregral gain, Ti
 *
 * @param self  An instance of pid_t
 * @param gain[out]  integral gain
 * @return
 */
uint8_t pid_get_integral_gain(pid_t* self, float* gain) {
    *gain = self->ti;
    return 0;
};


/**
 * @brief Set the derivative gain, Td
 *
 * @param self  An instance of pid_t
 * @param gain[in]  derivative gain
 * @return
 */
uint8_t pid_set_derivative_gain(pid_t* self, float gain) {
    self->td = gain;
    return 0;
};


/**
 * @brief Get the derivative gain, Td
 *
 * @param self  An instance of pid_t
 * @param gain[out]  derivative gain
 * @return
 */
uint8_t pid_get_derivative_gain(pid_t* self, float* gain) {
    *gain = self->td;
    return 0;
};


/**
 * @brief Set the sampling period in units of seconds
 *
 * @param self  An instance of pid_t
 * @param kp  proportional gain
 * @param ti  integral gain
 * @param td  derivate gain
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
 * @param self  An instance of pid_t
 * @param kp[out]  proportional gain
 * @param ti[out]  integral gain
 * @param td[out]  derivate gain
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
 * @param self  An instance of pid_t
 * @param gain[in]  the sampling period in units of seconds
 * @return
 */
uint8_t pid_set_sampling_period(pid_t* self, float period) {
    self->ts = period;
    return 0;
};


/**
 * @brief Get the sampling period in units of seconds
 *
 * @param self  An instance of pid_t
 * @param gain[out]  the sampling period in units of seconds
 * @return
 */
uint8_t pid_get_sampling_period(pid_t* self, float* period) {
    *period = self->ts;
    return 0;
};


/**
 * @brief Set the low-pass filter smoothing factor (alpha)
 *
 * @param self  An instance of pid_t
 * @param alpha[in]  the low-pass filter smoothing factor
 * @return
 */
uint8_t pid_set_smoothing_factor(pid_t* self, float alpha) {
    self->alpha = alpha;
    return 0;
};


/**
 * @brief Get the low-pass filter smoothing factor (alpha)
 *
 * @param self  An instance of pid_t
 * @param alpha[out]  the low-pass filter smoothing factor
 * @return
 */
uint8_t pid_get_smoothing_factor(pid_t* self, float* alpha) {
    *alpha = self->alpha;
    return 0;
};


/**
 * @brief Set the PID setpoint value
 *
 * @param self  An instance of pid_t
 * @param sp[in]  the PID setpoint value
 * @return
 */
uint8_t pid_set_setpoint(pid_t* self, float sp) {
    self->sp = sp;
    return 0;
};


/**
 * @brief Get the PID setpoint value
 *
 * @param self  An instance of pid_t
 * @param sp[out]  the PID setpoint value
 * @return
 */
uint8_t pid_get_setpoint(pid_t* self, float* sp) {
    *sp = self->sp;
    return 0;
};


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
 * @param self  An instance of pid_t
 * @return
 */
uint8_t pid_compute(pid_t* self) {
    float control_kp;
    float control_ti;
    float control_td;

    /* Compute the error value */
    self->error = self->sp - self->pv;

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

    /* Discrete PID so the previous control value is added to the new control value; therefore,
    the control value is a running sum (TODO: confirm authenticity of this comment!!!) */
    self->control += control_kp + control_ti + control_td;

    /* Anti-wind-up via clamping */
    if (self->control > self->control_max) {
        self->control = self->control_max;
    } else if (self->control < self->control_min) {
        self->control = self->control_min;
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

    float kp = 10.0;  /* define proportional gain */
    float ti = 1.0e15;  /* define integral time constant */
    float td = 0.0;  /* define derivative time constant */

    pid_init(&pid);  /* initialize all members */
    pid_set_gains(&pid, kp, ti, td);  /* set all gains */

    return 0;
}
