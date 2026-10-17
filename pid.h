/* Name: pid.h */
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
#ifndef PID_H
#define PID_H

/*! CPP guard */
#ifdef __cplusplus
extern "C" {
#endif


typedef float pid_real_t;

typedef struct pid_t pid_t;

typedef enum {
    PID_SUCCESS = 0,
    PID_FAILURE,
    PID_MEASURED_ERROR_WITHIN_DEADBAND,  /* the measured error is within the deadband */
    PID_DIVIDE_BY_ZERO_ENCOUNTERED
} pid_status_t;

typedef enum {
    PID_POLARITY_DIRECT = 0,
    PID_POLARITY_REVERSE
} pid_polarity_t;

struct pid_t {
    pid_real_t kp;  /* proportional gain */
    pid_real_t ti;  /* integral gain */
    pid_real_t td;  /* derivate gain */
    pid_real_t ts;  /* sampling period in units of seconds */
    pid_real_t alpha;  /* low-pass filter smoothing factor (i.e. alpha) */
    pid_real_t sp;  /* the setpoint value */
    pid_real_t pv;  /* process variable (the measured value) */
    pid_real_t deadband;  /* deadband */
    pid_real_t control_limit_max;  /* maximum acceptable control output */
    pid_real_t control_limit_min;  /* minimum acceptable control output */
    pid_polarity_t polarity;
    pid_real_t error;  /* difference between setpoint and process varible */
    pid_real_t error_last;  /* difference between setpoint and process varible */
    pid_real_t error_last2;  /* difference between setpoint and process varible */
    pid_real_t error_lpf;  /* difference between setpoint and process varible */
    pid_real_t error_lpf_last;  /* difference between setpoint and process varible */
    pid_real_t error_lpf_last2;  /* difference between setpoint and process varible */
    pid_real_t control;  /* control function; output from the PID algorithm */
};

pid_status_t pid_init(pid_t* self);
pid_status_t pid_reset(pid_t* self);
pid_status_t pid_set_proportional_gain(pid_t* self, pid_real_t gain);
pid_status_t pid_get_proportional_gain(pid_t* self, pid_real_t* gain);
pid_status_t pid_set_integral_time(pid_t* self, pid_real_t ti);
pid_status_t pid_get_integral_time(pid_t* self, pid_real_t* ti);
pid_status_t pid_set_derivative_time(pid_t* self, pid_real_t td);
pid_status_t pid_get_derivative_time(pid_t* self, pid_real_t* td);
pid_status_t pid_set_gains(pid_t* self, pid_real_t kp, pid_real_t ti, pid_real_t td);
pid_status_t pid_get_gains(pid_t* self, pid_real_t* kp, pid_real_t* ti, pid_real_t* td);
pid_status_t pid_set_sampling_period(pid_t* self, pid_real_t period);
pid_status_t pid_get_sampling_period(pid_t* self, pid_real_t* period);
pid_status_t pid_set_smoothing_factor(pid_t* self, pid_real_t alpha);
pid_status_t pid_get_smoothing_factor(pid_t* self, pid_real_t* alpha);
pid_status_t pid_set_setpoint(pid_t* self, pid_real_t sp);
pid_status_t pid_get_setpoint(pid_t* self, pid_real_t* sp);
pid_status_t pid_set_deadband(pid_t* self, pid_real_t deadband);
pid_status_t pid_get_deadband(pid_t* self, pid_real_t* deadband);
pid_status_t pid_set_control_limits(pid_t* self, pid_real_t cl_max, pid_real_t cl_min);
pid_status_t pid_get_control_limits(pid_t* self, pid_real_t* cl_max, pid_real_t* cl_min);
pid_status_t pid_set_polarity(pid_t* self, pid_polarity_t polarity);
pid_status_t pid_get_polarity(pid_t* self, pid_polarity_t* polarity);
pid_status_t pid_compute(pid_t* self, pid_real_t pv, pid_real_t* control);

#ifdef __cplusplus
}  /* End of CPP guard */
#endif

#endif  // PID_H
