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

#include <stdint.h>

typedef struct pid_t pid_t;

uint8_t pid_init(pid_t* self);
uint8_t pid_set_proportional_gain(pid_t* self, float gain);
uint8_t pid_get_proportional_gain(pid_t* self, float* gain);
uint8_t pid_set_integral_gain(pid_t* self, float gain);
uint8_t pid_get_integral_gain(pid_t* self, float* gain);
uint8_t pid_set_derivative_gain(pid_t* self, float gain);
uint8_t pid_get_derivative_gain(pid_t* self, float* gain);
uint8_t pid_set_gains(pid_t* self, float kp, float ti, float td);
uint8_t pid_get_gains(pid_t* self, float* kp, float* ti, float* td);
uint8_t pid_set_sampling_period(pid_t* self, float period);
uint8_t pid_get_sampling_period(pid_t* self, float* period);
uint8_t pid_set_smoothing_factor(pid_t* self, float alpha);
uint8_t pid_get_smoothing_factor(pid_t* self, float* alpha);
uint8_t pid_set_setpoint(pid_t* self, float sp);
uint8_t pid_get_setpoint(pid_t* self, float* sp);
uint8_t pid_compute(pid_t* self);

#ifdef __cplusplus
}  /* End of CPP guard */
#endif

#endif  // PID_H
