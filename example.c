/* Name: example.c */
/* Description: */
/* Author: John Houlihan */

/**
* MIT License
*
* Copyright (c) 2026 John Houlihan
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


int main(int argc, char** argv)
{
    pid_t pid;  /* implement global PID object */

    pid_real_t kp = 10.0;  /* define proportional gain */
    pid_real_t ti = 0.0;   /* define integral time constant */
    pid_real_t td = 0.0;   /* define derivative time constant */

    pid_real_t setpiont_temperature_degc = 25.0;
    pid_real_t measured_temperature_degc_placeholder = 24.9;

    pid_real_t control = 0.0;

    pid_init(&pid);                   /* initialize all members */
    pid_set_gains(&pid, kp, ti, td);  /* set all gains */
    pid_set_setpoint(&pid, setpiont_temperature_degc);

    while (1)
    {
        // Implement your get measured temperature functionality here
        // Example:
        // float measured_temperature_degc = thermometer_get_temperature();

        // Replace 'measured_temperature_degc_placeholder' with 'measured_temperature_degc'
        pid_compute(&pid, measured_temperature_degc_placeholder, &control);

        // Use 'control' to apply the correction to your control varible (i.e. TEC current, motor voltage, etc.)
        // Example:
        // float tec_current_last = 1.0;                    // let's say the last TEC supply current was 1.0 Amps
        // float tec_current = tec_current_last + control;  // apply the PID computed correction (i.e. control)
        // tec_current_last = tec_current;                  // save for the next iteration
    }

    return 0;
}
