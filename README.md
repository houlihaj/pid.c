# pid.c
Standard form discrete PID controller implementation using the incremental (velocity) algorithm for embedded systems in C

## Summary
This package contains files (pid.h and pid.c) to implement a simple PID controller.

## Features

- Full PID implementation using the standard form (i.e. Kp, Ti, Td)
- Derivative kick attenuation via applying a low-pass filter to the error signal
- Control signal anti-windup
- Statically allocated memory

## Integration details

- Integrate pid.h and pid.c files into your project.
- Include the pid.h header file in your code like below.

```c
#include "pid.h"
```

## File information

- pid.h : This header file contains the definitions of the user API.
- pid.c : This source file contains the implementation of the PID.

## References
- [Discrete-time PID controller](https://techteach.no/fag/process_control_nmbu_2018/book_pid_control/pidcontrol.pdf)
- [PID Control](https://www.cds.caltech.edu/~murray/courses/cds101/fa02/caltech/astrom-ch6.pdf)
- [Review of PID control design and tuning methods](https://iopscience.iop.org/article/10.1088/1742-6596/2649/1/012009/pdf)
- [Lecture 8: From Analog to Digital Controllers, PID Control](https://archive.control.lth.se/media/Education/EngineeringProgram/FRTN01/2014/L8_14.pdf)
- [PID Controllers](https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/PID-Controllers.html)
- [Arduino-PID-Library](https://github.com/br3ttb/Arduino-PID-Library)
- [The practical aspects of PID control](https://link.springer.com/chapter/10.1007/978-1-4039-1457-6_18)

