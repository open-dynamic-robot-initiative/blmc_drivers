Brushless Motor Control Real-Time CAN API
=========================================

This package provides a real-time API for communication with the motor control
board and the OptoForce sensor via CAN.

A description of the CAN protocol that is implemented by this library can be
found in the documentation of the
[mw_dual_motor_torque_ctrl](https://github.com/open-dynamic-robot-initiative/mw_dual_motor_torque_ctrl)
firmware.

This package contains a executable blmc_can_demo, that shows how to use the
library.


Dependencies and other Requirements
-----------------------------------

- Supports Ubuntu patched with PREEMPT_RT patched kernel (see documentation on
  [how to install the PREEMPT_RT
  kernel](http://people.tuebingen.mpg.de/mpi-is-software/robotfingers/docs/robot_interfaces/doc/real_time.html)).
- Experimental suport for Mac OS and Ubuntu patched with Xenomai

See also the [real_time_tools](https://github.com/machines-in-motion/real_time_tools) package.

A real-time CAN driver has to be loaded and the interface to be set up.  This
can be done with the script `start_rtcan` that is included in the [xenomai-core
repository](https://git-amd.tuebingen.mpg.de/amd-clmc/xenomai-core).
With PREEMPT_RT we also made good experience using the default driver, though.

Note that the board has to run a program that implements the same CAN protocol.
At the time of writing this README, this is only the case for
[mw_dual_motor_torque_ctrl](https://github.com/open-dynamic-robot-initiative/mw_dual_motor_torque_ctrl).


Links
-----

- [Documentation](http://people.tuebingen.mpg.de/mpi-is-software/robotfingers/docs/blmc_drivers/index.html)
- [Source Code](https://github.com/open-dynamic-robot-initiative/blmc_drivers)
- [Bug Tracker](https://github.com/open-dynamic-robot-initiative/blmc_drivers/issues)



Authors
-------

- Felix Widmaier
- Manuel Wuethrich
- Maximilien Naveau
- Steve Heim
- Diego Agudelo
- Julian Viereck


License
-------

BSD 3-Clause License

Copyright(c) 2018 Max Planck Gesellschaft, New York University
