Brushless Motor Control Real-Time CAN API
=========================================

This package provides a real-time API for communication with the motor control
board and the OptoForce sensor via CAN.

A description of the CAN protocol that is implemented by this library can be
found on Confluence:
https://atlas.is.localnet/confluence/display/AMDW/BLMC+-+CAN+Interface

This package contains a executable blmc_can_demo, that shows how to use the
library.


Dependencies and other Requirements
-----------------------------------

Requires Xenomai and the
[xenomai_ros](https://git-amd.tuebingen.mpg.de/amd-clmc/xenomai-ros) package.

A real-time CAN driver has to be loaded and the interface to be set up.  This
can be done with the script `start_rtcan` that is included in the xenomai-core
repository (https://git-amd.tuebingen.mpg.de/amd-clmc/xenomai-core).

Note that the board has to run a program that implements the same CAN protocol.
At the time of writing this README, this is only the case for
dual_motor_torque_ctrl.


Documentation
-------------

The code is well documented with Doxygen comments.  To generate a HTML
documentation, simply run the command

    doxygen

in the root directory.  The files are written to the subdirectory `docs` (which
will be created if it does not exist already).
