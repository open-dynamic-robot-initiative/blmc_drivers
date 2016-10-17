Brushless Motor Control Real-Time CAN API
=========================================

This package provides a real-time API for communication with the motor control
board and the OptoForce sensor via CAN.

A description of the CAN protocol that is implemented by this library can be
found on Confluence:
https://atlas.is.localnet/confluence/display/AMDW/BLMC+-+CAN+Interface

This package contains a executable blmc_can_demo, that shows how to use the
library.


Requirements
------------

A real-time CAN driver has to be loaded and the interface to be set up.  This
can be done with the script `start_rtcan` that is included in the xenomai-core
repository (https://git-amd.tuebingen.mpg.de/amd-clmc/xenomai-core).

Note that the board has to run a program that implements the same CAN protocol.
At the time of writing this README, this is only the case for
dual_motor_torque_ctrl.

While the library has no direct dependency on SL, the LAB.cmake of
sl_cmake_global is included. Therefore this package is required.
(TODO: This should be changed. Make the package independent of SL.)
