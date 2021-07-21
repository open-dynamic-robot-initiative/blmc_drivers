Build Instructions
==================

Dependencies
------------

We are using colcon_ as build tool and typically use ``ros2 run`` to execute our
applications.  While we are not really depending on any ROS_ packages, this
means a basic ROS 2 installation is recommended.

We are testing on Ubuntu 20.04 with ROS Foxy.  Other versions may work as well
but are not officially supported.


Get the Source
--------------

**blmc_drivers** depends on several other of our packages which are
organized in separate repositories.  We therefore use a workspace management
tool called treep_ which allows easy cloning of multi-repository projects.

treep can be installed via pip::

    pip install treep

Clone the treep configuration containing the "BLMC_DRIVERS" project::

    git clone git@github.com:machines-in-motion/treep_machines_in_motion.git

**Note:**  treep searches for a configuration directory from the current working
directory upwards.  So you can use treep in the directory in which you invoked
the ``git clone`` command above or any subdirectory.

Now clone the project (including packages for communication with a master
board)::

    treep --clone BLMC_DRIVERS

or, if you are only interested in CAN communication::

    treep --clone BLMC_DRIVERS_CAN_ONLY

**Important:** treep uses SSH to clone from github.  So for the above command to
work, you need a github account with a registered SSH key.  Further this key
needs to work without asking for a password everytime.  To achieve this, run

::

    ssh-add

first.


Build
-----

To build, cd into the ``workspace`` directory that was created by treep.  If not
done already, source the ROS setup::

    source /opt/ros/foxy/setup.bash

Then build with::

    colcon build


Build Documentation
~~~~~~~~~~~~~~~~~~~

If you want to build this documentation locally, run

::

    colcon build --cmake-args -DGENERATE_DOCUMENTATION=ON


The documentation can then be found in
``<workspace>/install/<package_name>/share/<package_name>/docs``


.. _colcon: https://colcon.readthedocs.io/en/released/index.html
.. _ROS: https://www.ros.org
.. _treep: https://pypi.org/project/treep/
