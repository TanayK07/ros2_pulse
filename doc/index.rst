ros2_pulse
==========

Per-topic message rates and node liveness for a running ROS 2 system, inter-process and
intra-process, measured inside each process on stock binaries. No rebuild, no subscriber, no
privileges and no network traffic.

This guide describes 0.7.0. Parts marked "Added in version 0.7.0" are not in the 0.6.0 binaries.

.. image:: images/pulse-top-demo.gif
   :alt: pulse-top reading the probe's log, catching a /scan stall and a /cmd_vel rate sag

Why
---

``ros2 topic hz`` answers "how fast is this topic" one topic at a time, by subscribing to it.
That costs something: one ``hz`` on a 100 KB topic at 50 Hz used 7 % of a core in our
measurements, ``ros2 topic echo`` used 31 %, and pointing ``hz`` at an intra-process topic made
the publishing process serialize every message (+52 % CPU). rclcpp's built-in topic statistics
never saw intra-process traffic on Humble and are removed in Rolling.

ros2_pulse hooks the tracetools functions rclcpp already calls on every publish and every
callback, counts in-process, and writes ready-to-read Hz to a small file every few seconds:
every topic and every node of every probed process, for about 2 % CPU on a deliberately harsh
4,900 msg/s stress and less on real graphs.

Install
-------

.. code-block:: bash

   sudo apt install ros-$ROS_DISTRO-ros2-pulse   # Humble, Jazzy, Kilted
   pip3 install ros2-pulse-top                   # optional: pulse-top and pulse-export

Try it
------

.. code-block:: bash

   export LD_PRELOAD=libros2_pulse.so
   ros2 run demo_nodes_cpp talker &
   ros2 run demo_nodes_cpp listener &
   sleep 11; cat ${TMPDIR:-/tmp}/topic_freq.*.log

Each probed process writes its own file, one block per 5 s window:

.. code-block:: text

   # ts_ns=1782887153899445923 window_s=5.000
   TOPIC /chatter 1.000000
   NODE /talker
   # ts_ns=1782887154012873310 window_s=5.000
   RECV /chatter inter=1.000000 intra=0.000000
   NODE /listener

That is the whole setup. The :doc:`quickstart` goes through it with a launch file, pulse-top
and an expected-rate spec.

What you can do with it
-----------------------

.. list-table::
   :header-rows: 1
   :widths: 60 40

   * - Goal
     - Use
   * - See every topic's rate live, over ssh
     - :doc:`pulse-top`
   * - Get warned when a topic is too slow, too fast or frozen, or a node is gone
     - :doc:`alerting`
   * - Fail a CI run or trip a watchdog on the same rules
     - :ref:`pulse-check <pulse-check>`
   * - Keep ``/statistics`` consumers working now that rclcpp removed it
     - :doc:`pulse-bridge`
   * - Feed ``diagnostic_aggregator`` or graph-monitor
     - :ref:`/diagnostics <bridge-diagnostics>`, :ref:`graph-monitor <bridge-graph-monitor>`
   * - Put the robot on a Grafana dashboard
     - :doc:`export`

.. toctree::
   :caption: User guide
   :hidden:
   :maxdepth: 2

   quickstart
   configuration
   output
   alerting
   pulse-top
   pulse-bridge
   export

.. toctree::
   :caption: Background
   :hidden:
   :maxdepth: 2

   how-it-works
   overhead
   compatibility

.. toctree::
   :caption: Project
   :hidden:
   :maxdepth: 1
   :glob:

   Changelog <__CHANGELOG>
   Contributing <__CONTRIBUTING>
   License <__LICENSE>
   __links
   __ros_package_dependencies

.. toctree::
   :hidden:

   genindex
