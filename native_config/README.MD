# Assumption
The mavlink is spawned in ros namespace `/uav1/mavros/`.
Everything noted about ID need to be changed for changing ID other than 1.

# File functions
- px4.config -------- The configuration file for PX4 Firmware(Linux appl)
- px4.launch -------- The launch file for mavros (with setting depend on px4.config)

# NOTE
So far only px4.config and px4.launch is modified to suit our use case.
