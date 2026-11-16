# Historical experiments

Demo0806 and Demo0810 contain old controller/IP calls, ROS dependencies and recordings
from a different experimental setup. They are retained as historical data and are
not RDK 1.9.4.1 entry points. TODO/ is likewise legacy work-in-progress code.

Use ../Examples for the maintained Rizon 4 interface. To port a historical application,
replace the constructor with the robot serial number, migrate direct SDK calls as
listed in ../docs/SDK_MIGRATION.md. Pass recorded SDK poses directly as
[x,y,z,qw,qx,qy,qz] lists (wxyz); inputs and outputs use the same order.
Recheck tool/world calibration before replaying any recording.
