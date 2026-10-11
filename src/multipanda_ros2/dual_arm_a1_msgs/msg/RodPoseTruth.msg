# A1 rod pose truth (P1b).
#
# One structured message carrying the run identity AND the settled rod pose
# together, so the two can never be paired across runs. Two separate latched
# topics would not be atomically pairable.
#
# Published only by the A1 scene runner, only after the pose has settled and
# been read back, and only once per run. Never published on /detected_rod_pose.

std_msgs/Header header
    # frame_id: the frame `pose` is expressed in (as returned by the sim).
    # stamp:    publisher-local ROS time at publication. The simulation's own
    #           service returns a zero stamp, so the receiver must NOT treat
    #           this stamp as a freshness measure; it correlates by run_id and
    #           stamps the arrival itself.

string run_id
    # Identifies the run that produced this pose. A receiver must reject a
    # payload whose run_id does not match the run it was started for.

geometry_msgs/Pose pose
    # Rod body pose: position is the box centre, orientation is the rod body
    # axes. The rod's long axis is its local Y.

geometry_msgs/Vector3 rod_size
    # Full box size in metres (x = 0.04, y = 0.40, z = 0.04). Usable directly as
    # the BOX dimensions of the corresponding planning-scene collision object.

float64 mass_kg
    # Rod mass reported by the simulation for this run (model record).
