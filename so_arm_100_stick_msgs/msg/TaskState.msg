# Published on a latched topic (/stick_task/state) after every state
# transition, so a client (or the Blender bridge, if one is ever built) can
# learn the current state without polling
# (ROS2_IMPLEMENTATION_PLAN.md Sec 6).

string state                     # IDLE, PICKING, HOLDING, PLACING, AT_PLACE, ERROR
string current_stick_id          # empty when not holding/placing a stick
float64 current_stick_length_m
string message                   # human-readable detail, e.g. an ERROR reason
