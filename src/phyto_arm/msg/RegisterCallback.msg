# RegisterCallback
# Used for registering service callbacks at runtime

# Callback for determining whether instrument is busy
string busy_callback
# Callback for retrieving goal depth
string goal_depth_callback
# Callback for running instrument payload
string run_instrument_callback
# Callback for winch movement action
string move_winch_callback

---
bool success
string message
