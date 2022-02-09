# Motion
# This message contains information read and decoded from JVL's registers. 

#####################################################################

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

#####################################################################

int8 MODE_PASSIVE = 0
int8 MODE_VELOCITY = 1
int8 MODE_POSITION = 2

int8 mode

int32 position
float32 velocity

# TODO: FLWERR
