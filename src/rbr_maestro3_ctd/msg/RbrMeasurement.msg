# RbrMeasurement

#####################################################################

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

#####################################################################

string channel

float32 value  # assuming it is always a float?
string unit
