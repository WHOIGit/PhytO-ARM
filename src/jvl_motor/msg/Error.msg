# Error
# Contains the value of the ERR_STAT register.

#####################################################################

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

#####################################################################

uint32 err_stat
uint32 change_mask  # previous (xor) current
