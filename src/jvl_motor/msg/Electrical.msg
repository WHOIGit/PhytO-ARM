# Electrical
# This message contains information read and decoded from JVL's registers.
#
# Due to the range of register addresses, these values cannot be read
# simultaneously. Time information in the headers is approximate.

#####################################################################

# The standard 2-part DsHeader block
# This allows both a standard ROS header and DS-specific header blocks
# See HEADERS.md in ds_core_msgs for details
std_msgs/Header header
ds_core_msgs/DsHeader ds_header

#####################################################################

uint32 u_24v  # control voltage (engineering units)
float32 u_bus  # voltage of internal DC bus (V)
