from rclpy.parameter import Parameter


def declare_number(node, name, default=Parameter.Type.DOUBLE):
    # YAML represents whole numbers and decimals as distinct ROS parameter types.
    override = node._parameter_overrides.get(name)
    if override is not None and override.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
        return node.declare_parameter(name, override.type_).value
    return node.declare_parameter(name, default).value
