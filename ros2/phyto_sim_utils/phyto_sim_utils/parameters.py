import re

from rclpy.parameter import Parameter


def declare_number(node, name, default=Parameter.Type.DOUBLE):
    # YAML represents whole numbers and decimals as distinct ROS parameter types.
    override = node._parameter_overrides.get(name)
    if override is not None and override.type_ in (Parameter.Type.INTEGER, Parameter.Type.DOUBLE):
        return node.declare_parameter(name, override.type_).value
    return node.declare_parameter(name, default).value


def declare_matching_parameters(node, pattern, *types):
    # Humble has no public method to enumerate undeclared deployment-specific names.
    for name, override in node._parameter_overrides.items():
        if re.fullmatch(pattern, name):
            if not types and override.type_ == Parameter.Type.NOT_SET:
                # Humble reads an empty YAML sequence as an untyped override
                node.declare_parameter(name, [], ignore_override=True)
                continue
            kind = override.type_ if not types or override.type_ in types else types[0]
            node.declare_parameter(name, kind)
