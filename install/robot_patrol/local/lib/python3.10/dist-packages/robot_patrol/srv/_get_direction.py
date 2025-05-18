# generated from rosidl_generator_py/resource/_idl.py.em
# with input from robot_patrol:srv/GetDirection.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetDirection_Request(type):
    """Metaclass of message 'GetDirection_Request'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_patrol')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_patrol.srv.GetDirection_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_direction__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_direction__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_direction__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_direction__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_direction__request

            from sensor_msgs.msg import LaserScan
            if LaserScan.__class__._TYPE_SUPPORT is None:
                LaserScan.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetDirection_Request(metaclass=Metaclass_GetDirection_Request):
    """Message class 'GetDirection_Request'."""

    __slots__ = [
        '_laser_data',
    ]

    _fields_and_field_types = {
        'laser_data': 'sensor_msgs/LaserScan',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'LaserScan'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import LaserScan
        self.laser_data = kwargs.get('laser_data', LaserScan())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.laser_data != other.laser_data:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def laser_data(self):
        """Message field 'laser_data'."""
        return self._laser_data

    @laser_data.setter
    def laser_data(self, value):
        if __debug__:
            from sensor_msgs.msg import LaserScan
            assert \
                isinstance(value, LaserScan), \
                "The 'laser_data' field must be a sub message of type 'LaserScan'"
        self._laser_data = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_GetDirection_Response(type):
    """Metaclass of message 'GetDirection_Response'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_patrol')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_patrol.srv.GetDirection_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_direction__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_direction__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_direction__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_direction__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_direction__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetDirection_Response(metaclass=Metaclass_GetDirection_Response):
    """Message class 'GetDirection_Response'."""

    __slots__ = [
        '_direction',
    ]

    _fields_and_field_types = {
        'direction': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.direction = kwargs.get('direction', str())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.direction != other.direction:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def direction(self):
        """Message field 'direction'."""
        return self._direction

    @direction.setter
    def direction(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'direction' field must be of type 'str'"
        self._direction = value


class Metaclass_GetDirection(type):
    """Metaclass of service 'GetDirection'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('robot_patrol')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'robot_patrol.srv.GetDirection')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_direction

            from robot_patrol.srv import _get_direction
            if _get_direction.Metaclass_GetDirection_Request._TYPE_SUPPORT is None:
                _get_direction.Metaclass_GetDirection_Request.__import_type_support__()
            if _get_direction.Metaclass_GetDirection_Response._TYPE_SUPPORT is None:
                _get_direction.Metaclass_GetDirection_Response.__import_type_support__()


class GetDirection(metaclass=Metaclass_GetDirection):
    from robot_patrol.srv._get_direction import GetDirection_Request as Request
    from robot_patrol.srv._get_direction import GetDirection_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
