// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from robot_patrol:srv/GetDirection.idl
// generated code does not contain a copyright notice
#include "robot_patrol/srv/detail/get_direction__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `laser_data`
#include "sensor_msgs/msg/detail/laser_scan__functions.h"

bool
robot_patrol__srv__GetDirection_Request__init(robot_patrol__srv__GetDirection_Request * msg)
{
  if (!msg) {
    return false;
  }
  // laser_data
  if (!sensor_msgs__msg__LaserScan__init(&msg->laser_data)) {
    robot_patrol__srv__GetDirection_Request__fini(msg);
    return false;
  }
  return true;
}

void
robot_patrol__srv__GetDirection_Request__fini(robot_patrol__srv__GetDirection_Request * msg)
{
  if (!msg) {
    return;
  }
  // laser_data
  sensor_msgs__msg__LaserScan__fini(&msg->laser_data);
}

bool
robot_patrol__srv__GetDirection_Request__are_equal(const robot_patrol__srv__GetDirection_Request * lhs, const robot_patrol__srv__GetDirection_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // laser_data
  if (!sensor_msgs__msg__LaserScan__are_equal(
      &(lhs->laser_data), &(rhs->laser_data)))
  {
    return false;
  }
  return true;
}

bool
robot_patrol__srv__GetDirection_Request__copy(
  const robot_patrol__srv__GetDirection_Request * input,
  robot_patrol__srv__GetDirection_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // laser_data
  if (!sensor_msgs__msg__LaserScan__copy(
      &(input->laser_data), &(output->laser_data)))
  {
    return false;
  }
  return true;
}

robot_patrol__srv__GetDirection_Request *
robot_patrol__srv__GetDirection_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_patrol__srv__GetDirection_Request * msg = (robot_patrol__srv__GetDirection_Request *)allocator.allocate(sizeof(robot_patrol__srv__GetDirection_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_patrol__srv__GetDirection_Request));
  bool success = robot_patrol__srv__GetDirection_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_patrol__srv__GetDirection_Request__destroy(robot_patrol__srv__GetDirection_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_patrol__srv__GetDirection_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_patrol__srv__GetDirection_Request__Sequence__init(robot_patrol__srv__GetDirection_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_patrol__srv__GetDirection_Request * data = NULL;

  if (size) {
    data = (robot_patrol__srv__GetDirection_Request *)allocator.zero_allocate(size, sizeof(robot_patrol__srv__GetDirection_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_patrol__srv__GetDirection_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_patrol__srv__GetDirection_Request__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_patrol__srv__GetDirection_Request__Sequence__fini(robot_patrol__srv__GetDirection_Request__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_patrol__srv__GetDirection_Request__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_patrol__srv__GetDirection_Request__Sequence *
robot_patrol__srv__GetDirection_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_patrol__srv__GetDirection_Request__Sequence * array = (robot_patrol__srv__GetDirection_Request__Sequence *)allocator.allocate(sizeof(robot_patrol__srv__GetDirection_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_patrol__srv__GetDirection_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_patrol__srv__GetDirection_Request__Sequence__destroy(robot_patrol__srv__GetDirection_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_patrol__srv__GetDirection_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_patrol__srv__GetDirection_Request__Sequence__are_equal(const robot_patrol__srv__GetDirection_Request__Sequence * lhs, const robot_patrol__srv__GetDirection_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_patrol__srv__GetDirection_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_patrol__srv__GetDirection_Request__Sequence__copy(
  const robot_patrol__srv__GetDirection_Request__Sequence * input,
  robot_patrol__srv__GetDirection_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_patrol__srv__GetDirection_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_patrol__srv__GetDirection_Request * data =
      (robot_patrol__srv__GetDirection_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_patrol__srv__GetDirection_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_patrol__srv__GetDirection_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_patrol__srv__GetDirection_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `direction`
#include "rosidl_runtime_c/string_functions.h"

bool
robot_patrol__srv__GetDirection_Response__init(robot_patrol__srv__GetDirection_Response * msg)
{
  if (!msg) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__init(&msg->direction)) {
    robot_patrol__srv__GetDirection_Response__fini(msg);
    return false;
  }
  return true;
}

void
robot_patrol__srv__GetDirection_Response__fini(robot_patrol__srv__GetDirection_Response * msg)
{
  if (!msg) {
    return;
  }
  // direction
  rosidl_runtime_c__String__fini(&msg->direction);
}

bool
robot_patrol__srv__GetDirection_Response__are_equal(const robot_patrol__srv__GetDirection_Response * lhs, const robot_patrol__srv__GetDirection_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->direction), &(rhs->direction)))
  {
    return false;
  }
  return true;
}

bool
robot_patrol__srv__GetDirection_Response__copy(
  const robot_patrol__srv__GetDirection_Response * input,
  robot_patrol__srv__GetDirection_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // direction
  if (!rosidl_runtime_c__String__copy(
      &(input->direction), &(output->direction)))
  {
    return false;
  }
  return true;
}

robot_patrol__srv__GetDirection_Response *
robot_patrol__srv__GetDirection_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_patrol__srv__GetDirection_Response * msg = (robot_patrol__srv__GetDirection_Response *)allocator.allocate(sizeof(robot_patrol__srv__GetDirection_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(robot_patrol__srv__GetDirection_Response));
  bool success = robot_patrol__srv__GetDirection_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
robot_patrol__srv__GetDirection_Response__destroy(robot_patrol__srv__GetDirection_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    robot_patrol__srv__GetDirection_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
robot_patrol__srv__GetDirection_Response__Sequence__init(robot_patrol__srv__GetDirection_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_patrol__srv__GetDirection_Response * data = NULL;

  if (size) {
    data = (robot_patrol__srv__GetDirection_Response *)allocator.zero_allocate(size, sizeof(robot_patrol__srv__GetDirection_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = robot_patrol__srv__GetDirection_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        robot_patrol__srv__GetDirection_Response__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
robot_patrol__srv__GetDirection_Response__Sequence__fini(robot_patrol__srv__GetDirection_Response__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      robot_patrol__srv__GetDirection_Response__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

robot_patrol__srv__GetDirection_Response__Sequence *
robot_patrol__srv__GetDirection_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  robot_patrol__srv__GetDirection_Response__Sequence * array = (robot_patrol__srv__GetDirection_Response__Sequence *)allocator.allocate(sizeof(robot_patrol__srv__GetDirection_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = robot_patrol__srv__GetDirection_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
robot_patrol__srv__GetDirection_Response__Sequence__destroy(robot_patrol__srv__GetDirection_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    robot_patrol__srv__GetDirection_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
robot_patrol__srv__GetDirection_Response__Sequence__are_equal(const robot_patrol__srv__GetDirection_Response__Sequence * lhs, const robot_patrol__srv__GetDirection_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!robot_patrol__srv__GetDirection_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
robot_patrol__srv__GetDirection_Response__Sequence__copy(
  const robot_patrol__srv__GetDirection_Response__Sequence * input,
  robot_patrol__srv__GetDirection_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(robot_patrol__srv__GetDirection_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    robot_patrol__srv__GetDirection_Response * data =
      (robot_patrol__srv__GetDirection_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!robot_patrol__srv__GetDirection_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          robot_patrol__srv__GetDirection_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!robot_patrol__srv__GetDirection_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
