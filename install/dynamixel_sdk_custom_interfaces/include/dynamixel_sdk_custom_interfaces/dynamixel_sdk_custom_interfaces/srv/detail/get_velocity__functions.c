// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dynamixel_sdk_custom_interfaces:srv/GetVelocity.idl
// generated code does not contain a copyright notice
#include "dynamixel_sdk_custom_interfaces/srv/detail/get_velocity__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `id`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__init(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * msg)
{
  if (!msg) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint8__Sequence__init(&msg->id, 0)) {
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__fini(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * msg)
{
  if (!msg) {
    return;
  }
  // id
  rosidl_runtime_c__uint8__Sequence__fini(&msg->id);
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__are_equal(const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * lhs, const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint8__Sequence__are_equal(
      &(lhs->id), &(rhs->id)))
  {
    return false;
  }
  return true;
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__copy(
  const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * input,
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  if (!rosidl_runtime_c__uint8__Sequence__copy(
      &(input->id), &(output->id)))
  {
    return false;
  }
  return true;
}

dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request *
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * msg = (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request *)allocator.allocate(sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request));
  bool success = dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__destroy(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__init(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * data = NULL;

  if (size) {
    data = (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request *)allocator.zero_allocate(size, sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__fini(&data[i - 1]);
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
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__fini(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * array)
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
      dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__fini(&array->data[i]);
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

dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence *
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * array = (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence *)allocator.allocate(sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__destroy(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__are_equal(const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * lhs, const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence__copy(
  const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * input,
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request * data =
      (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_sdk_custom_interfaces__srv__GetVelocity_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


// Include directives for member types
// Member `velocity`
// already included above
// #include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__init(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * msg)
{
  if (!msg) {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__int32__Sequence__init(&msg->velocity, 0)) {
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__fini(msg);
    return false;
  }
  return true;
}

void
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__fini(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * msg)
{
  if (!msg) {
    return;
  }
  // velocity
  rosidl_runtime_c__int32__Sequence__fini(&msg->velocity);
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__are_equal(const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * lhs, const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__int32__Sequence__are_equal(
      &(lhs->velocity), &(rhs->velocity)))
  {
    return false;
  }
  return true;
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__copy(
  const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * input,
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // velocity
  if (!rosidl_runtime_c__int32__Sequence__copy(
      &(input->velocity), &(output->velocity)))
  {
    return false;
  }
  return true;
}

dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response *
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * msg = (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response *)allocator.allocate(sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response));
  bool success = dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__destroy(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__init(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * data = NULL;

  if (size) {
    data = (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response *)allocator.zero_allocate(size, sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__fini(&data[i - 1]);
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
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__fini(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * array)
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
      dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__fini(&array->data[i]);
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

dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence *
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * array = (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence *)allocator.allocate(sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__destroy(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__are_equal(const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * lhs, const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence__copy(
  const dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * input,
  dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response * data =
      (dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dynamixel_sdk_custom_interfaces__srv__GetVelocity_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
