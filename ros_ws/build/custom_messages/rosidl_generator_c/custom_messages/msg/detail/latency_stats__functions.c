// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_messages:msg/LatencyStats.idl
// generated code does not contain a copyright notice
#include "custom_messages/msg/detail/latency_stats__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_messages__msg__LatencyStats__init(custom_messages__msg__LatencyStats * msg)
{
  if (!msg) {
    return false;
  }
  // structure_needs_at_least_one_member
  return true;
}

void
custom_messages__msg__LatencyStats__fini(custom_messages__msg__LatencyStats * msg)
{
  if (!msg) {
    return;
  }
  // structure_needs_at_least_one_member
}

bool
custom_messages__msg__LatencyStats__are_equal(const custom_messages__msg__LatencyStats * lhs, const custom_messages__msg__LatencyStats * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // structure_needs_at_least_one_member
  if (lhs->structure_needs_at_least_one_member != rhs->structure_needs_at_least_one_member) {
    return false;
  }
  return true;
}

bool
custom_messages__msg__LatencyStats__copy(
  const custom_messages__msg__LatencyStats * input,
  custom_messages__msg__LatencyStats * output)
{
  if (!input || !output) {
    return false;
  }
  // structure_needs_at_least_one_member
  output->structure_needs_at_least_one_member = input->structure_needs_at_least_one_member;
  return true;
}

custom_messages__msg__LatencyStats *
custom_messages__msg__LatencyStats__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__LatencyStats * msg = (custom_messages__msg__LatencyStats *)allocator.allocate(sizeof(custom_messages__msg__LatencyStats), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_messages__msg__LatencyStats));
  bool success = custom_messages__msg__LatencyStats__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_messages__msg__LatencyStats__destroy(custom_messages__msg__LatencyStats * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_messages__msg__LatencyStats__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_messages__msg__LatencyStats__Sequence__init(custom_messages__msg__LatencyStats__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__LatencyStats * data = NULL;

  if (size) {
    data = (custom_messages__msg__LatencyStats *)allocator.zero_allocate(size, sizeof(custom_messages__msg__LatencyStats), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_messages__msg__LatencyStats__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_messages__msg__LatencyStats__fini(&data[i - 1]);
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
custom_messages__msg__LatencyStats__Sequence__fini(custom_messages__msg__LatencyStats__Sequence * array)
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
      custom_messages__msg__LatencyStats__fini(&array->data[i]);
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

custom_messages__msg__LatencyStats__Sequence *
custom_messages__msg__LatencyStats__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_messages__msg__LatencyStats__Sequence * array = (custom_messages__msg__LatencyStats__Sequence *)allocator.allocate(sizeof(custom_messages__msg__LatencyStats__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_messages__msg__LatencyStats__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_messages__msg__LatencyStats__Sequence__destroy(custom_messages__msg__LatencyStats__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_messages__msg__LatencyStats__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_messages__msg__LatencyStats__Sequence__are_equal(const custom_messages__msg__LatencyStats__Sequence * lhs, const custom_messages__msg__LatencyStats__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_messages__msg__LatencyStats__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_messages__msg__LatencyStats__Sequence__copy(
  const custom_messages__msg__LatencyStats__Sequence * input,
  custom_messages__msg__LatencyStats__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_messages__msg__LatencyStats);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_messages__msg__LatencyStats * data =
      (custom_messages__msg__LatencyStats *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_messages__msg__LatencyStats__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_messages__msg__LatencyStats__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_messages__msg__LatencyStats__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
