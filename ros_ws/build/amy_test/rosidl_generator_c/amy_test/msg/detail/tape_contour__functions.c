// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from amy_test:msg/TapeContour.idl
// generated code does not contain a copyright notice
#include "amy_test/msg/detail/tape_contour__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
amy_test__msg__TapeContour__init(amy_test__msg__TapeContour * msg)
{
  if (!msg) {
    return false;
  }
  // found
  // center_x
  // center_y
  // angle
  // area
  // sharp_turn
  return true;
}

void
amy_test__msg__TapeContour__fini(amy_test__msg__TapeContour * msg)
{
  if (!msg) {
    return;
  }
  // found
  // center_x
  // center_y
  // angle
  // area
  // sharp_turn
}

bool
amy_test__msg__TapeContour__are_equal(const amy_test__msg__TapeContour * lhs, const amy_test__msg__TapeContour * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // found
  if (lhs->found != rhs->found) {
    return false;
  }
  // center_x
  if (lhs->center_x != rhs->center_x) {
    return false;
  }
  // center_y
  if (lhs->center_y != rhs->center_y) {
    return false;
  }
  // angle
  if (lhs->angle != rhs->angle) {
    return false;
  }
  // area
  if (lhs->area != rhs->area) {
    return false;
  }
  // sharp_turn
  if (lhs->sharp_turn != rhs->sharp_turn) {
    return false;
  }
  return true;
}

bool
amy_test__msg__TapeContour__copy(
  const amy_test__msg__TapeContour * input,
  amy_test__msg__TapeContour * output)
{
  if (!input || !output) {
    return false;
  }
  // found
  output->found = input->found;
  // center_x
  output->center_x = input->center_x;
  // center_y
  output->center_y = input->center_y;
  // angle
  output->angle = input->angle;
  // area
  output->area = input->area;
  // sharp_turn
  output->sharp_turn = input->sharp_turn;
  return true;
}

amy_test__msg__TapeContour *
amy_test__msg__TapeContour__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amy_test__msg__TapeContour * msg = (amy_test__msg__TapeContour *)allocator.allocate(sizeof(amy_test__msg__TapeContour), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(amy_test__msg__TapeContour));
  bool success = amy_test__msg__TapeContour__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
amy_test__msg__TapeContour__destroy(amy_test__msg__TapeContour * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    amy_test__msg__TapeContour__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
amy_test__msg__TapeContour__Sequence__init(amy_test__msg__TapeContour__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amy_test__msg__TapeContour * data = NULL;

  if (size) {
    data = (amy_test__msg__TapeContour *)allocator.zero_allocate(size, sizeof(amy_test__msg__TapeContour), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = amy_test__msg__TapeContour__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        amy_test__msg__TapeContour__fini(&data[i - 1]);
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
amy_test__msg__TapeContour__Sequence__fini(amy_test__msg__TapeContour__Sequence * array)
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
      amy_test__msg__TapeContour__fini(&array->data[i]);
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

amy_test__msg__TapeContour__Sequence *
amy_test__msg__TapeContour__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  amy_test__msg__TapeContour__Sequence * array = (amy_test__msg__TapeContour__Sequence *)allocator.allocate(sizeof(amy_test__msg__TapeContour__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = amy_test__msg__TapeContour__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
amy_test__msg__TapeContour__Sequence__destroy(amy_test__msg__TapeContour__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    amy_test__msg__TapeContour__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
amy_test__msg__TapeContour__Sequence__are_equal(const amy_test__msg__TapeContour__Sequence * lhs, const amy_test__msg__TapeContour__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!amy_test__msg__TapeContour__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
amy_test__msg__TapeContour__Sequence__copy(
  const amy_test__msg__TapeContour__Sequence * input,
  amy_test__msg__TapeContour__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(amy_test__msg__TapeContour);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    amy_test__msg__TapeContour * data =
      (amy_test__msg__TapeContour *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!amy_test__msg__TapeContour__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          amy_test__msg__TapeContour__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!amy_test__msg__TapeContour__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
