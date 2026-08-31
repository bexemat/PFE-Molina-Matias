// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from extra_interfaces:msg/Trama.idl
// generated code does not contain a copyright notice
#include "extra_interfaces/msg/detail/trama__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
extra_interfaces__msg__Trama__init(extra_interfaces__msg__Trama * msg)
{
  if (!msg) {
    return false;
  }
  // Initialize fixed-size arrays to zero
  memset(msg->q, 0, sizeof(msg->q));
  memset(msg->qd, 0, sizeof(msg->qd));
  msg->t_total = 0.0;
  msg->n_iter = 0;
  return true;
}

void
extra_interfaces__msg__Trama__fini(extra_interfaces__msg__Trama * msg)
{
  if (!msg) {
    return;
  }
  // No dynamic memory to free for fixed-size arrays and primitives
  (void)msg;
}

bool
extra_interfaces__msg__Trama__are_equal(const extra_interfaces__msg__Trama * lhs, const extra_interfaces__msg__Trama * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // q
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->q[i] != rhs->q[i]) {
      return false;
    }
  }
  // qd
  for (size_t i = 0; i < 3; ++i) {
    if (lhs->qd[i] != rhs->qd[i]) {
      return false;
    }
  }
  // t_total
  if (lhs->t_total != rhs->t_total) {
    return false;
  }
  // n_iter
  if (lhs->n_iter != rhs->n_iter) {
    return false;
  }
  return true;
}

bool
extra_interfaces__msg__Trama__copy(
  const extra_interfaces__msg__Trama * input,
  extra_interfaces__msg__Trama * output)
{
  if (!input || !output) {
    return false;
  }
  // q
  memcpy(output->q, input->q, sizeof(input->q));
  // qd
  memcpy(output->qd, input->qd, sizeof(input->qd));
  // t_total
  output->t_total = input->t_total;
  // n_iter
  output->n_iter = input->n_iter;
  return true;
}

extra_interfaces__msg__Trama *
extra_interfaces__msg__Trama__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  extra_interfaces__msg__Trama * msg = (extra_interfaces__msg__Trama *)allocator.allocate(sizeof(extra_interfaces__msg__Trama), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(extra_interfaces__msg__Trama));
  bool success = extra_interfaces__msg__Trama__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
extra_interfaces__msg__Trama__destroy(extra_interfaces__msg__Trama * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    extra_interfaces__msg__Trama__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
extra_interfaces__msg__Trama__Sequence__init(extra_interfaces__msg__Trama__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  extra_interfaces__msg__Trama * data = NULL;

  if (size) {
    data = (extra_interfaces__msg__Trama *)allocator.zero_allocate(size, sizeof(extra_interfaces__msg__Trama), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = extra_interfaces__msg__Trama__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        extra_interfaces__msg__Trama__fini(&data[i - 1]);
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
extra_interfaces__msg__Trama__Sequence__fini(extra_interfaces__msg__Trama__Sequence * array)
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
      extra_interfaces__msg__Trama__fini(&array->data[i]);
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

extra_interfaces__msg__Trama__Sequence *
extra_interfaces__msg__Trama__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  extra_interfaces__msg__Trama__Sequence * array = (extra_interfaces__msg__Trama__Sequence *)allocator.allocate(sizeof(extra_interfaces__msg__Trama__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = extra_interfaces__msg__Trama__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
extra_interfaces__msg__Trama__Sequence__destroy(extra_interfaces__msg__Trama__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    extra_interfaces__msg__Trama__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
extra_interfaces__msg__Trama__Sequence__are_equal(const extra_interfaces__msg__Trama__Sequence * lhs, const extra_interfaces__msg__Trama__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!extra_interfaces__msg__Trama__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
extra_interfaces__msg__Trama__Sequence__copy(
  const extra_interfaces__msg__Trama__Sequence * input,
  extra_interfaces__msg__Trama__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(extra_interfaces__msg__Trama);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    extra_interfaces__msg__Trama * data =
      (extra_interfaces__msg__Trama *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!extra_interfaces__msg__Trama__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          extra_interfaces__msg__Trama__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!extra_interfaces__msg__Trama__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
