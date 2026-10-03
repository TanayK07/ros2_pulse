/* Copyright 2026 ros2_pulse contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 *
 * A plugin that links librcl, for test_loaned.py. rtld_local_host dlopen()s it RTLD_LOCAL and
 * does not link rcl itself, so librcl ends up OUTSIDE the global scope: the probe's
 * dlsym(RTLD_NEXT) cannot see it, while this plugin's PLT still binds to the preloaded wrapper.
 * Declared opaquely (same ABI as rcl/publisher.h) so the test needs no rcl headers. */
#include <stddef.h>

int rcl_publish_loaned_message(const void* publisher, void* ros_message, void* allocation);

/* A zero-initialized rcl_publisher_t ({impl = NULL}): the real rcl answers
 * RCL_RET_PUBLISHER_INVALID (300) without touching the middleware. */
int call_publish_loaned_on_zero_publisher(void) {
  struct {
    void* impl;
  } pub = {NULL};
  int dummy = 0;
  return rcl_publish_loaned_message(&pub, &dummy, NULL);
}
