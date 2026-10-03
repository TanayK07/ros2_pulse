/* Copyright 2026 ros2_pulse contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 *
 * Host for test_loaned.py: dlopen()s the rcl-linked plugin RTLD_LOCAL (this binary does not
 * link rcl) and prints what rcl_publish_loaned_message returned through the plugin. */
#include <dlfcn.h>
#include <stdio.h>

int main(int argc, char** argv) {
  if (argc < 2) {
    return 2;
  }
  void* h = dlopen(argv[1], RTLD_NOW | RTLD_LOCAL);
  if (h == NULL) {
    fprintf(stderr, "%s\n", dlerror());
    return 3;
  }
  int (*f)(void) = (int (*)(void))dlsym(h, "call_publish_loaned_on_zero_publisher");
  if (f == NULL) {
    return 4;
  }
  printf("ret=%d\n", f());
  return 0;
}
