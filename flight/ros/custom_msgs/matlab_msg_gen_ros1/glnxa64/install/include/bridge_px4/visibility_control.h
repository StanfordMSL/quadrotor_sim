#ifndef BRIDGE_PX4__VISIBILITY_CONTROL_H_
#define BRIDGE_PX4__VISIBILITY_CONTROL_H_
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define BRIDGE_PX4_EXPORT __attribute__ ((dllexport))
    #define BRIDGE_PX4_IMPORT __attribute__ ((dllimport))
  #else
    #define BRIDGE_PX4_EXPORT __declspec(dllexport)
    #define BRIDGE_PX4_IMPORT __declspec(dllimport)
  #endif
  #ifdef BRIDGE_PX4_BUILDING_LIBRARY
    #define BRIDGE_PX4_PUBLIC BRIDGE_PX4_EXPORT
  #else
    #define BRIDGE_PX4_PUBLIC BRIDGE_PX4_IMPORT
  #endif
  #define BRIDGE_PX4_PUBLIC_TYPE BRIDGE_PX4_PUBLIC
  #define BRIDGE_PX4_LOCAL
#else
  #define BRIDGE_PX4_EXPORT __attribute__ ((visibility("default")))
  #define BRIDGE_PX4_IMPORT
  #if __GNUC__ >= 4
    #define BRIDGE_PX4_PUBLIC __attribute__ ((visibility("default")))
    #define BRIDGE_PX4_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define BRIDGE_PX4_PUBLIC
    #define BRIDGE_PX4_LOCAL
  #endif
  #define BRIDGE_PX4_PUBLIC_TYPE
#endif
#endif  // BRIDGE_PX4__VISIBILITY_CONTROL_H_
// Generated 13-Aug-2021 17:24:04
// Copyright 2019-2020 The MathWorks, Inc.
