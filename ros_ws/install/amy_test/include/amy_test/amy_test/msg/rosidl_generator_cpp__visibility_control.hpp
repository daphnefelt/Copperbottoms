// generated from rosidl_generator_cpp/resource/rosidl_generator_cpp__visibility_control.hpp.in
// generated code does not contain a copyright notice

#ifndef AMY_TEST__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
#define AMY_TEST__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_GENERATOR_CPP_EXPORT_amy_test __attribute__ ((dllexport))
    #define ROSIDL_GENERATOR_CPP_IMPORT_amy_test __attribute__ ((dllimport))
  #else
    #define ROSIDL_GENERATOR_CPP_EXPORT_amy_test __declspec(dllexport)
    #define ROSIDL_GENERATOR_CPP_IMPORT_amy_test __declspec(dllimport)
  #endif
  #ifdef ROSIDL_GENERATOR_CPP_BUILDING_DLL_amy_test
    #define ROSIDL_GENERATOR_CPP_PUBLIC_amy_test ROSIDL_GENERATOR_CPP_EXPORT_amy_test
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_amy_test ROSIDL_GENERATOR_CPP_IMPORT_amy_test
  #endif
#else
  #define ROSIDL_GENERATOR_CPP_EXPORT_amy_test __attribute__ ((visibility("default")))
  #define ROSIDL_GENERATOR_CPP_IMPORT_amy_test
  #if __GNUC__ >= 4
    #define ROSIDL_GENERATOR_CPP_PUBLIC_amy_test __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_GENERATOR_CPP_PUBLIC_amy_test
  #endif
#endif

#ifdef __cplusplus
}
#endif

#endif  // AMY_TEST__MSG__ROSIDL_GENERATOR_CPP__VISIBILITY_CONTROL_HPP_
