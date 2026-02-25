#ifndef myco_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_
#define myco_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define myco_HARDWARE_INTERFACE_EXPORT __attribute__((dllexport))
#define myco_HARDWARE_INTERFACE_IMPORT __attribute__((dllimport))
#else
#define myco_HARDWARE_INTERFACE_EXPORT __declspec(dllexport)
#define myco_HARDWARE_INTERFACE_IMPORT __declspec(dllimport)
#endif
#ifdef myco_HARDWARE_INTERFACE_BUILDING_LIBRARY
#define myco_HARDWARE_INTERFACE_PUBLIC myco_HARDWARE_INTERFACE_EXPORT
#else
#define myco_HARDWARE_INTERFACE_PUBLIC myco_HARDWARE_INTERFACE_IMPORT
#endif
#define myco_HARDWARE_INTERFACE_PUBLIC_TYPE myco_HARDWARE_INTERFACE_PUBLIC
#define myco_HARDWARE_INTERFACE_LOCAL
#else
#define myco_HARDWARE_INTERFACE_EXPORT __attribute__((visibility("default")))
#define myco_HARDWARE_INTERFACE_IMPORT
#if __GNUC__ >= 4
#define myco_HARDWARE_INTERFACE_PUBLIC __attribute__((visibility("default")))
#define myco_HARDWARE_INTERFACE_LOCAL __attribute__((visibility("hidden")))
#else
#define myco_HARDWARE_INTERFACE_PUBLIC
#define myco_HARDWARE_INTERFACE_LOCAL
#endif
#define myco_HARDWARE_INTERFACE_PUBLIC_TYPE
#endif

#endif  // myco_HARDWARE_INTERFACE__VISIBILITY_CONTROL_HPP_