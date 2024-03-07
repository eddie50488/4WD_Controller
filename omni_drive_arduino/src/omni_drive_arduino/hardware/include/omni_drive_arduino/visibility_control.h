#ifndef OMNI_DRIVE_ARDUINO__VISIBILITY_CONTROL_H_
#define OMNI_DRIVE_ARDUINO__VISIBILITY_CONTROL_H_

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OMNI_DRIVE_ARDUINO_EXPORT __attribute__((dllexport))
#define OMNI_DRIVE_ARDUINO_IMPORT __attribute__((dllimport))
#else
#define OMNI_DRIVE_ARDUINO_EXPORT __declspec(dllexport)
#define OMNI_DRIVE_ARDUINO_IMPORT __declspec(dllimport)
#endif
#ifdef OMNI_DRIVE_ARDUINO_BUILDING_DLL
#define OMNI_DRIVE_ARDUINO_PUBLIC OMNI_DRIVE_ARDUINO_EXPORT
#else
#define OMNI_DRIVE_ARDUINO_PUBLIC OMNI_DRIVE_ARDUINO_IMPORT
#endif
#define OMNI_DRIVE_ARDUINO_PUBLIC_TYPE OMNI_DRIVE_ARDUINO_PUBLIC
#define OMNI_DRIVE_ARDUINO_LOCAL
#else
#define OMNI_DRIVE_ARDUINO_EXPORT __attribute__((visibility("default")))
#define OMNI_DRIVE_ARDUINO_IMPORT
#if __GNUC__ >= 4
#define OMNI_DRIVE_ARDUINO_PUBLIC __attribute__((visibility("default")))
#define OMNI_DRIVE_ARDUINO_LOCAL __attribute__((visibility("hidden")))
#else
#define OMNI_DRIVE_ARDUINO_PUBLIC
#define OMNI_DRIVE_ARDUINO_LOCAL
#endif
#define OMNI_DRIVE_ARDUINO_PUBLIC_TYPE
#endif

#endif // OMNI_DRIVE_ARDUINO__VISIBILITY_CONTROL_H_
