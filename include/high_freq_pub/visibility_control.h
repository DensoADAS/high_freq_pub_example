// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef HIGH_FREQ_PUB__VISIBILITY_CONTROL_H_
#define HIGH_FREQ_PUB__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define HIGH_FREQ_PUB_EXPORT __attribute__ ((dllexport))
    #define HIGH_FREQ_PUB_IMPORT __attribute__ ((dllimport))
  #else
    #define HIGH_FREQ_PUB_EXPORT __declspec(dllexport)
    #define HIGH_FREQ_PUB_IMPORT __declspec(dllimport)
  #endif
  #ifdef HIGH_FREQ_PUB_BUILDING_DLL
    #define HIGH_FREQ_PUB_PUBLIC HIGH_FREQ_PUB_EXPORT
  #else
    #define HIGH_FREQ_PUB_PUBLIC HIGH_FREQ_PUB_IMPORT
  #endif
  #define HIGH_FREQ_PUB_PUBLIC_TYPE HIGH_FREQ_PUB_PUBLIC
  #define HIGH_FREQ_PUB_LOCAL
#else
  #define HIGH_FREQ_PUB_EXPORT __attribute__ ((visibility("default")))
  #define HIGH_FREQ_PUB_IMPORT
  #if __GNUC__ >= 4
    #define HIGH_FREQ_PUB_PUBLIC __attribute__ ((visibility("default")))
    #define HIGH_FREQ_PUB_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define HIGH_FREQ_PUB_PUBLIC
    #define HIGH_FREQ_PUB_LOCAL
  #endif
  #define HIGH_FREQ_PUB_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // HIGH_FREQ_PUB__VISIBILITY_CONTROL_H_
