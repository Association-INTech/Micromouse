#pragma once

// this file is only meant to be included by main.h from cubemx
// it is the boundary between the cubemx C and the application C++

#ifdef __cplusplus
extern "C" {
#endif

[[noreturn]] void app_main();

#ifdef __cplusplus
}
#endif
