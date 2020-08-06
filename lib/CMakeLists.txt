project(rstm32-lib)
include_directories(include)
add_library(rstm32 src/platform.cpp)
target_include_directories(rstm32 INTERFACE include)
