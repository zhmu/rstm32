#!/bin/sh -e
R=`realpath $PWD`
rm -rf build
cmake -GNinja -DCMAKE_TOOLCHAIN_FILE="$R/target/toolchain.cmake" -B build -S .
cd build && ninja
