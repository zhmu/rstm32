#!/bin/sh
find . -name \*.cpp -or -name \*.h -exec clang-format -i {} \;
