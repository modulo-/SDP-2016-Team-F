#!/bin/bash
clang++ --std=c++11 -o vision src/*.cpp -Iinclude -I/usr/include $(pkg-config --libs opencv)

