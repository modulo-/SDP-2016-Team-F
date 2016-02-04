#!/bin/bash
g++ -o vision src/main.cpp -I/usr/include $(pkg-config --libs opencv)

