cmake_minimum_required(VERSION 3.9)
project(MeEncoder C)

set(CMAKE_C_STANDARD 99)

add_executable(MeEncoder MeHostParser.c MeHostParser.h MeEncoderMotor.c MeEncoderMotor.h )