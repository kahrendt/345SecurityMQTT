#!/bin/sh
g++  -o 345toMqtt -fdiagnostics-color --std=c++11 mqtt.cpp digitalDecoder.cpp analogDecoder.cpp main.cpp -lyaml-cpp -lrtlsdr -lmosquittopp
