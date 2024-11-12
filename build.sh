#!/bin/bash
if [ "$1" == "debug" ]; then
  cmake -DCMAKE_BUILD_TYPE=Debug -B build
  make -C build -j
elif [ "$1" == "debugRun" ]; then
  cmake -DCMAKE_BUILD_TYPE=Debug -B build
  make -C build -j
  ./build/bin/HybridAstar
elif [ "$1" == "release" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -B build
  make -C build -j
elif [ "$1" == "releaseRun" ]; then
  cmake -DCMAKE_BUILD_TYPE=Release -B build
  make -C build -j
  ./build/bin/HybridAstar
elif [ "$1" == "clean" ]; then
  rm -rf build
elif [ "$1" == "run" ]; then
  ./build/bin/HybridAstar
else
  echo "Usage: ./build.sh [run|debug|debugRun|release|releaseRun|clean]"
fi
