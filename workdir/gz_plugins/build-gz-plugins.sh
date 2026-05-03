#!/bin/bash
set -e
for dir in "$@"; do
    name=$(basename "$dir")
    so_files=("$dir"/build/lib*.so)
    if [[ ! -f "${so_files[0]}" ]]; then
        echo "Building Gazebo plugin: $name ..."
        mkdir -p "$dir/build"
        pushd "$dir/build" > /dev/null
        cmake ..
        make -j$(nproc)
        popd > /dev/null
        echo "Built: $name"
    else
        echo "Gazebo plugin already built: $name"
    fi
done
