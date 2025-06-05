#!/bin/bash

MAP_DIR="/home/dominicd/maps"

if [ ! -d "$MAP_DIR" ]; then
    mkdir -p "$MAP_DIR"
    echo "Created directory $MAP_DIR"
else
    echo "Directory $MAP_DIR already exists"
fi
