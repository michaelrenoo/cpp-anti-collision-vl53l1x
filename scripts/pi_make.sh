#! /bin/bash

echo "stop diaven-cc service..."
{
    sudo systemctl stop diaven-cc
    echo "ok"
}|| {
    echo "diaven-cc did not run as a service yet"
}

cd $(dirname "$0")/../main
make -j4 -f makefile_pi3 $1
