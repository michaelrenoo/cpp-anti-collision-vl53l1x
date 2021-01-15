#! /bin/bash

cd $(dirname "$0")/..


echo "check if update is prepared"
UPDATE_DIR="update"
RUN_DIR="build"
if [ -d "$UPDATE_DIR" ]; then
    echo "install update..."
    cp -r "$UPDATE_DIR/"* "$RUN_DIR"
    echo "cleaning up update..."
    rm -f -r "$UPDATE_DIR"
    echo "update done"
fi

echo "claim permission for serial connections..."
{
    sudo chmod 666 /dev/ttyS*
    echo "/dev/ttyS* ok" 
}|| {
    echo "could not change permisison for /dev/ttyS*"
}

{
    sudo chmod 666 /dev/ttyUSB*
}|| {
    echo "could not change permisison for /dev/ttyUSB*"
    echo "/dev/ttyUSB* ok" 
}
echo "run main..."
build/main