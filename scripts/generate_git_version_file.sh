#! /bin/bash

cd "$(dirname "$0")/../main"

echo -en "#ifndef GITVERSION_H_\n#define GITVERSION_H_\n#define GIT_HASH 0x$(git rev-parse --short HEAD)UL\n#define GIT_BRANCH \"$(git rev-parse --abbrev-ref HEAD)\"\n#endif\n" > gitversion.hpp