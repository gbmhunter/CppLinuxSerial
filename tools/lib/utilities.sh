#!/usr/bin/env bash

# Any subsequent commands which fail will cause the shell script to exit immediately
# set -e

# ANSI escape codes for message colouring
RED='\033[0;31m'
GREEN='\033[0;32m'
LIGHT_GREEN='\033[1;32m'
NC='\033[0m' # No Color

printInfo () {
	echo -e "${LIGHT_GREEN}${1}${NC}"
}
export -f printInfo

printError () {
	echo -e "${RED}${1}${NC}"
}
export -f printError