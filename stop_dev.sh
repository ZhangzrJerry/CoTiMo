#!/bin/bash
SCRIPT_DIR=$(cd $(dirname $0); pwd)

export SRC_PATH=$SCRIPT_DIR

docker compose down