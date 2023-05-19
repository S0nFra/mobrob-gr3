#!/bin/bash
PKG_DIR=$( cd -- "$(dirname "${0}")" >/dev/null 2>&1 ; pwd -P )

# echo $PKG_DIR/../launch/
python3 "$PKG_DIR/../launch/launch.py" 