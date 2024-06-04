#!/usr/bin/env bash

cd ${TRHEX_LINUX_DIR}
. env.cache
cd operator
./rhex_gui 3000 localhost 5000
