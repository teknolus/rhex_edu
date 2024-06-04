#!/usr/bin/env bash

cd ${TRHEX_LINUX_DIR}
. env.cache
./bin/supervisor $@
