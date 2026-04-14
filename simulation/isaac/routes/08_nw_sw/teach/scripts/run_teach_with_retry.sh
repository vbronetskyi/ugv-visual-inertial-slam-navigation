#!/bin/bash
# Wrapper: run teach; retry up to N times on drift abort.
set -eu
ROUTE=${1:-08_nw_sw}
MAX_RETRIES=${MAX_RETRIES:-3}
HERE=$(cd "$(dirname "$0")" && pwd)

for ATTEMPT in $(seq 1 $MAX_RETRIES); do
    echo "########## TEACH $ROUTE attempt $ATTEMPT/$MAX_RETRIES ##########"
    bash $HERE/run_teach_single.sh $ROUTE && { echo "OK on attempt $ATTEMPT"; exit 0; }
    CODE=$?
    if [ $CODE -ne 2 ] && [ $CODE -ne 1 ]; then
        echo "non-retryable exit code $CODE on attempt $ATTEMPT"
        exit $CODE
    fi
    echo "drift abort - waiting 10 s, retry ..."
    sleep 10
done
echo "!!! all $MAX_RETRIES attempts failed on drift gate"
exit 2
