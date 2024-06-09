#!/bin/bash
set -e

echo "Collision Checker (TRAF22)"

for filename in /results/*occupancies.csv; do
    python3 check_collision.py $filename
done

# the next line has the purpose to keep the output alive
error
