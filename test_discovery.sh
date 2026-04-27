#!/bin/bash
# Quick test script for WildBridge drone discovery

echo "======================================"
echo "WildBridge Drone Discovery Quick Test"
echo "======================================"
echo ""

# Check if Python 3 is available
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed"
    exit 1
fi

# Install required packages if needed
echo "Checking dependencies..."
pip3 list | grep -q requests || pip3 install --user requests

echo ""
echo "Running discovery test..."
echo ""

python3 /home/alejp/dev/WildBridge/test_discovery.py

exit $?
