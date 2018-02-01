#!/bin/bash

if [ "$1" == '1' ];then
    echo "Launching  vehicle 1"
    python SetupAgent.py "$2" -100 0 0 3 0 0 False
    
elif [ "$1" == '2' ];then
    echo "Launching  vehicle 2"
    python SetupAgent.py "$2" 0 -100 0 0 3 0 False

elif [ "$1" == '3' ];then
    echo "Launching  vehicle 3"
    python SetupAgent.py "$2" -50 -50 0 1.5 1.5 0 False
    
else
    echo "options are 1,2"
fi
