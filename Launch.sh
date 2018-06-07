#!/bin/bash

if [ "$1" == '1' ];then
    echo "Launching  vehicle 1"
    python SetupAgent.py "$2" -100 0 3 True
    
elif [ "$1" == '2' ];then
    echo "Launching  vehicle 2"
    python SetupAgent.py "$2" 0 -100 3 False

elif [ "$1" == '3' ];then
    echo "Launching  vehicle 3"
    python SetupAgent.py "$2" -50 -50 3 0 False
    
else
    echo "options are 1,2,3"
fi
