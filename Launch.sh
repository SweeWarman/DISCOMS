#!/bin/bash

if [ "$1" == '1' ];then
    echo "Launching  vehicle 1"
    python SetupAgent.py vehicle1 -100 0 0 4 0 0 True
    
elif [ "$1" == '2' ];then
    echo "Launching  vehicle 2"
    python SetupAgent.py vehicle2 0 -100 0 0 4 0 False
    
else
    echo "options are 1,2"
fi
