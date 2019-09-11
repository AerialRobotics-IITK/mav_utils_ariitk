#!/bin/bash
sleep 2s
rosservice call /$1/home_reset "{}"
