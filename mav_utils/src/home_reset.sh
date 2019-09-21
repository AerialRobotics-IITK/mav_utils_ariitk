#!/bin/bash
sleep 2s
rosservice call /$MAV_NAME/home_reset "{}"
