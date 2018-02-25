#!/bin/bash

rosrun tf static_transform_publisher 0 0 0 0 0 0 /base_footprint /laser 100
