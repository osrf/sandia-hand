#!/bin/bash
rosrun image_view image_view image:=right/left/image_raw  _autosize:=true &
rosrun image_view image_view image:=right/right/image_raw _autosize:=true &
rosrun image_view image_view image:=left/left/image_raw   _autosize:=true &
rosrun image_view image_view image:=left/right/image_raw  _autosize:=true &
