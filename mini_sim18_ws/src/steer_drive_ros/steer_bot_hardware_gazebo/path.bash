#!/bin/bash

# include path を作成する
include_path=$(pwd)/include

# CPATHに追加
export CPATH=${CPATH}:${include_path}:${gazebo_path}:${sdf_path}
