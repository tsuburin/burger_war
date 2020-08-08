#!/bin/bash

script_dir=$(cd $(dirname $0) && pwd)
compose_dir=$(cd $script_dir/.. && pwd)

cd $compose_dir
docker-compose exec -w "/home/burger/catkin_ws" burger bash -i -c 'exec catkin_make "$@"' "catkin_make" "$@"
