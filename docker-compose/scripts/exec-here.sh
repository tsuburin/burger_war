#!/bin/bash

# カレントディレクトリがリポジトリ内の場合、
# カレントディレクトリに対応するコンテナ内のディレクトリに入ってコマンドを実行する
# 
# カレントディレクトリがリポジトリ外の場合、何もせずエラー終了

set -e

self_dir=$(cd $(dirname $0) && pwd)
compose_dir=$(cd $self_dir/.. && pwd)
repo_dir=$(cd $compose_dir/.. && pwd)
current_dir=$(pwd)

# current directoryがリポジトリ内じゃなければエラー終了
if !(echo $current_dir | grep "^$repo_dir" >/dev/null 2>&1); then
  echo "error: you are out of the root of the repository, $repo_dir" >&2
  exit 1
fi

path_in_repo=${current_dir#$repo_dir}

cd $compose_dir

docker-compose exec -w "/home/burger/catkin_ws/src/burger_war/$path_in_repo" burger bash -i -c 'exec "$@"' "exec" "$@"
