#!/bin/bash

# コンテナ内のホームディレクトリでコマンド実行

set -e

self_dir=$(cd $(dirname $0) && pwd)
compose_dir=$(cd $self_dir/.. && pwd)

cd $compose_dir

docker-compose exec burger bash -i -c 'exec "$@"' "exec" "$@"
