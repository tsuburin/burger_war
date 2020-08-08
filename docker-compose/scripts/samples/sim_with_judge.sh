#!/bin/bash

# リポジトリのルートに移動
cd $(dirname $0)/../../../

# burger_war/scripts/sim_with_judge.shと同等のコマンドを実行
alacritty -e docker-compose/scripts/exec-here.sh python judge/judgeServer.py &
alacritty -e docker-compose/scripts/exec-here.sh python judge/visualizeWindow.py &

docker-compose/scripts/exec-here.sh bash judge/test_scripts/init_single_play.sh judge/marker_set/sim.csv localhost:5000 you enemy

docker-compose/scripts/exec-here.sh roslaunch burger_war setup_sim.launch
