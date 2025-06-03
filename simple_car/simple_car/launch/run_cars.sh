#!/bin/bash

cd ~/Documentos/ISEP/OmniWatch/simple_car

# Nome da sessão
SESSION_NAME="mysession"

# Cria a sessão tmux desligada
tmux new-session -d -s $SESSION_NAME

# Primeiro comando no primeiro painel (pane 0)
tmux send-keys -t 0 "source install/setup.zsh; ros2 run simple_car waypoint_follower --ros-args -p car_name:=car_1" C-m

# Para cada carro de car_2 a car_20, cria um novo painel e envia o comando
for i in $(seq 2 6); do
  tmux split-window -v
  tmux select-layout tiled
  tmux send-keys "source install/setup.zsh; ros2 run simple_car waypoint_follower --ros-args -p car_name:=car_${i}" C-m
done

# Anexa a sessão para ver tudo
tmux attach-session -t $SESSION_NAME
