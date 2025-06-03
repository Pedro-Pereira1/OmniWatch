#!/bin/bash

NAME=$1
X=${2:-0}
Y=${3:-0}

# Caminho original do modelo
MODEL_PATH=~/.gazebo/models/simple_car/model.sdf

# Caminho temporário com substituição
TEMP_PATH=/tmp/${NAME}.sdf

# Substitui __MODEL_NAME__ pelo nome fornecido
sed "s/__MODEL_NAME__/${NAME}/g" $MODEL_PATH > $TEMP_PATH

# Spawna o modelo com o SDF modificado
ros2 run gazebo_ros spawn_entity.py -entity ${NAME} -file ${TEMP_PATH} -x ${X} -y ${Y} -z 0.1
