# 🤖 Robot Surveillance System

Sistema de vigilância inteligente usando ROS 2 Humble e SPADE Agents.

## 🔍 Use Cases

- UC1: Robô movimenta-se aleatoriamente
- UC2: Coloca sensores e recolhe dados (Temp, CO2, Hum.)
- UC3: Ativação de sprinklers
- UC4: CO2 apenas sem presença humana
- UC5: Deteção de intrusões
- UC6: Cooperação entre robôs e câmaras
- UC7: Reaprendizagem com novos objetos
- UC8: Competição entre robôs (não ir para o mesmo local)

## 📦 Estrutura

- `ros2_ws/` - Workspace com os nós ROS 2
- `spade_agents/` - Agentes SPADE
- `simulation/` - Ambientes de simulação
- `docs/` - Documentação do sistema

## 🛠️ Instalação

```bash
chmod +x install.sh
./install.sh
