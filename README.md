# Projeto de Cidade Inteligente com Veículos Autónomos e Deteção de Ataques

## 📘 Visão Geral

Este projeto visa a simulação de uma cidade inteligente com uma frota de veículos autónomos operados por uma empresa de transporte semelhante à Uber. Estes veículos operam de forma totalmente autónoma e são distribuídos por quatro zonas distintas da cidade. O foco está na deteção e resposta a ataques de cibersegurança simulados utilizando um modelo de Machine Learning.

---

## 🧠 Objetivos

- Simular veículos autónomos operando em quatro zonas distintas.
- Utilizar redes IP privadas para comunicação entre os veículos.
- Integrar deteção de intrusões baseada em machine learning usando.
- Desenvolver resposta automática a ataques: isolamento do veículo e substituição.
- Utilizar as ferramentas ROS2, Gazebo e SPADE para implementação e simulação.

---

## 🗺️ Arquitetura do Sistema

### Zonas da Cidade

- Zona 1: Rede `10.150.1.0`
- Zona 2: Rede `10.150.2.0`
- Zona 3: Rede `10.150.3.0`
- Zona 4: Rede `10.150.4.0`

Cada zona é operada por **2 veículos autónomos**, formando uma **sub-rede isolada**.

### Componentes Principais

- **Veículos Autónomos**: Simulados no ROS2 + Gazebo, com sensores, navegação e comunicação.
- **Agentes SPADE**: Controlam comportamento dos veículos e interação com os sistemas centrais.
- **Sistema Central (Câmara Municipal)**: Avalia pacotes enviados pelos veículos para deteção de intrusões.
- **Modelo de Deteção de Intrusões**: Classificador que identifica se um pacote é malicioso ou não.

---

## 🔁 Fluxo de Operação

1. **Veículos operam normalmente nas suas zonas.**
2. **Periodicamente enviam um "pacote" com dados baseados num dataset escolhido.**
3. **Sistema Central avalia o pacote com um modelo de ML.**
4. **Se detetado ataque:**
   - O veículo entra em estado de paragem.
   - Um novo veículo é solicitado de outra zona com base no tempo estimado até ao local.
5. **O veículo substituto assume a tarefa do veículo atacado.**

---

## 🛠️ Tecnologias Utilizadas

| Componente        | Tecnologia/Ferramenta        |
|-------------------|------------------------------|
| Simulação Robótica| ROS2, Gazebo                 |
| Agentes           | SPADE                        |
| ML/Dados          | Dataset, Scikit-learn, TensorFlow ou PyTorch |
| Comunicação       | Redes IP privadas (10.150.X.0) |
| Coordenação       | MQTT, DDS ou outro protocolo interno de mensagens |

---

## 👥 Organização da Equipa

### 1. **Simulação e Robótica (ROS2 + Gazebo)**
Responsável por:
- Modelação e movimentação dos veículos.
- Sensores virtuais e navegação.
- Comunicação entre veículos e com o sistema central.

---

### 2. **Agentes Inteligentes (SPADE)**
Responsável por:
- Implementação dos agentes em SPADE.
- Comunicação entre os veículos e o sistema de deteção central.
- Lógica de substituição e estado dos veículos.

---

### 3. **Sistema de Deteção de Intrusões**
Responsável por:
- Treino e validação do modelo de Machine Learning.
- Integração do classificador com o sistema central.
- Avaliação de pacotes simulados baseados num dataset escolhido.

---

### 4. **Infraestrutura de Rede e Comunicação**
Responsável por:
- Implementação e simulação das redes IP privadas.
- Gestão da comunicação entre zonas e o sistema central.
- Análise de tempo de resposta e substituição entre zonas.

---

### 5. **Coordenação Geral e Integração**
Responsável por:
- Garantir a integração entre os componentes ROS2, SPADE e ML.
- Planeamento e gestão do cronograma do projeto.
- Documentação e entrega final.

---

## ✅ Tarefas Principais

| Tarefa                                 |
|----------------------------------------|
| Modelar veículos no Gazebo             |
| Desenvolver agentes SPADE              |
| Treinar modelo                         |
| Simular redes e IPs privadas           |
| Integrar ROS2 + SPADE + ML             |
| Testar deteção de ataques              |
| Documentar e preparar apresentação     |

---

## 📎 Referências

- [ROS2 Documentation](https://docs.ros.org/)
- [Gazebo Documentation](https://gazebosim.org/)
- [SPADE Agents Framework](https://github.com/javipalanca/spade)

---

## 📌 Notas Finais

- Todos os componentes devem ser modulares e facilmente testáveis.
- Utilizar logs e simulações para validação de cada etapa.
- Priorizar a integração contínua e testes incrementais.
