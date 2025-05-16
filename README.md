# Projeto de Navegação Walle

Um projeto ROS2 com Walle, um robô de tração diferencial que navega em uma simulação Gazebo com obstáculos. Este projeto demonstra a evasão de obstáculos usando sensores ultrassônicos e controle ROS2.

## Visão Geral
- **Robô**: Walle, um robô de tração diferencial equipado com sensores ultrassônicos.
- **Simulação**: Gazebo com um mundo personalizado (`obstacle_world.world`) contendo obstáculos como caixas, cilindros e paredes.
- **Framework**: ROS2 Humble.
- **Objetivo**: Demonstrar navegação autônoma e evasão de obstáculos.

## Pré-requisitos
- Ubuntu 22.04
- ROS2 Humble
- Gazebo
- Python 3.8+

## Instalação
1. **Configurar ROS2 Humble**:
   Siga o [guia oficial de instalação do ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html).

2. **Instalar Dependências**:
   ```bash
   sudo apt update
   sudo apt install ros-humble-gazebo-ros ros-humble-gazebo-plugins ros-humble-rviz2
   ```

3. **Clonar o Repositório**:
   ```bash
   mkdir -p ~/walle_ws/src
   cd ~/walle_ws/src
   git clone <seu-url-do-repositorio>
   ```

4. **Compilar o Workspace**:
   ```bash
   cd ~/walle_ws
   colcon build
   source install/setup.bash
   ```

## Uso
1. **Iniciar a Simulação**:
   ```bash
   ros2 launch walle_description gazebo_walle.launch.py
   ```
   - Isso inicia o Gazebo com `obstacle_world.world`, gera o Walle e abre o RViz.

2. **Executar o Nó de Evasão de Obstáculos**:
   ```bash
   ros2 run walle_control obstacle_avoidance
   ```
   - O Walle navegará e evitará obstáculos usando sensores ultrassônicos.

## Estrutura do Projeto
- **`walle_description/`**:
  - `urdf/walle.urdf`: Modelo do robô com sensores.
  - `worlds/obstacle_world.world`: Mundo Gazebo com obstáculos.
  - `launch/gazebo_walle.launch.py`: Arquivo de lançamento para simulação.
- **`walle_control/`**:
  - `obstacle_avoidance.py`: Nó ROS2 para navegação e evasão de obstáculos.

## Funcionalidades
- Controle de tração diferencial com `/cmd_vel` e `/odom`.
- Sensores ultrassônicos para detecção de obstáculos.
- Mundo Gazebo personalizado com três obstáculos: uma caixa, um cilindro e uma parede.
- Visualização em tempo real no RViz.

## Solução de Problemas
- **Gazebo Não Carrega o Mundo**:
  - Certifique-se de que `obstacle_world.world` está em `walle_description/worlds/`.
  - Verifique o `$ROS_PACKAGE_PATH`:
    ```bash
    echo $ROS_PACKAGE_PATH
    ```
    - Deve incluir `/home/<seu-usuario>/walle_ws/install/walle_description/share/walle_description`.

- **Simulação Travando ou Lentidão**:
  - Reduza as configurações gráficas do Gazebo (`Edit > GUI Settings`).
  - Monitore os recursos do sistema:
    ```bash
    htop
    ```

## Melhorias Futuras
- Adicionar algoritmos mais complexos de evasão (ex.: DWA, A*).
- Incluir um mapa para SLAM.
- Melhorar o mundo Gazebo com obstáculos dinâmicos.

## Licença
Este projeto está licenciado sob a Licença MIT - veja o arquivo [LICENSE](LICENSE) para detalhes.

## Agradecimentos
- Comunidades ROS2 e Gazebo pelos excelentes ferramentas e documentação.
- Inspirado em diversos tutoriais de navegação ROS2.