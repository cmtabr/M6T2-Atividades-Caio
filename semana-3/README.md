# Semana 3 
Como solicitado no autoestudo "Exercício estruturas de dados", abaixo vê-se o vídeo relativo a iniciação do ambiente de simulação, Gazebo, com os packages do TurtleBot3. 

## Objetivo

Demonstrar a capacidade de interagir de forma dinâmica com uma plataforma de simulação utilizando os conceitos de estrutura de dados e programação orientada à objetos, outrossim utilizando os protocolos de comunicação via publicação e subscrição em tópicos com os parâmetros de mensagens definidas (https://github.com/rticommunity/ros-data-types).

## Solução 
O código apresentado implementa um nó chamado MovingRobot, que herda a classe Node do ROS 2.
Ele se inscreve no tópico **`odom`** para obter a posição atual do robô e utiliza uma fila para receber os pontos de destino fornecidos pelo usuário. 
O nó calcula as velocidades linear e angular necessárias para mover o robô em direção aos pontos de destino. O método **point_callback** é responsável por esse cálculo e publicação das velocidades no tópico **`cmd_vel`**. 
O método **position_callback** atualiza a posição atual do robô quando uma nova mensagem Odometry é recebida. 
O método **get_user_input** obtém os pontos de destino do usuário. 
A função main é responsável por inicializar e executar o nó. 
Em resumo, o código cria um nó ROS 2 que permite ao usuário fornecer pontos de destino para controlar o movimento do robô.

## Vídeo 
Abaixo você pode ver o vídeo da simulação funcionando com o código implementado e o link para o youtube do vídeo em questão:

https://github.com/cmtabr/M6T2-Atividades-Caio/assets/99201276/3fd83c26-0d54-446f-8813-5402bd64aea6

**Youtube:** https://youtu.be/peI45p2xHoE

Pontos utilizados na demonstração:
|Ponto| X | Y |
|----------|----------|----------|
| 1 | 10.0 | 0.0 |
| 2 | 3.0 | 4.0 |
| 3 | 0.0 | 0.0 |

## Conclusão: 
A atividade requisita que seja feita uma simulação de movimentação de robôs utilizando conceitos de estrutura de dados e POO, desta forma, para provar a capacidade do aluno de interagir com um sistema operacional de robôs e sua simulação através do Gazebo, utilizamos o código a seguir para abrir a interface gráfica do turtlesim:
```bash 
ros2 launch turtlebot3_gazebo empty_world.launch.py
``` 

Então, para que possamos movimentar o robô, utilizando os pontos inseridos para movimentação, utilizaremos o seguinte comando em outro terminal Ubuntu: 
```bash 
cd Github/M6T2-Atividades-Caio/semana-3
python3 lecture.py
``` 
