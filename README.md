# Atividades
> Repositório dedicado às atividades de programação do módulo 6 da turma de engenharia da computação.

## Semana 1 
Como solicitado no autoestudo "Turtlesim: simulando um ambiente robótico integrado no ROS", abaixo vê-se o vídeo relativo a configuração do ambiente ROS. 

https://user-images.githubusercontent.com/99201276/234757012-6822c6b9-873f-4dc9-9bbc-f3e89bd3815e.mp4

[LINK para o vídeo no Youtube](https://www.youtube.com/shorts/1VSrzrdVP3U)

Como é possível ubservar, utilizando o Windows Subsystem for Linux (WSL), foi criado um ambiente Linux e neste, utilizando o sistema operacional Ubuntu, pudemos fazer a execução dos pacotes do Software Robot Operating System (ROS), para a o desenvolvimento com robôs, requisito para a progressão deste módulo do curso. 

Adicionando a chave GPG do ROS2 para facilitar a integração dos pacotes com o sistema operacional Ubuntu, com os seguintes comandos:
```bash Bash
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
``` 

Então adiciona-se o repositório de referência do ROS2 a lista de repositórios do Linux, através do comando abaixo: 
```bash 
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
``` 

Atualizamos o Advanced Package Tool (APT) para atualizar os caches de todos os repositórios depois de configurá-los e verificamos se o sistema Ubuntu está atualizado em decorrência da frequência com a qual o ROS2 é aprimorado, através dos seguintes comandos: 
```bash 
sudo apt update
sudo apt upgrade
``` 

Então por final, instalaremos os pacotes e bibliotecas referentes ao próprio ROS, de modo que possamos utilizar o sistema operacional em sua plena capacidade, utilizando os seguintes comandos para tal: 
```bash 
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base
sudo apt install ros-dev-tools
``` 

Agora para testar se os pacotes foram corretamente instalados, para além do vídeo acima podemos fazer um teste configurando um talker e um listener para identificar se há conexão quando pretendemos estabelecer um nó com o ROS, para tal utilizaremos os seguintes comandos para o talker e para o listener, respectivamente: 
>```bash 
>source /opt/ros/humble/setup.bash
>ros2 run demo_nodes_cpp talker
>``` 

>```bash 
>source /opt/ros/humble/setup.bash
>ros2 run demo_nodes_py listener
>``` 

Obtendo o seguinte resultado: 
![image](https://user-images.githubusercontent.com/99201276/232633815-8788e458-f2e9-4f5b-8485-d37225c034e4.png)
> Como ambos não foram iniciados ao mesmo tempo, não vemos correspondência para as primeiras mensagens do talker, contudo vemos que as demais mensagens que se seguem são recebidas pelo listener.

### Conclusão: 
A atividade requisita que testes sejam feitos utilizando o Turtlesim, desta forma, para provar a capacidade do aluno de interagir com um sistema operacional de robôs utilizamos o código a seguir para abrir a interface gráfica do turtlesim:
```bash 
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
``` 

Então, para que possamos movimentar a tartaruga, utilizando um código pré-definido para movimentação, utilizaremos o seguinte comando em outro terminal Ubuntu: 
```bash 
source /opt/ros/humble/setup.bash
cd Github/M6T2-Atividades-Caio/semana-1
python3 exercise.py
``` 
Obtendo assim o resultado como mostrado no vídeo anteriormente.

Por fim, a imagem que se segue é aquela gerada pelo script python:

![Print do Pendulo](https://user-images.githubusercontent.com/99201276/234754627-31efeeb3-a541-46f2-81d5-de1015d4c1dc.png)

## Semana 3 
Como solicitado no autoestudo "Exercício estruturas de dados", abaixo vê-se o vídeo relativo a iniciação do ambiente de simulação, Gazebo, com os packages do TurtleBot3. 

### Objetivo

Demonstrar a capacidade de interagir de forma dinâmica com uma plataforma de simulação utilizando os conceitos de estrutura de dados e programação orientada à objetos, outrossim utilizando os protocolos de comunicação via publicação e subscrição em tópicos com os parâmetros de mensagens definidas (https://github.com/rticommunity/ros-data-types).

### Solução 
O código apresentado implementa um nó chamado MovingRobot, que herda a classe Node do ROS 2.
Ele se inscreve no tópico **`odom`** para obter a posição atual do robô e utiliza uma fila para receber os pontos de destino fornecidos pelo usuário. 
O nó calcula as velocidades linear e angular necessárias para mover o robô em direção aos pontos de destino. O método **point_callback** é responsável por esse cálculo e publicação das velocidades no tópico **`cmd_vel`**. 
O método **position_callback** atualiza a posição atual do robô quando uma nova mensagem Odometry é recebida. 
O método **get_user_input** obtém os pontos de destino do usuário. A função main é responsável por inicializar e executar o nó. 
Em resumo, o código cria um nó ROS 2 que permite ao usuário fornecer pontos de destino para controlar o movimento do robô.

### Vídeo 
Abaixo você pode ver o vídeo da simulação funcionando com o código implementado

