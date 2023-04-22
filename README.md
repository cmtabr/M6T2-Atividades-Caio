# Atividades
> Repositório dedicado às atividades de programação do módulo 6 da turma de engenharia da computação.

## Semana 1 
Como solicitado no autoestudo "Turtlesim: simulando um ambiente robótico integrado no ROS", abaixo vê-se o vídeo relativo a configuração do ambiente ROS. 

https://user-images.githubusercontent.com/99201276/233801080-4c85679f-b142-444e-9562-f4c20c9f161d.mp4

[LINK para o vídeo no Youtube](https://youtu.be/g1h_C9zxKB0)

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

![Print do Pendulo](https://user-images.githubusercontent.com/99201276/233801425-5848acbc-1382-403a-ae36-d9ad05c8fa68.jpg)
