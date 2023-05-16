#! /usr/bin/env python3 

import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

from tf_transformations import euler_from_quaternion

from queue import Queue

class MovingRobot(Node):
    
    def __init__(self, simulation_timer=0.05):

        super().__init__('moving_robot')

        # Assinatura no tópico 'odom' para receber as informações de posição
        self.pose_subscription = self.create_subscription(
            msg_type=Odometry,
            topic='odom',
            callback=self.position_callback,
            qos_profile=10
        )

        # Publicador para enviar os comandos de velocidade
        self.publisher_ = self.create_publisher(
            msg_type=Twist,
            topic='cmd_vel',
            qos_profile=10
        )
        
        # Timer para controlar a frequência de atualização
        self.timer_ = self.create_timer(
            simulation_timer, 
            self.updated_point_callback
        )
        
        # Instnacia da mensagem de comando de velocidade
        self.twist_msg_ = Twist()
        
        # Variável para armazenar a posição atual
        self.current_pose_ = []

        # Variável para armazenar o próximo ponto da fila
        self.path_points_ = None

        # Fila de posição dos pontos a serem percorridos
        self.position_queue_ = Queue()


    def point_callback(self):
        """
        Essa função é chamada para fazer publicações no tópico 
        'cmd_vel' para movimentação do robo
        """
        
        # Verifica se não há próximo ponto na fila
        if self.path_points_ is None:
            # Define os comandos de velocidade como zero para parar o robô
            self.twist_msg_.linear.x = 0.0
            self.twist_msg_.angular.z = 0.0
        else:  
            # Calculo da diferença no eixo X e Y entre o próximo ponto e 
            # a posição atual do robô     
            dx = self.path_points_[0] - self.current_pose_[0]
            dy = self.path_points_[1] - self.current_pose_[1]
            
            # Calculo da distância entre dois pontos, o atual e 
            # o que se pretende chegar
            dist = math.sqrt(dx ** 2 + dy ** 2)

            # Calculo do ângulo de orientação para o próximo ponto em 
            # relação à posição atual do robô no intervalo de -π a π
            ang = math.atan2(dy, dx) - self.current_pose_[2]

            # Calculo da diferença angular considerando a rotação mais 
            # curta em um intervalo de 
            ang_diff = math.atan2(math.sin(ang), math.cos(ang))

            # Definição da velocidade linear como a menor entre 
            # a distância e 1.0
            linear_velocity = min(dist, 1.0) 

            # Definição da velocidade angular como a diferença 
            # entre os angulos
            angular_velocity = ang_diff

            # Atualização dos comandos de velocidade no tópico
            # twist com as velocidades calculadas
            self.twist_msg_.linear.x = linear_velocity
            self.twist_msg_.angular.z = angular_velocity

            # Verificação de distância, se o o ponto atual 
            # dista à menos de 0.1 da distância real do objetivo, 
            # então, o a variável 'path_points_' é esvaziada e o 
            # callback de position sobrescreve-a com o próximo 
            # valor na fila 
            if dist < 0.1:  
                self.path_points_ = None

        # publica as informações no tópico 
        self.publisher_.publish(self.twist_msg_)

    def position_callback(self, msg):
        """
        Essa função é chamada para fazer a leitura das informações 
        de posição do tópico 'odom' no qual a classe está inscrita
        """
        x = msg.pose.pose.position.x # Posição no eixo x
        y = msg.pose.pose.position.y # Posição no eixo y
        z = msg.pose.pose.position.z # Posição no eixo z

        # Obtem-se a orientação do robô a partir da mensagem recebida
        orientation = msg.pose.pose.orientation

        # Tranforma-se a os quarternions recebidos do tópicos em 
        # ângulos de Euler
        _, _, theta = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        
        # Variável que recebe as posições atuais do robô 
        self.current_pose_ = [x, y, theta]

        # Verifica se não há um próximo ponto na fila e se a fila de 
        # posições não está vazia
        if self.path_points_ is None and not self.position_queue_.empty():
            # Pega o próximo item da fila e retorna ele para o robo como 
            # sendo seu próximo ponto a ser processado 
            self.path_points_ = self.position_queue_.get()

    def get_user_input(self):
        while True: # Loop que pega as coordenadas x e y que o 
                    # usuário deseja percorrer com o robo
            x = float(input("Manda uma coordenada no eixo 'X' aí: "))
            y = float(input("Manda uma coordenada no eixo 'Y' aí: "))

            # Enderaça na fila uma nova lista com os pontos que o usuário
            # cadastrou no terminal 
            self.position_queue_.put([float(x), float(y)])

            # Verifica se o usuário quer continuar a adicionar pontos 
            # através de um input simples que se verificado quebra o loop
            response = input("Quer adicionar mais pontos? (s/n): ")
            if response.lower() == "n":
                break


    def updated_point_callback(self):
        """
        Essa função é chamada para atualizar a função point_callback
        para movimentação do robo, em intervalos de 50 milisegundos. Desta
        forma a função supramencionada só é atividade quando a posição do 
        robo é conhecida, para evitar possíveis erros
        """
        # Verifica se a posição atual do robô está vazia
        if len(self.current_pose_) == 0:
            return

        self.point_callback()

def main(args=None):

    rclpy.init(args=args)
    
    moving_robot = MovingRobot()
    
    moving_robot.get_user_input()
    
    rclpy.spin(moving_robot)
    
    moving_robot.destroy_node()
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()