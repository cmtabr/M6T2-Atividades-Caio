#!/usr/bin/env python3

# Import das bibliotecas utilizadas 
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist 
from turtlesim.msg import Pose
from turtlesim.srv import SetPen
from functools import partial

class PendularDraw(Node): # Instancia do nó que sera utilizado para fazer a comunicação 
    def __init__(self):
        super().__init__('pendular_draw') # Chama o construtor da classe Node passando o nome do nó como parâmetro
        
        self.previous_x_ = 0 # Estabelece um valor comparativo para o callback da posição da tartaruga 
         
        self.publisher_ = self.create_publisher(Twist, 'turtle1/cmd_vel', 10) # Cria um publisher para publicar a mensagem Twist no tópico 
                                                                              # 'turtle1/cmd_vel' com taxa de 10 Hz
        
        # Cria um subscriber para receber a mensagem Pose do tópico 'turtle1/pose' com taxa de 10 Hz 
        # e chama a callback pose_callback quando uma nova mensagem for recebida
        self.pose_subscriber_ = self.create_subscription(Pose, 'turtle1/pose', self.pose_callback, 10) 
        
        self.get_logger().info(f' Pendular Draw Node has been started') # Imprime uma mensagem informando que o nó foi iniciado
        
        self.twist_msg_ = Twist() # Cria uma instância da mensagem Twist
        
    # Callback que é chamada quando uma mensagem Pose é recebida
    def pose_callback(self, pose: Pose):
        if pose.x > 8.0 or pose.x < 3.0 or pose.y > 8.0 or pose.y < 3.0:
            self.twist_msg_.linear.x = 1.0
            self.twist_msg_.angular.z = 0.9
        elif pose.x < 6 or pose.x > 5 or pose.y < 6 or pose.y> 5:
            self.twist_msg_.linear.x = 2.0 
            self.twist_msg_.angular.z = 0.9
        else:
            self.twist_msg_.linear.x = 5.0
            self.twist_msg_.angular.z = -1.0
        self.publisher_.publish(self.twist_msg_) # Publica a mensagem Twist para controlar o movimento do robô

        if pose.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Vemelho")
            self.call_set_pen_service(255, 0, 0, 3, 0) # Chamando o serviço SetPen para definir a cor da linha como vermelha
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Branco")
            self.call_set_pen_service(255, 255, 255, 3, 0) # Chamando o serviço SetPen para definir a cor da linha como branca
    
    # Método auxiliar que chama o serviço SetPen
    def call_set_pen_service(self, r, g, b, width, off):
        # Criando cliente para o serviço SetPen
        client = self.create_client(SetPen, 'turtle1/set_pen')
        while not client.wait_for_service(1.0):
            self.get_logger().info('service not available, waiting again...')
        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        
        future.add_done_callback(partial(self.callback_set_pen))
        
    # Callback que é chamada quando a chamada ao serviço SetPen é concluída
    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().info('Service call failed %r' % (e,))
        
def main(args=None):
    rclpy.init(args=args)
    pendular_draw = PendularDraw()
    rclpy.spin(pendular_draw)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
