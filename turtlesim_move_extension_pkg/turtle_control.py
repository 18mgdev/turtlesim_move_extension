import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen, TeleportAbsolute
from std_srvs.srv import Empty
from pynput import keyboard
from pynput.keyboard import Key

class TurtleControl(Node):
    def __init__(self):
        super().__init__('turtle_control')

        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.clear_client = self.create_client(Empty, '/clear')
        self.reset_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')

        #parametros dinamicos
        self.declare_parameter('log_level', 'INFO')  #logging level
        self.declare_parameter('speed', 2.0)  #turtle speed

        #parametros
        log_level = self.get_parameter('log_level').get_parameter_value().string_value.upper() #valor log tipo STRING
        self.vel = self.get_parameter('speed').get_parameter_value().double_value #valor speed tipo double
        self.twist = Twist()
        self.is_painting = True
        self.set_logger_level(log_level)


        self.get_logger().info('-CONTROLES-\nW,A,S,D: movimiento\nSPACE: alternar pintado\nC: limpiar pintado\nR: reset posición')

        # Listener de teclas
        self.active_keys = set()
        self.listener = keyboard.Listener(on_press=self.on_press, on_release=self.on_release)
        self.listener.start()

        self.create_timer(1.0, self.update_parameters)


    def on_press(self, key):
        try:
            if key.char not in self.active_keys:
                self.active_keys.add(key.char)
            self.make_action()
        except AttributeError:
            if key == Key.space and 'SPACE' not in self.active_keys:
                self.active_keys.add('SPACE')
            self.make_action()

    def on_release(self, key):
        try:
            if key.char in self.active_keys:
                self.active_keys.remove(key.char)
            self.make_action()
        except AttributeError:
            if key == Key.space and 'SPACE' in self.active_keys:
                self.active_keys.remove('SPACE')
            self.make_action()

    def make_action(self):
        self.twist.linear.x = 0.0
        self.twist.angular.z = 0.0

        if 'w' in self.active_keys:
            self.twist.linear.x = self.vel
        if 's' in self.active_keys:
            self.twist.linear.x = -self.vel
        if 'a' in self.active_keys:
            self.twist.angular.z = self.vel
        if 'd' in self.active_keys:
            self.twist.angular.z = -self.vel

        self.publisher_.publish(self.twist)

        if 'c' in self.active_keys:
            self.clear_screen()
            self.active_keys.remove('c')
        if 'SPACE' in self.active_keys:
            self.toggle_painting()
            self.active_keys.remove('SPACE')
        if 'r' in self.active_keys:
            self.reset_turtle()
            self.active_keys.remove('r')

    def toggle_painting(self):
        self.is_painting = not self.is_painting
        pen_request = SetPen.Request()

        if not self.is_painting:
            self.get_logger().info('Painting desactivado.')
            pen_request.r = 0
            pen_request.g = 0
            pen_request.b = 0
            pen_request.width = 0
            pen_request.off = 1
        else:
            self.get_logger().info('Painting activado.')
            pen_request.r = 179
            pen_request.g = 184
            pen_request.b = 255
            pen_request.width = 3
            pen_request.off = 0

        if self.set_pen_client.service_is_ready():
            self.set_pen_client.call_async(pen_request)

    def clear_screen(self):
        if self.clear_client.service_is_ready():
            self.clear_client.call_async(Empty.Request())
            self.get_logger().info('Screen clear realizado.')

    def reset_turtle(self):
        change_painting=False
        if self.reset_client.service_is_ready():
            if self.is_painting: #desactiva el pintado en caso de estar activado antes del teleport
                self.toggle_painting()
                change_painting=True
            reset_request = TeleportAbsolute.Request()
            reset_request.x = 5.5
            reset_request.y = 5.5
            reset_request.theta = 0.0
            if change_painting:
                self.toggle_painting()
            self.reset_client.call_async(reset_request)
            self.get_logger().info('Reseteo de posicion realizado.')

    def set_logger_level(self, level):
        log_levels = {
            'DEBUG': rclpy.logging.LoggingSeverity.DEBUG,
            'INFO': rclpy.logging.LoggingSeverity.INFO,
            'WARN': rclpy.logging.LoggingSeverity.WARN,
            'ERROR': rclpy.logging.LoggingSeverity.ERROR,
            'FATAL': rclpy.logging.LoggingSeverity.FATAL
        }
        if level in log_levels:
            self.get_logger().set_level(log_levels[level])
        else:
            self.get_logger().warn(f"Log level '{level}' desconocido, cambiando a INFO.")
            self.get_logger().set_level(rclpy.logging.LoggingSeverity.INFO)

    def update_parameters(self):
        # Update speed dynamically if the parameter changes
        new_speed = self.get_parameter('speed').get_parameter_value().double_value
        if new_speed != self.vel:
            self.vel = new_speed
            self.get_logger().info(f'Valor velocidad actualizado a: {self.vel}')

        # Update log level dynamically if it changes
        log_level = self.get_parameter('log_level').get_parameter_value().string_value.upper()
        self.set_logger_level(log_level)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.listener.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
