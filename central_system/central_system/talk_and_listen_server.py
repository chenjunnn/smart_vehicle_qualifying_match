# Copyright (c) 2022 ChenJun
# Licensed under the MIT License.

import rclpy
from rclpy.node import Node

import remi.gui as gui
from remi import start, App

from std_msgs.msg import String


class TalkAndListenServer(Node):
    def __init__(self):
        super().__init__('talk_and_listen_server')
        # Publisher
        self.publisher = self.create_publisher(String, 'from_client', 10)
        # Subscriber
        self.sub = self.create_subscription(String, 'from_server', self.callback, 10)
        self.received_msg = String()

    def callback(self, msg):
        self.received_msg = msg


class MyApp(App):
    def __init__(self, *args):
        super(MyApp, self).__init__(*args)

    def main(self):
        # creating a container VBox type, vertical
        wid = gui.VBox(width=300, height=200)

        # creating a text label to show msg heard from server
        self.lb_1 = gui.Label('Heard from server: ')
        self.lb_2 = gui.Label(width='80%', height='20%', style={"white-space":"pre"})

        # text input from web page
        self.lb_3 = gui.Label('Text to say: ')
        self.text_input = gui.TextInput(width='80%', height='20%')

        # a button for simple interaction
        bt = gui.Button('Send', width=200, height=30)

        # setting up the listener for the click event
        bt.onclick.do(self.on_button_pressed)
        
        # adding the widgets to the main container
        wid.append(self.lb_1)
        wid.append(self.lb_2)
        wid.append(self.lb_3)
        wid.append(self.text_input)
        wid.append(bt)

        self.node = TalkAndListenServer()

        # returning the root widget
        return wid


    # bottom function
    def on_button_pressed(self, emitter):
        msg = String()
        print('Button pressed')
        msg.data = self.text_input.get_value()
        self.node.publisher.publish(msg)
        self.node.get_logger().info('Publish: [%s]' % msg.data)

    def idle(self):
        rclpy.spin_once(self.node, timeout_sec=0.1)
        text = self.node.received_msg.data
        self.lb_2.set_text(text)


def main(args=None):
    rclpy.init(args=args)

    start(MyApp, address='0.0.0.0', port=8080, start_browser=False)


if __name__ == '__main__':
    main()
