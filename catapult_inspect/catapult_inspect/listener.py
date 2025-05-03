import rclpy
from rclpy.node import Node
from rclpy.duration import Duration, Infinite
from rclpy.time import Time
import rclpy.qos as qos
from rclpy.qos_event import SubscriptionEventCallbacks

from control_msgs.action._follow_joint_trajectory import FollowJointTrajectory_FeedbackMessage

from matplotlib.animation import FuncAnimation
import matplotlib.pyplot as plt

import signal
import socket
import numpy as np


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            FollowJointTrajectory_FeedbackMessage,
            '/custom_trajectory_controller/follow_joint_trajectory/_action/feedback',
            self.listener_callback,
            10)
        
        self.subscription  # prevent unused variable warning
        print("contructed")

        self.fig, ((self.ax_pos, self.ax_vel), (self.ax_pos_err, self.ax_vel_err)) = plt.subplots(2,2)
        self.pos_desired_plotter = DataPlotter(self.ax_pos, 'qd', fmt='--', lw=3)
        self.pos_actual_plotter = DataPlotter(self.ax_pos, 'q', fmt='-', lw=1.5)
        self.vel_desired_plotter = DataPlotter(self.ax_vel, 'dqd', fmt='--', lw=3)
        self.vel_actual_plotter = DataPlotter(self.ax_vel, 'dq', fmt='-', lw=1.5)

        self.pos_err_plotter = DataPlotter(self.ax_pos_err, 'e', lw=2)
        self.vel_err_plotter = DataPlotter(self.ax_vel_err, 'de', lw=2)


    def listener_callback(self, msg):
        # update position subplot
        if self.ax_pos.get_legend() is not None:
            self.ax_pos.get_legend().remove()

        self.pos_desired_plotter.update(msg.feedback.desired.positions, msg.feedback.header.stamp)
        self.pos_actual_plotter.update(msg.feedback.actual.positions, msg.feedback.header.stamp)
        self.ax_pos.set_title('Joint Positions vs Time')
        self.ax_pos.set_xlabel('time [s]')
        self.ax_pos.set_ylabel('position [rad]')
        self.ax_pos.legend(loc='upper left')

        # update velocity subplot
        if self.ax_vel.get_legend() is not None:
            self.ax_vel.get_legend().remove()
        self.vel_desired_plotter.update(msg.feedback.desired.velocities, msg.feedback.header.stamp)
        self.vel_actual_plotter.update(msg.feedback.actual.velocities, msg.feedback.header.stamp)
        self.ax_vel.set_title('Joint Velocities vs Time')
        self.ax_vel.set_xlabel('time [s]')
        self.ax_vel.set_ylabel('velocity [rad/s]')
        self.ax_vel.legend()

        # update position error subplot
        if self.ax_pos_err.get_legend() is not None:
            self.ax_pos_err.get_legend().remove()
        pos_errs = np.array(msg.feedback.desired.positions)-np.array(msg.feedback.actual.positions)
        self.pos_err_plotter.update(pos_errs, msg.feedback.header.stamp)
        self.ax_pos_err.set_title('Joint Position Errors vs Time')
        self.ax_pos_err.set_xlabel('time [s]')
        self.ax_pos_err.set_ylabel('position error [rad]')
        self.ax_pos_err.legend()

        # update velocity error subplot
        if self.ax_vel_err.get_legend() is not None:
            self.ax_vel_err.get_legend().remove()
        vel_errs = np.array(msg.feedback.desired.velocities)-np.array(msg.feedback.actual.velocities)
        self.vel_err_plotter.update(vel_errs, msg.feedback.header.stamp)
        self.ax_vel_err.set_title('Joint Velocity Errors vs Time')
        self.ax_vel_err.set_xlabel('time [s]')
        self.ax_vel_err.set_ylabel('velocity error [rad/s]')
        self.ax_vel_err.legend()

        
        plt.pause(0.001)

    def savePlot(self):
        plt.savefig('pos-plot.png')
        print('saved plot')


class DataPlotter:
    def __init__(self, ax, label_desc, fmt='-', lw=1):
        self.ax = ax
        self.desc = label_desc
        self.fmt = fmt
        self.lw = lw
        
        self.lines = []
        # self.labels = []
        self.num_joints = 6
        self.joint_data = [[] for _ in range(self.num_joints)]
        self.time_data = [] # seconds since start
        self.start_time = None

    def update(self, new_joint_data, time_from_start):
        time = Time.from_msg(time_from_start)
        if self.start_time is None:
            self.start_time = time
        
        self.time_data.append((float(time.nanoseconds) - float(self.start_time.nanoseconds))*1e-9)
        for i in range(self.num_joints):
            self.joint_data[i].append(new_joint_data[i])

        # remove lines from plot
        for line in self.lines:
            line.remove()
        self.lines = []

        # create new lines for plot
        for i in range(self.num_joints):
            x = self.time_data
            y = self.joint_data[i]
            line = self.ax.plot(x, y, self.fmt, linewidth=self.lw, label='{}[{}]'.format(self.desc, i))[0]
            self.lines.append(line)
        

minimal_subscriber = None
def keyboardInt(sig, frame):
    print("keyboardInt")
    # save the graph
    if minimal_subscriber is not None:
        minimal_subscriber.savePlot()
    

def main(args=None):
    signal.signal(signal.SIGINT, keyboardInt)
    
    rclpy.init(args=args)

    global minimal_subscriber
    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
