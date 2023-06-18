#!/usr/bin/env python3

import rospy
import numpy as np
import matplotlib.pyplot as plt
import collections
from matplotlib.animation import FuncAnimation
from matplotlib.lines import Line2D

from vanttec_msgs.msg import Trajectory, EtaPose
from geometry_msgs.msg import Vector3, Twist

class PlotTrajectory:
    def __init__(self):
        rospy.init_node('Trajectory_Plotter', anonymous=True)
        rospy.Subscriber("/uuv_motion_planning/trajectory_planner/s_curve", Trajectory, self.update_trajectory)
        rospy.Subscriber("/vectornav/ins_3d/ins_acc", Vector3, self.update_accel)
        rospy.Subscriber("/uuv_simulation/dynamic_model/vel", Twist, self.update_vel)
        rospy.Subscriber("/uuv_simulation/dynamic_model/eta_pose", EtaPose, self.update_pose)

        self.ref_trajectory = Trajectory()

        self.N = 100  # Number of elements in the path
        self.zero_pased = False
        self.traj_arrived = False
        self.step = 100 #ms

        # Generate random path
        self.traj_path = np.random.randint(0, 10, self.N)
        self.path = [np.array([]), np.array([]), np.array([])]

        # traversed_path = np.empty(1)
        self.lines = [None, None, None]  # Lines for robot positions

        # Generate time array
        self.time = np.arange(0,self.N,self.step/1000)

        # Start collections with zeros
        # self.robot_position = collections.deque(np.zeros(N))
        self.robot_position = [collections.deque(), collections.deque(), collections.deque()]
        self.robot_vel = [collections.deque(), collections.deque(), collections.deque()]
        self.robot_accel = [collections.deque(), collections.deque(), collections.deque()]
        self.pose_msg = [0, 0, 0]
        self.vel_msg = [0, 0, 0]
        self.accel_msg = [0, 0, 0]
        self.kinem_msg_arrived = [False, False, False]

        self.dofs = ['x','y','z']

        # Define and adjust figure
        self.fig, self.axs = plt.subplots(3, 1, figsize=(15, 9))
        self.fig.subplots_adjust(hspace=0.5)

        for idx, ax in enumerate(self.axs):
            ax.set_facecolor('#DEDEDE')
            self.lines[idx] = Line2D([], [], color='r', label='Robot position ' + self.dofs[idx])
            ax.add_line(self.lines[idx])


    def update_trajectory(self, msg):
        if not self.traj_arrived:
            self.ref_trajectory = msg
            self.N = len(self.ref_trajectory.eta_pose)
            self.traj_path = self.ref_trajectory.eta_pose
            self.time = np.arange(0,self.N,self.step/1000)

            self.path = [np.array([]), np.array([]), np.array([])]

            for elem in self.traj_path:
                self.path[0] = np.append(self.path[0],elem.x)
                self.path[1] = np.append(self.path[1],elem.y)
                self.path[2] = np.append(self.path[2],elem.z)

            # print(len(self.time))
            # print(len(self.path[0]))

            self.traj_arrived = True

    def update_accel(self, msg):
        self.kinem_msg_arrived[0] = True
        self.accel_msg[0] = msg.x
        self.accel_msg[1] = msg.y
        self.accel_msg[2] = msg.z
        
    def update_vel(self, msg):
        self.kinem_msg_arrived[1] = True
        self.vel_msg[0] = msg.linear.x
        self.vel_msg[1] = msg.linear.y
        self.vel_msg[2] = msg.linear.z

    def update_pose(self, msg):
        self.kinem_msg_arrived[2] = True
        self.pose_msg[0] = msg.x
        self.pose_msg[1] = msg.y
        self.pose_msg[2] = msg.z

    def update_plot(self,i):
        if(i == 0 and not self.zero_pased):
            self.zero_pased = True
            return self.lines

        self.time_aux = np.array(self.time[:i+1])

        if self.traj_arrived and all(self.kinem_msg_arrived):
            self.robot_position[0].append(self.pose_msg[0])   # x
            self.robot_position[1].append(self.pose_msg[1])   # y
            self.robot_position[2].append(self.pose_msg[2])   # z

            for idx in range(len(self.axs)):
                print("time: ", len(self.time_aux))
                print("line 1: ", len(self.path[idx][:i+1]))
                print("line 2: ", len(self.robot_position[idx]))
                self.lines[0].set_data(self.time_aux, self.path[idx][:i+1])
                self.lines[1].set_data(self.time_aux, self.robot_position[idx])

                self.axs[idx].set_xlim(0, self.time_aux[-1]+0.1)
                self.axs[idx].set_ylim(min(min(self.path[idx][:i+1])-1, min(self.robot_position[idx])-1),
                                        max(max(self.path[idx][:i+1])+1, max(self.robot_position[idx])+1))

        return self.lines

        '''
        if self.traj_arrived and all(self.kinem_msg_arrived):
                # Get data
                # if   idx == 0:
                #     self.robot_position[0].append(self.traj_path[i % self.N].x)
                # elif idx == 1:
                #     self.robot_position[1].append(self.traj_path[i % self.N].y)
                # elif idx == 2:
                #     self.robot_position[2].append(self.traj_path[i % self.N].z)
                
            self.robot_accel[0].append(self.accel_msg[0])   # x
            self.robot_accel[1].append(self.accel_msg[1])   # y
            self.robot_accel[2].append(self.accel_msg[2])   # z
            
            self.robot_vel[0].append(self.vel_msg[0])   # x
            self.robot_vel[1].append(self.vel_msg[1])   # y
            self.robot_vel[2].append(self.vel_msg[2])   # z
            
            self.robot_position[0].append(self.pose_msg[0])   # x
            self.robot_position[1].append(self.pose_msg[1])   # y
            self.robot_position[2].append(self.pose_msg[2])   # z
            
            for idx in range(len(self.axs)):
                # Clear axis
                self.axs[idx].cla()

                # Plot path
                self.axs[idx].plot(self.time_aux, self.path[idx][:i+1], label='Path ' + self.dofs[idx])

                # Plot robot position
                # self.axs[idx].plot(self.time[i], self.robot_position[-1], 'ro', label='Robot position')
                self.axs[idx].plot(self.time_aux, self.robot_position[idx], color='r', label='Robot position ' + self.dofs[idx])

                # Set labels and limits
                self.axs[idx].set_xlabel('Time')
                self.axs[idx].set_ylabel('Position ' + self.dofs[idx])
                self.axs[idx].set_ylim(min(min(self.path[idx][:i+1])-1,min(self.robot_position[idx])-1), max(max(self.path[idx][:i+1])+1,max(self.robot_position[idx])+1))

                # Show legend
                self.axs[idx].legend()

        '''

    def plot_path(self):
        # Animate
        self.ani = FuncAnimation(self.fig, self.update_plot, frames=self.N, interval=self.step, blit=True)
        plt.show()
        rospy.spin()

def main():
    plot = PlotTrajectory()
    plot.plot_path()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass