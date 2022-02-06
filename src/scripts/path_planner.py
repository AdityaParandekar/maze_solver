#!/usr/bin/env python3

#Following is the import line for importing the obstacle detection methods i.e. isValidPoint()
import obstacle_detection as obsdet
import rospy
import random
import numpy as np
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import matplotlib.pyplot as plt

global rate

rospy.init_node('path_planner', anonymous = True)
pub = rospy.Publisher('/path', Path, queue_size = 1)
rate = rospy.Rate(50)                       # 50 Hz



class Node:
    def __init__(self, x, y, parent = None ):
        self.x, self.y = x, y
        self.parent = parent 
        self.distance = 0

    def __eq__(self, o: object) -> bool:
        return self.x == o.x and self.y == o.y



class RRT():
    def __init__(self, start, goal, threshold, max_iter=20000):   #50000
        self.start = start
        self.goal = goal
        self.threshold = threshold
        self.nodes = []
        self.reached = False
        self.max_iter = max_iter
        self.nodes= []
        
    def distance(self, point1, point2):
        distance = np.sqrt((point1.x - point2.x)**2 + (point1.y - point2.y)**2)
        return distance

    def new_vector(self, point1, point2, threshold):
        vector = Node((point2.x - point1.x), (point2.y - point1.x))
        vector.x, vector.y = vector.x/np.sqrt((vector.x**2 + vector.y**2)), vector.y/np.sqrt((vector.x**2 + vector.y**2))
        vector.x, vector.y = point1.x + vector.x * threshold, point1.y + vector.y * threshold
        return vector

    def get_random_node(self):
        new_node = new_node = Node(random.uniform(-6, 6) , random.uniform(-3, 3))
        if self.distance(new_node, self.goal) < self.threshold:     # if the random point is in threshold region around goal
            new_node = self.goal
            self.goal.parent = new_node
        return new_node

    def search(self):
        self.nodes.append(self.start)
        count = 0
        while count < self.max_iter:
            new_node = self.get_random_node()
            for node in self.nodes:
                node.distance = self.distance(node, new_node)
            self.nodes = sorted(self.nodes, key=lambda node: node.distance)
            nearby_node = self.nodes[0]
            for node in self.nodes:
                node.distance = 0 

            if self.distance(nearby_node, new_node) <= self.threshold:
                if obsdet.isValidPoint(nearby_node.x, nearby_node.y, new_node.x, new_node.y) == True:
                    new_node.parent = nearby_node
                    self.nodes.append(new_node)
                else:
                    pass
            else:
                new_node = self.new_vector(nearby_node, new_node, self.threshold)
                if obsdet.isValidPoint(nearby_node.x, nearby_node.y, new_node.x, new_node.y) == True:
                    new_node.parent = nearby_node
                    self.nodes.append(new_node)
                else :
                    pass

            if new_node == self.goal:
                self.reached == True
                str = "Path found in %s steps" %count
                rospy.loginfo(str)
                break
            count = count + 1
        if count == self.max_iter:
            str = "Not reached"
            rospy.loginfo(str)

    def get_path(self):
        self.search()
        path = [self.nodes[-1]]
        current = self.nodes[-1]
        while True:
            if current.x == self.start.x and current.y == self.start.y:
                path.append(current)
                break
            else:
                path.append(current.parent)
                current = current.parent
        path_planned = [(p.x, p.y) for p in path]
        return path_planned



def tranjectory_path(input_path):
    
    path = Path()
    path.header.frame_id = "/map"
    path.header.stamp = rospy.Time.now()
    for pt in input_path:
        pose = PoseStamped()
        pose.pose.position.x = pt[0]
        pose.pose.position.y = pt[1]
        pose.pose.position.z = 0
        path.poses.append(pose)
    
    return path

if __name__ == "__main__" :
    start = Node(-5.2, -2.6)
    goal = Node(3.2, 0.27)
    rrt = RRT(start, goal, 0.5)
    path = rrt.get_path()
    path = path[::-1]
    cmd_traj = tranjectory_path(path)

    ## Printing and plotting the path for debugging
    # print(path)
    # display_maze = [obsdet.Wall(-5.191, 0.9886, 1, 0.15), obsdet.Wall(-5.639, -0.8309, 0.15, 3.769200), obsdet.Wall(-5.672, 1.785, 0.15, 1.597130), obsdet.Wall(-4.957, 2.543, 1.597130, 0.15), obsdet.Wall(-4.277, 2.007956, 0.15, 1.169920),obsdet.Wall(-0.0037, 2.51, 8.729630, 0.15), obsdet.Wall(-1.588, 1.8136, 0.15, 1.25), obsdet.Wall(-1.588, 0.0886, 0.15, 2.5), obsdet.Wall(-2.138, 1.26, 1.25, 0.15), obsdet.Wall(-2.668, 0.7136, 0.15, 1.25), obsdet.Wall(-3.488, 0.16, 1.75, 0.15), obsdet.Wall(2.405, 0.656, 0.75, 0.15), obsdet.Wall(2.705, 0.956, 0.15, 0.75), obsdet.Wall(3.2522, 1.2566, 1.25, 0.15), obsdet.Wall(3.80526, 0.2066, 0.15, 2.25), obsdet.Wall(3.3802, -0.844, 1, 0.15), obsdet.Wall(2.955, -0.5433, 0.15, 0.75), obsdet.Wall(2.7802, -0.2433, 0.5, 0.15), obsdet.Wall(2.605, -0.5433, 0.15, 0.75), obsdet.Wall(4.301, 2.189, 0.15, 0.810003), obsdet.Wall(4.975, 2.5196, 1.50, 0.15), obsdet.Wall(5.711, 1.998, 0.15, 1.192330), obsdet.Wall(5.306, 1.463, 0.919672, 0.15), obsdet.Wall(5.698, 0.301, 0.15, 2.276490), obsdet.Wall(5.185, -0.885, 1.119670, 0.15),obsdet.Wall(4.7, -1.296, 0.15, 0.982963),obsdet.Wall(5.67, -1.7033, 0.15, 1.75),obsdet.Wall(5.154, -2.521, 1.185380, 0.15),obsdet.Wall(0.673, -2.534, 7.883080, 0.15),obsdet.Wall(1.906, -1.93, 0.15, 1.206910),obsdet.Wall(0.877, -1.7, 0.15, 1.719980),obsdet.Wall(0.2502, -0.917, 1.50, 0.15),obsdet.Wall(-0.433, -1.389, 0.15, 1.072),obsdet.Wall(-0.4292, -0.4799, 0.15, 0.927565),obsdet.Wall(0.9177, 0.2156, 0.15, 2.416050),obsdet.Wall(0.23527, 1.3486, 1.5, 0.15),obsdet.Wall(-0.439, 1.048, 0.15, 0.75),obsdet.Wall(-3.2627, -1.72, 0.15, 1.75),obsdet.Wall(-3.883, -0.9203, 1.414750, 0.15),obsdet.Wall(-3.9377, -2.52, 1.5, 0.15),obsdet.Wall(-4.615, -2.157, 0.15, 0.870384),obsdet.Wall(2.105, 1.58, 0.15, 2.15893)]
    # for wall in display_maze:
    #     x, y = wall.polygon.exterior.xy
    #     plt.plot(x, y)
    # for pt in path:
    #     plt.plot(pt[0],pt[1], 'bo')
    # plt.show()


    # publishing just once to the node subscribing

    try:
        while not rospy.is_shutdown():
            connections = pub.get_num_connections()
            # rospy.loginfo('Connections: %d', connections)
            if connections > 0:
                pub.publish(cmd_traj)
                rospy.loginfo('Published')
                break
            rate.sleep()
        # rospy.spin()
    except rospy.ROSInterruptException:
        pass