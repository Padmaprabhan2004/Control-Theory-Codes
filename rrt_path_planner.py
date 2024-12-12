import math
import random
import matplotlib.pyplot as plt
import numpy as np

show_animation=True


class Node:
    def __init__(self,x,y):
        self.x=x
        self.y=y
        self.path_x=[]
        self.path_y=[]
        self.parent=None

class AreaBounds:
    def __init__(self,area):
        self.xmin=float(area[0])
        self.xmax=float(area[1])
        self.ymin=float(area[2])
        self.ymax=float(area[3])

class RRT:


    def __init__(self,start,goal,obstacle_list,rand_area,
                 expand_dis=3.0,path_resolution=0.5,goal_sample_rate=5,max_iter=500,
                 play_area=None,robot_radius=0.0,):
        self.start=Node(start[0],start[1])
        self.goal=Node(goal[0],goal[1])
        self.min_rand=rand_area[0]
        self.max_rand=rand_area[1]
        if play_area is not None:
            self.play_area=AreaBounds(play_area)
        else:
            self.play_area=None
        self.obstacle_list=obstacle_list
        self.expand_dis=expand_dis
        self.path_resolution=path_resolution
        self.goal_sample_rate=goal_sample_rate
        self.max_iter=max_iter
        self.node_list=[]
        self.robot_radius=robot_radius
    
    def planning(self,animation=True):
        self.node_list=[self.start]
        for i in range(self.max_iter):
            node=self.sample_node()
            idx,nearest_nbr=self.get_nearest_nbr(node)
            new_node=self.steer(nearest_nbr,node,self.expand_dis)
            if self.check_if_outside_play_area(new_node,self.play_area) and self.check_collision(new_node,self.obstacle_list,self.robot_radius):
                self.node_list.append(new_node)
            
            if animation:
                self.draw_graph(new_node)
            
            #if goal is found
            if self.calc_dist_to_goal(new_node.x,new_node.y)<=self.expand_dis:
                final_node=self.steer(new_node,self.goal,self.expand_dis)
                if self.check_collision(final_node,self.obstacle_list,self.robot_radius):
                    return self.generate_final_trajectory(len(self.node_list)-1)
    
    def generate_final_trajectory(self,goal_ind):
        path=[[self.goal.x,self.goal.y]]
        node=self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x,node.y])
            node=node.parent
        path.append([node.x,node.y])
        return path
    
    def calc_dist_to_goal(self,x,y):
        dx=x-self.goal.x
        dy=y-self.goal.y
        return math.hypot(dx,dy)

    def draw_graph(self, rnd=None):
        plt.clf()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        if rnd is not None:
            plt.plot(rnd.x, rnd.y, "^k")
            if self.robot_radius > 0.0:
                self.plot_circle(rnd.x, rnd.y, self.robot_radius, '-r')
        for node in self.node_list:
            if node.parent:
                plt.plot(node.path_x, node.path_y, "-g")

        for (ox, oy, size) in self.obstacle_list:
            self.plot_circle(ox, oy, size)

        if self.play_area is not None:
            plt.plot([self.play_area.xmin, self.play_area.xmax,
                      self.play_area.xmax, self.play_area.xmin,
                      self.play_area.xmin],
                     [self.play_area.ymin, self.play_area.ymin,
                      self.play_area.ymax, self.play_area.ymax,
                      self.play_area.ymin],
                     "-k")

        plt.plot(self.start.x, self.start.y, "xr")
        plt.plot(self.goal.x, self.goal.y, "xr")
        plt.axis("equal")
        plt.axis([self.min_rand, self.max_rand, self.min_rand, self.max_rand])
        plt.grid(True)
        plt.pause(0.01)        
        
    def calculate_d_theta(self,from_node,to_node):
        dx=to_node.x-from_node.x
        dy=to_node.y-from_node.y
        d=math.hypot(dx,dy)
        theta=math.atan2(dy,dx)
        return d,theta
    
    def steer(self,from_node,to_node,extend_length=float('inf')):
        new_node=Node(from_node.x,from_node.y)
        d,theta=self.calculate_d_theta(new_node,to_node)
        new_node.path_x=[new_node.x]
        new_node.path_y=[new_node.y]

        if extend_length>d:
            extend_length=d
        n_expand=math.floor(extend_length/self.path_resolution)

        for _ in range(n_expand):
            new_node.x+=self.path_resolution*math.cos(theta)
            new_node.y+=self.path_resolution*math.sin(theta)
            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)
        
        #final call to check whether path is in new_node or not
        d,_=self.calculate_d_theta(new_node,to_node)
        if d<=self.path_resolution: #loop wouldnt have handled this, so do manually
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x=to_node.x
            new_node.y=to_node.y
        
        new_node.parent=from_node
        return new_node

    def check_if_outside_play_area(self,node, play_area):

        if play_area is None:
            return True  # no play_area was defined, every pos should be ok

        if node.x < play_area.xmin or node.x > play_area.xmax or \
           node.y < play_area.ymin or node.y > play_area.ymax:
            return False  # outside - bad
        return True
    
    def check_collision(self,node,obstacle_list,robot_radius):
        if node is None:
            return False
        for (ox,oy,radius) in obstacle_list:
            dx_list=[ox-x for x in node.path_x]
            dy_list=[oy-y for y in node.path_y]
            d_list=[dx**2+dy**2 for (dx,dy) in zip(dx_list,dy_list)]
            if min(d_list)<=(radius+robot_radius)**2:
                return False
        return True


    @staticmethod
    def plot_circle(x, y, size, color="-b"):  # pragma: no cover
        deg = list(range(0, 360, 5))
        deg.append(0)
        xl = [x + size * math.cos(np.deg2rad(d)) for d in deg]
        yl = [y + size * math.sin(np.deg2rad(d)) for d in deg]
        plt.plot(xl, yl, color)

    def sample_node(self):
        if random.randint(0,100)>self.goal_sample_rate:
            rnd=Node(random.uniform(self.min_rand,self.max_rand),random.uniform(self.min_rand,self.max_rand))
        else:
            rnd=self.goal
        return rnd
    
    def get_nearest_nbr(self,sampled_node):
        node=None
        idx=None
        dist=float('inf')
        for i in range(len(self.node_list)):
            node_=self.node_list[i]
            dist_=math.sqrt((sampled_node.x-node_.x)**2+(sampled_node.y-node_.y)**2)
            if(dist_<dist):
                dist=dist_
                node=node_
                idx=i
        return i,node_
    
def main(gx=6, gy=6.0):
    print("start " + __file__)

    # ====Search Path with RRT====
    obstacleList = [(3, 3, 1), (4, 7, 2), (5, 8, 2), (1, 10, 2), (7, 6, 1),
                    (10, 4, 1), (4, 10, 1)]  # [x, y, radius]
    # Set Initial parameters
    rrt = RRT(
        start=[0, 0],
        goal=[gx, gy],
        rand_area=[-2, 15],
        obstacle_list=obstacleList,
        # play_area=[0, 10, 0, 14]
        robot_radius=0.8
        )
    path = rrt.planning(animation=show_animation)

    if path is None:
        print("Cannot find path")
    else:
        print("found path!!")

        # Draw final path
        if show_animation:
            rrt.draw_graph()
            plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
            plt.grid(True)
            plt.pause(0.01)  # Need for Mac
            plt.show()


if __name__ == '__main__':
    main()
