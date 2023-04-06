#! /usr/bin/env python
import numpy as np
import math
import cv2

class Cell:
    def __init__(self, key, cost_to_go, orientation, back_ptr, pid):
        
        self.orientation = orientation
        self.key = key
        self.cost_to_go = cost_to_go
        self.back_ptr = back_ptr
        self.pid = pid

tab_width = 600
tab_height = 200


def get_inquality_obstacles(x, y, clearance):
    if (x >= (tab_width - clearance)) or (y >= (tab_height - clearance)) or (x <= clearance) or (y <= clearance) or\
       ((y >= 75 - clearance) and (x <= (165 + clearance)) and (x >= (150 - clearance)) and (y <= tab_height)) or\
       ((y <= (125 + clearance)) and (x >= (250 - clearance)) and (y >= 0) and (x <= (265 + clearance))) or\
        (((x-400)**2 + (y-110)**2) < (60 + clearance)**2):
            return True
    
    return False

def generate_graph():
    graph = np.ndarray((tab_width,tab_height),dtype=Cell)
    return graph

def inflate_obstacles(clearance, robot_radius):

    graph = generate_graph()
 
    for x in range(tab_width):
        for y in range(tab_height):
            graph[x, y] = Cell(1000000, 0, 0, np.array([-1, -1]), -1)
            if get_inquality_obstacles(x,y, clearance+robot_radius):
                graph[x, y] = Cell(-1, 0, 0, np.array([-1, -1]), -1)
                
    return graph


class Astar:
    def __init__(self, start, goal, space, clearance, robot_radius, RPM_left, RPM_right):
        self.start = start
        self.goal = goal
        self.space = space
        self.clearance = clearance
        self.robot_radius = robot_radius
        self.RPM_left = RPM_left
        self.RPM_right = RPM_right
        self.RPM_left = 5
        self.RPM_right = 10
        self.action_set = [[0, RPM_left],
                      [RPM_left, 0],
                      [RPM_left, RPM_left],
                      [0, RPM_right],
                      [RPM_right, 0],
                      [RPM_right,RPM_right],
                      [RPM_left, RPM_right], 
                      [RPM_right, RPM_left]]
        self.yellow = (0, 255, 255)
        self.blue = (0, 0, 255)
        self.green = (0, 255, 0)
        self.white = (255, 255, 255)
        self.pq = {}
        self.visited = np.zeros((tab_width, tab_height))
        self.space[self.start[0], self.start[1]].key = 0
        self.space[self.start[0], self.start[1]].cost_to_go = 0
        self.space[self.start[0], self.start[1]].orientation = self.start[2]
        self.space[self.start[0], self.start[1]].back_ptr = np.array([self.start[1], self.start[1]])
        
        src_x = self.start[0]
        src_y = self.start[1]
        
        
        self.pq[(src_x, src_y)] = self.space[src_x, src_y].key

    def check_bounds(self, x, y):
    
        if (not (0 < x < tab_width)) or (not (0 < y < tab_height)) or (self.space[x, y].key == -1):
            return False
    
        return True
        
    def search(self):

        cv_backgroud = np.zeros((tab_width, tab_height, 3), np.uint8)

        for x in range(tab_width):
            for y in range(tab_height):

                if get_inquality_obstacles(x,y, self.clearance):   
                    cv_backgroud[x][y] = self.yellow

                if get_inquality_obstacles(x,y,0):
                    cv_backgroud[x][y] = self.blue
        
        print("planning ...")

        while(len(self.pq)):

            curr_cell = min(self.pq, key = self.pq.get)
            self.pq.pop(curr_cell)
            
            x = curr_cell[0]
            y = curr_cell[1]
            ori = self.space[x, y].orientation

            cv2.circle(self.visited, (y, x), 15, 1, -1)

            for neighbor in self.neighbors(x, y, ori):

                neigh_x, neigh_y,_, _, f_new,_  = neighbor

                if(self.is_goal(neigh_x, neigh_y)):
                    self.space[neigh_x, neigh_y].back_ptr = np.array([x, y])
                    return self.backtrack(neigh_x, neigh_y, cv_backgroud)
                    
                if(self.check_bounds(neigh_x, neigh_y)):    
                    if self.visited[neigh_x, neigh_y] == 0:
                        if self.space[neigh_x, neigh_y].key == 1000000 \
                                or self.space[neigh_x, neigh_y].key > f_new:

                            self.update_neigh(neighbor, cv_backgroud, x, y, ori)

        print("no path")
        return []
                            
    def update_neigh(self, neighbor, cv_background, x, y, ori):

        neigh_x, neigh_y,neigh_ori, parent_index, neigh_key, neigh_cost_to_go  = neighbor

        self.pq[neigh_x, neigh_y] = neigh_key

        self.space[neigh_x, neigh_y].key = neigh_key
        self.space[neigh_x, neigh_y].cost_to_go = neigh_cost_to_go
        self.space[neigh_x, neigh_y].orientation  = neigh_ori
        self.space[neigh_x, neigh_y].pid = parent_index
        self.space[neigh_x, neigh_y].back_ptr = np.array([x, y])
        
        self.draw_traj(cv_background, x, y, ori, self.action_set[parent_index][0], self.action_set[parent_index][1])


    def backtrack(self, last_x, last_y, cv_background):
        
        x = last_x
        y = last_y

        path = []
        path.append((self.goal[0], self.goal[1]))
        
        while (x != self.start[0]) or (y != self.start[1]):
            x, y = self.space[x, y].back_ptr
            path.append((x, y))

        path.append((self.start[0], self.start[1]))
        path.reverse()
        self.visualize_path(path, cv_background)

        return path

    def visualize_path(self, path, vis):

        for point in range(len(path) - 1):
            cv2.line(vis, (path[point][1], path[point][0]), (path[point+1][1], path[point+1][0]), self.green, 2)

        cv2_background = cv2.rotate(vis, cv2.ROTATE_90_COUNTERCLOCKWISE)
        cv2.imshow('A*', cv2_background)
        cv2.waitKey(500)
        cv2.destroyAllWindows()
    

    def neighbors(self, x, y, theta):

        neighbors = []
        parent_g = self.space[x, y].cost_to_go

        for i in range(len(self.action_set)):
            neigh_x, neigh_y, neigh_ori = self.simulate_dynamics(x, y, theta, self.action_set[i][0], self.action_set[i][1])

            if(neigh_x == None or neigh_y == None or neigh_ori == None):
                continue

            h = self.heuristic(neigh_x, neigh_y)
            g = self.calc_cost_to_go(neigh_x, neigh_y, parent_g, x, y)

            priority = h + g

            neighbors.append([neigh_x, neigh_y, neigh_ori, i, priority, g])
            
        return neighbors

    def calc_cost_to_go(self, x, y, parent_g, p_x, p_y):
        g = parent_g + (math.sqrt(((x - p_x)**2) + ((y - p_y)**2)))
        return g

    def heuristic(self, x, y):
        h = (math.sqrt(((x - self.goal[0])**2) + ((y - self.goal[1])**2)))
        return h

    def is_goal(self, x, y):
        threshold = 10
        return (x - self.goal[0])**2 + (y - self.goal[1])**2 <= threshold*threshold

    def simulate_dynamics(self, x0, y0, theta0, rpm_l, rpm_r):
        
        t = 0
        r = 3.8
        L = 35.4
        # r = 0.038
        # L = 0.354
        dt = 0.1
        xn = x0
        yn = y0
        thetan = 3.14 * theta0 / 180
        while t < 1:
            t = t + dt
            xn += 0.5 * r * (rpm_l + rpm_r) * math.cos(thetan) * dt
            yn += 0.5 * r * (rpm_l + rpm_r) * math.sin(thetan) * dt
            thetan += (r / L) * (rpm_r - rpm_l) * dt

            if not self.check_bounds(round(xn), round(yn)):
                return None, None, None
             
        thetan = 180 * (thetan) / 3.14

        xn = round(xn)
        yn = round(yn)
        thetan = round(thetan)

        return xn, yn, thetan


    def draw_traj(self, cv_background, x0, y0, theta0, rpm_l, rpm_r):
        
        t = 0
        r = 3.8
        L = 35.4
        # r = 0.038
        # L = 0.354
        dt = 0.1
        xn = x0
        yn = y0
        thetan = 3.14 * theta0 / 180

        while t < 1:
            t = t + dt
            xtemp = xn
            ytemp = yn
            xn += 0.5 * r * (rpm_l + rpm_r) * math.cos(thetan) * dt
            yn += 0.5 * r * (rpm_l + rpm_r) * math.sin(thetan) * dt
            thetan += (r / L) * (rpm_r - rpm_l) * dt

            if not self.check_bounds(round(xn), round(yn)):
                continue

            cv2.line(cv_background, (round(yn), round(xn)) , (round(ytemp), round(xtemp)), self.white, 1)



def astar_planner():

    #start_x = int(input("Enter start x coordinate"))
    #start_y = int(input("Enter start y coordinate"))
    #start_ori = int(input("Enter start orientation"))

    start_x   = 50
    start_y   = tab_height - 100
    start_ori = 0


    #goal_x = int(input("Enter goal x coordinate"))
    #goal_y = int(input("Enter goal y coordinate"))

    goal_x = 550
    goal_y = tab_height - 100
    goal_ori = 0


    goal_y = 20

    clearance = int(input("enter clearance: "))
    robot_radius = 5

    RPM_left = int(input("enter RPM1: "))
    RPM_right = int(input("enter RPM2: "))
    RPM_left = 5
    RPM_right = 10

    start = np.array([start_x, start_y, start_ori])
    goal = np.array([goal_x, goal_y, goal_ori])


    space = inflate_obstacles(clearance, robot_radius)

    obj = Astar(start, goal, space, clearance, robot_radius, RPM_left, RPM_right)

    if not (obj.check_bounds(start_x, start_y) and obj.check_bounds(goal_x, goal_y)):
        return 

    
    path = obj.search()

    return path


#astar_planner()