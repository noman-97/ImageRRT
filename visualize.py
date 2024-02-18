"""
This module contains classes and functions related to pathfinding using a sampling-based algorithm:
 - 'pathing_problem'
 - 'rrt_planner'
"""

import numpy as np
import matplotlib.pyplot as plt
import sys
import random
import math
import copy

try:
    from read_map import get_bnw_image, get_start_end_nodes, confirm_final_nodes

except ImportError:
    raise



class pathing_problem():
 
    class Node():
        """
        Node for coordinates on image_map
        
        Attributes:
        ----------------
        - x         - x coordinate of the node
        - y         - y coordinate of the current node
        - parent    - a reference to the parent node object
        - cost      - total length travelled from start node to current node (use for RRT*, not RRT)
        """

        def __init__(self, x_val, y_val):
            self.x_val=x_val
            self.y_val=y_val
            
            self.parent=None
            self.cost=0
        

        def print_node(self):
            print('(x, y) = (',self.x_val,', ',self.y_val,')')

          
        def __str__(self):
            return f"({self.x_val}, {self.y_val})"
        

        def __repr__(self):
            return self.__str__()
        



    def __init__(self, start, end, image_map, max_iteration=4000):
        """
        Parameters
        ------------------
        - start         - Start coordinates (x,y)
        - end           - End coordinates (x,y)
        - image_map     - Grayscale image to search in
        - max_iteration - Maximum number of samples before searching stops (avoid expensive searches or infinite loops).
        """

        self.start = self.Node(start[0],start[1])
        self.end = self.Node(end[0],end[1])
        self.image_map = image_map
        self.image_map_array=np.copy(image_map) # image as np array
        self.max_iteration = max_iteration
        self.node_list = [self.start] # List of nodes explored

    


    def plot_graph(self):
        """
        plots an image showing:
        - entire grayscale image
        - all explored nodes
        - all paths between parent and child nodes
        - start and goal point

        No input or output
        """
        plt.clf()

        # allows stopping search with escape key
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        

        for node in self.node_list:
            if node.parent != None:
                plt.plot([node.parent.x_val,node.x_val],[node.parent.y_val, node.y_val], '-g')
        

        plt.plot(self.start.x_val,self.start.y_val,"xr")
        plt.plot(self.end.x_val,self.end.y_val,"xr")
        plt.imshow(self.image_map)

        plt.grid(True)
        plt.pause(0.01) # short delay between images

    


    def link_nodes(self, from_node, to_node):
        """
        Helper function that creates a new copied_node which:
        - shares the state/coordinates of to_node
        - has the cost (total distance travelled) of from_node
        - has from_node as the assigned parent

        Parameters
        ------------------
        - from_node (Node)     - parent node of path traversed
        - to_node (Node)       - child/goal node of path traversed

        Returns
        ------------------
        - copied_node (Node)   - child node sharing state with to_node, with from_node as parent
        """
        copied_node=copy.deepcopy(from_node)
        copied_node.x_val=to_node.x_val
        copied_node.y_val=to_node.y_val

        copied_node.parent=from_node
        copied_node.cost+=math.dist([from_node.x_val,from_node.y_val],[to_node.x_val,to_node.y_val])

        return copied_node
    


    def collision_check(self, from_node, to_node):
        """
        collision_check is a modification of the Bresenham line algorithm
        to check if the path between from_node and to_node has any obstacles

        Parameters
        ------------------
        - from_node (Node)     - node from which path starts
        - to_node (Node)       - node at which path ends

        Returns
        ------------------
        - (boolean)            - true if no collisions present, false otherwise
        """

        # non-valid inputs
        if from_node is None or to_node is None:
            return False
        

        x1 = from_node.x_val
        y1 = from_node.y_val


        x2 = to_node.x_val
        y2 = to_node.y_val

        


        dx = abs(x2-x1)
        dy = abs(y2-y1)

        sx = 1 if x1 < x2 else -1 # for traversal direction (x-axis)
        sy = 1 if y1 < y2 else -1 # for traversal direction (y-axis)

        err = dx-dy 

        while x1 != x2 or y1 != y2:
     
            # edge case for ends of image_map
            if x1<0 or y1<0 or x1>= self.image_map.size[0] or y1>=self.image_map.size[0]:
                
                return True
            
            # if grid contains obstacle pixel (0) return collision
            if self.image_map_array[y1,x1]==0:
                return False
            

            # Bresenham grid traversal
            e2=2*err

            if e2>-dy:
                err -=dy
                x1 +=sx

            if e2<dx:
                err +=dx
                y1+=sy
        
        
        return True
        











def rrt_planner(pathing_solution, display_progress=False):
    """
        2D implementation of the Rapidly exploring Random Tree (RRT) for pathing problems.

        Parameters
        ------------------
        - pathing_solution (pathing_problem)     - class containing problem specifications (start, end, image_map, max_iterations)
        - display_progress (bool)                - flag for showing progress as animation (set False for faster solutions)

        Returns
        ------------------
        - final_list                             - list of nodes from start to end containing the final path (returns None if no solution present)
    """

    start=pathing_solution.start # start coordinates
    end=pathing_solution.end # end coordinates
    grayimage=pathing_solution.image_map

    x_lim=pathing_solution.image_map.size[0] # max x-coordinate to search
    y_lim=pathing_solution.image_map.size[1] # max y-coordinate to search
    max_node_cost = (x_lim*y_lim) # max node cost (area of entire map)

    # planner search parameters (feel free to adjust)
    goal_bias_radius=50 # radius around the goal for bias in exploration
    exploration_bias=0.4 # bias towards exploring near goal (0 - 1)
    goal_bias=0.2*exploration_bias # bias towards exploring directly at goal coordinate (0 - 1)
    distance_to_goal_radius=2 # radius for goal proximity in solution checking

    goal_x_val=pathing_solution.end.x_val
    goal_y_val=pathing_solution.end.y_val
    i=0

    while i<pathing_solution.max_iteration:
        i+=1
        # Generate a random node/state (x, y)
        # Random check to see if node should be the goal (goal bias), near the goal (exploration bias), or a purely random node
        random_bias_check=random.random()

        if random_bias_check<goal_bias:
            xRand=goal_x_val
            yRand=goal_y_val
        
        elif random_bias_check<exploration_bias:
            xRand=random.randint(max(goal_x_val-goal_bias_radius,0),min(goal_x_val+goal_bias_radius,x_lim))
            yRand=random.randint(max(goal_y_val-goal_bias_radius,0),min(goal_y_val+goal_bias_radius,y_lim))

        else:
            xRand=random.randint(0,x_lim)
            yRand=random.randint(0,y_lim)

        random_node=pathing_solution.Node(xRand,yRand)
        nearest_node=start
        nearest_cost=max_node_cost

        # Iterate through current explored nodes
        for j in pathing_solution.node_list:
   
            # Check if current node is the closest node
            if math.dist([j.x_val,j.y_val],[random_node.x_val,random_node.y_val]) < nearest_cost:

                # if it's the nearest node, check for collision
                if pathing_solution.collision_check(j,random_node):
                    nearest_node=j
                    nearest_cost=math.dist([j.x_val,j.y_val],[random_node.x_val,random_node.y_val])

        
        # if cost is not equal to max node cost, add valid node to list
        if nearest_cost != max_node_cost:
            random_node = pathing_solution.link_nodes(nearest_node,random_node)
            pathing_solution.node_list.append(random_node)

            # break loop if added node is within (distance_to_goal_radius) units of goal
            if math.dist([random_node.x_val,random_node.y_val],[end.x_val,end.y_val]) < distance_to_goal_radius:
                break
        
        # show progress
        if display_progress:
            pathing_solution.plot_graph()
    

    if i==pathing_solution.max_iteration:
        print('Reached Max Iterations.')
        return None

    # list of nodes showing final path
    final_list=[]
    final_node=pathing_solution.node_list[-1]

    # Run until final node reaches the start, after which its parent will be None
    while final_node.parent is not None:
        final_list.append(final_node)
        final_node=final_node.parent
    

    final_list.append(start)
    final_list.reverse()

    print("Iterations: ",i)
    return final_list
    









def main():

    # variables
    # filepath to image
    filepath="planningalg/map/path3.png"

    # max iterations for searching
    max_search_iters=4000

    # show progress graph
    disp_progress=True


    
    grayimage = get_bnw_image(filepath)
    start,end=get_start_end_nodes(grayimage)
    #new_problem=pathing_problem(start,end,grayimage)
    new_problem_2=pathing_problem(start,end,grayimage,max_iteration=max_search_iters)
    problem_test=rrt_planner(new_problem_2,display_progress=disp_progress)

    print(problem_test)
    new_problem_2.plot_graph()

    path=[]

    if problem_test is None:
        print("No solution found.")

    else:

        for node in problem_test:
            path.append([node.x_val,node.y_val])
        plt.plot([x for (x, y) in path], [y for (x, y) in path], '-r')
        plt.show()
    


if __name__ == '__main__':
    main()

        




        





            


