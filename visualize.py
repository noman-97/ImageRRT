import numpy as np
import matplotlib.pyplot as plt
import sys
import random
import math
import copy

from read_map import get_bnw_image, get_start_end_nodes, confirm_final_nodes





class pathing_problem():

    class Node():

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
        

        self.start = self.Node(start[0],start[1])
        self.end = self.Node(end[0],end[1])
        self.image_map = image_map
        self.image_map_array=np.copy(image_map)
        self.max_iteration = max_iteration
        self.node_list = [self.start]

    


    def plot_graph(self):
        plt.clf()
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        

        for node in self.node_list:
            if node.parent != None:
                plt.plot([node.parent.x_val,node.x_val],[node.parent.y_val, node.y_val], '-g')
        

        plt.plot(self.start.x_val,self.start.y_val,"xr")
        plt.plot(self.end.x_val,self.end.y_val,"xr")
        plt.imshow(self.image_map)

        plt.grid(True)
        plt.pause(0.01)

    


    def link_nodes(self, from_node, to_node):

        copied_node=copy.deepcopy(from_node)
        copied_node.x_val=to_node.x_val
        copied_node.y_val=to_node.y_val

        copied_node.parent=from_node
        copied_node.cost+=math.dist([from_node.x_val,from_node.y_val],[to_node.x_val,to_node.y_val])

        return copied_node
    


    def collision_check(self, from_node, to_node):

        # True if no collisions, False if collision

        if from_node is None or to_node is None:
            return False
        

        x1 = from_node.x_val
        y1 = from_node.y_val


        x2 = to_node.x_val
        y2 = to_node.y_val

        


        dx = abs(x2-x1)
        dy = abs(y2-y1)

        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1

        err = dx-dy 

        while x1 != x2 or y1 != y2:


            

            if x1<0 or y1<0 or x1>= self.image_map.size[0] or y1>=self.image_map.size[0]:
                
                return True
            
            if self.image_map_array[y1,x1]==0:
                return False
            
        
            e2=2*err

            if e2>-dy:
                err -=dy
                x1 +=sx

            if e2<dx:
                err +=dx
                y1+=sy
        
        
        return True
        











def rrt_planner(pathing_solution, display_progress=False):

    iter=0

    start=pathing_solution.start
    end=pathing_solution.end
    grayimage=pathing_solution.image_map

    #pathing_solution = pathing_problem(start,end,grayimage)

    x_lim=pathing_solution.image_map.size[0]
    y_lim=pathing_solution.image_map.size[1]

    max_node_cost = (x_lim*y_lim)


    exploration_bias=0.4
    goal_bias_radius=5

    goal_bias=0.2*exploration_bias
    goal_bias=0*exploration_bias

    distance_to_goal_radius=2


    goal_x_val=pathing_solution.end.x_val
    goal_y_val=pathing_solution.end.y_val

    i=0
    while i<pathing_solution.max_iteration:
        
        random_bias_check=random.random()
        i+=1

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


        for j in pathing_solution.node_list:

            if math.dist([j.x_val,j.y_val],[random_node.x_val,random_node.y_val]) < nearest_cost:

                if pathing_solution.collision_check(j,random_node):
                    nearest_node=j
                    nearest_cost=math.dist([j.x_val,j.y_val],[random_node.x_val,random_node.y_val])

        

        if nearest_cost != max_node_cost:

            random_node = pathing_solution.link_nodes(nearest_node,random_node)

            pathing_solution.node_list.append(random_node)

            if math.dist([random_node.x_val,random_node.y_val],[end.x_val,end.y_val]) < distance_to_goal_radius:

                break
        

        if display_progress:
            pathing_solution.plot_graph()
    

    if i==pathing_solution.max_iteration:
        print('Reached Max Iterations.')
        return None





    final_list=[]

    final_node=pathing_solution.node_list[-1]

    while final_node.parent is not None:
        final_list.append(final_node)

        final_node=final_node.parent
    

    final_list.append(start)

    final_list.reverse()

    
    print("Iterations: ",i)
    return final_list
    









def main():

    filepath="maps/path4.png"


    
    grayimage = get_bnw_image(filepath)

    start,end=get_start_end_nodes(grayimage)


    

    
    #new_problem=pathing_problem(start,end,grayimage,max_iteration=10000)

    new_problem_2=pathing_problem(start,end,grayimage)

    

    

    
    problem_test=rrt_planner(new_problem_2,display_progress=False)

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

        




        





            


