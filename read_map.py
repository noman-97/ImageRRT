"""
This module includes the following functions for transforming an RGB image into a grayscale maze for path planning:
 - 'get_bnw_image'
 - 'get_start_end_nodes'
 - 'confirm_final_nodes'
"""

import numpy as np
import matplotlib.pyplot as plt
from PIL import Image


def get_bnw_image(filepath):
    """
        takes an image path as input and returns a black and white image as output.
    
        Parameters
        ------------------
        - filepath: A string representing the path to the image file ('proj/xyz.png')
        
        Returns
        ------------------
        - image: A black and white image object 
    """
    # Open image and convert to grayscale
    orig_image = Image.open(filepath).convert('RGB')
    image=orig_image.convert("L")

    # threshold value (0 - Black/Obstacle, 255 - White/Path)
    threshold=200
    th=lambda x : 255 if x > threshold else 0

    # Convert the image to black and white using the threshold value
    image = image.convert('L').point(th, mode='1')
    image=image.convert("L")

    return image


def get_start_end_nodes(image):
    """
        takes a black and white image as input (converts RGB images), allows the user to choose a start and end point via two clicks, and returns the corresponding points.
        
        Parameters
        ------------------
        - image: A black and white image object
        
        Returns
        ------------------
        - start_node: A numpy array representing the coordinates of the start point
        - end_node: A numpy array representing the coordinates of the end point
    """

    fig, ax = plt.subplots()
    plt.imshow(image, cmap='gray')

    # node list that contains start and end node
    nodes_list=[]
    


    # Inner function, handles button press event for selecting points

    def select_nodes(event):

        # x and y coordinates on click
        x_val, y_val = event.xdata, event.ydata

        # Print coordinates for confirmation
        print("Coordinate chosen: ", (np.round(x_val.astype(int)), np.round(y_val.astype(int))))

        nodes_list.append((x_val, y_val))

        # Break after two nodes selected
        if len(nodes_list) == 2:
            fig.canvas.mpl_disconnect(map_event)  
            plt.close()


    # Call inner function, set event as button press
    map_event=fig.canvas.mpl_connect("button_press_event", select_nodes)
    plt.show()

    # Get temporary start node coordinates from nodes_list
    float_start_node=nodes_list[0]

    # Final start node as numpy array
    start_node=np.array([np.round(float_start_node[0].astype(int)),np.round(float_start_node[1].astype(int))])

    # Get temporary end node coordinates from nodes_list  
    float_end_node=nodes_list[1]

    # Final end node as numpy array
    end_node=np.array([np.round(float_end_node[0].astype(int)),np.round(float_end_node[1].astype(int))])


    return start_node,end_node


def confirm_final_nodes(image, start_node, end_node):
    """
        takes a black and white image, as well two arrays with start/end node coordinates as input, and displays the coordinates as blue (start) and red (end) boxes on the image.
        
        Parameters
        ------------------
        - image: A black and white image object
        - start_node: Array of start node coordinates (x, y)
        - end_node: Array of end node coordinates (x, y)
    """

    # Copy black and white image, convert to RGB
    check_image=np.copy(image.convert('RGB'))

    # Width variable, each side of final square is equal to 2*width
    width = 10

    # Change start node coordinate colour to blue, and end coordinates to red
    check_image[start_node[1]-width:start_node[1]+width,start_node[0]-width:start_node[0]+width]=[0,0,255]
    check_image[end_node[1]-width:end_node[1]+width,end_node[0]-width:end_node[0]+width]=[255,0,0]

    fig, ax = plt.subplots()
    ax.imshow(check_image)
    plt.show()


def main():

    # Boolean to show chosen nodes in final image
    confirmation=True

    # Filepath to image/map
    filepath="map\path2.png"

    # Get black and white image from path
    grayimage=get_bnw_image(filepath)

    # Choose and acquire start/end node coordinates
    start,end=get_start_end_nodes(grayimage)

    # Warning in case chosen coordinates are placed on an obstacle
    if np.asarray(grayimage)[start[1],start[0]]==0 or np.asarray(grayimage)[end[1],end[0]]==0:
        print("Warning: chosen coordinates are inside an obstacle.")
        
    # Show final nodes
    if confirmation:
        confirm_final_nodes(grayimage,start,end)



if __name__ == '__main__':
    main()


















