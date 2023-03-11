import numpy as np
import time
import cv2 as cv
import heapq

nodes={}                        # To store all nodes
start_node=(5,5)                # To store the start node(Default given to avoid errors)
end_node=None                   # To store the end node

# Visualizing the path by writing video
frameSize = (600, 250)
fourcc = cv.VideoWriter_fourcc('m','p','4','v')
out  = cv.VideoWriter('output.mp4', fourcc, 250, frameSize)


# Function to create the map and obstacle map
def create_map():
    # Main map for display
    map=np.ones((250,600,3),dtype=np.uint8)
    map[:,:,0]=51
    map[:,:,1]=51
    map[:,:,2]=44
    
    # Obstacle map for path planning
    obs_map=np.ones((250,600),dtype=np.uint8)*255
    
    # Draw obstacles on the maps
    cv.rectangle(map,(95,0),(155,105),(222,228,203),-1)
    cv.rectangle(map,(95,145),(155,255),(222,228,203),-1)
    
    # On the obstacle map
    cv.rectangle(obs_map,(95,0),(155,105),(0,0,0),-1)
    cv.rectangle(obs_map,(95,145),(155,255),(0,0,0),-1)
    # Draw rectangles on the map
    cv.rectangle(map,(100,0),(150,100),(136, 131, 14),-1)
    cv.rectangle(map,(100,150),(150,250),(136, 131, 14),-1)
    
    # Draw a hexagon on the image
    center = [300, 125]
    vertices = []
    obs_vertices=[]
    for i in range(6):
        angle = np.deg2rad(60 * i)
        x,y=center + 75 * np.array([np.cos(angle),np.sin(angle)])
        vertices.append((int(x), int(y)))
        x,y=center + 80 * np.array([np.cos(angle),np.sin(angle)])
        obs_vertices.append((int(x), int(y)))
    cv.fillConvexPoly(map, np.array(obs_vertices), (222,228,203))
    cv.fillConvexPoly(obs_map, np.array(obs_vertices), (0,0,0))
    cv.fillConvexPoly(map, np.array(vertices), (136, 131, 14))

    # Draw a traingle on the image
    vertices = [(460, 25), (460, 225), (510, 125)]
    obs_vertices = [(455, 10), (455, 240), (517, 125)]
    cv.fillConvexPoly(map, np.array(obs_vertices), (222,228,203))
    cv.fillConvexPoly(obs_map, np.array(obs_vertices), (0,0,0))
    cv.fillConvexPoly(map, np.array(vertices), (136, 131, 14))

    # Clearance for the walls
    cv.rectangle(obs_map,(0,0),(600,250),(0,0,0),6)
    cv.rectangle(map,(0,0),(600,250),(222,228,203),5)
    
    # cv.imshow('Map',obs_map)
    # cv.imshow('Mafp',map)
    # cv.waitKey(0)
    
    out.write(map)
    return map,obs_map

# Function to insert a node into the nodes dictionary
def insert_node(cost=None,node=None,parent=None):    
    if len(nodes)==0:
        nodes.update({start_node:[None,0]})
    else:
        nodes.update({node:[parent,cost]})
        
# Action functions to move the point robot in 8 directions
def Actions(node):
    
    # Function to move the point robot to the left
    def ActionMoveLeft(node):
        i,j=node
        if i==0:
            return None,None
        else:
            i=i-1
            j=j
            cost=1
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot to the right
    def ActionMoveRight(node):
        i,j=node
        if i==600:
            return None,None
        else:
            i=i+1
            j=j
            cost=1
            # print(nodes.keys())
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot up
    def ActionMoveUp(node):
        i,j=node
        if j==0:
            return None,None
        else:
            i=i
            j=j-1
            cost=1
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot down
    def ActionMoveDown(node):
        i,j=node
        if j==250:
            return None,None
        else:
            i=i
            j=j+1
            cost=1
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot diagonally up left
    def ActionMoveUpLeft(node):
        i,j=node
        if i==0 or j==0:
            return None,None
        else:
            i=i-1
            j=j-1
            cost=1.4
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot diagonally up right
    def ActionMoveUpRight(node):
        i,j=node
        if i==600 or j==0:
            return None,None
        else:
            i=i+1
            j=j-1
            cost=1.4
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot diagonally down left
    def ActionMoveDownLeft(node):
        i,j=node
        if i==0 or j==250:
            return None,None
        else:
            i=i-1
            j=j+1
            cost=1.4
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost

    # Function to move the point robot diagonally down right
    def ActionMoveDownRight(node):
        i,j=node
        if i==600 or j==250:
            return None,None
        else:
            i=i+1
            j=j+1
            cost=1.4
            if (i,j) in nodes.keys():
                if nodes[(i,j)][1]==float('inf'):
                    return None,None
                else:
                    return (i,j),cost
            else:
                return (i,j),cost
        
    return [(ActionMoveLeft(node)), (ActionMoveUp(node)), (ActionMoveRight(node)), (ActionMoveDown(node)),
            (ActionMoveUpLeft(node)),(ActionMoveUpRight(node)),(ActionMoveDownLeft(node)),(ActionMoveDownRight(node))]

# Tree function to generate the heap tree graph of the nodes using Djkstra's Algorithm
def tree(end_node):
    global nodes
    open_list=[]
    closed_list=[]
    explored_nodes=[]
    heapq.heappush(open_list,(0,start_node))
    while open_list:
        current_distance,current_node=heapq.heappop(open_list)
        if current_node not in explored_nodes:
            explored_nodes.append(current_node)
        closed_list.append(current_node)
        
        if current_node==end_node:
            return current_node
        
        for action in Actions(current_node):
            new_node,cost=action
            if new_node is not None and new_node not in closed_list:
                if new_node not in explored_nodes:
                    new_cost=current_distance+cost
                    heapq.heappush(open_list,(new_cost,new_node))
                    explored_nodes.append(new_node)
                    insert_node(new_cost,new_node,current_node)
                else:
                    for i in range(len(open_list)):
                        if open_list[i][1]==new_node:
                            new_cost=current_distance+cost
                            if open_list[i][0]>new_cost:
                                open_list[i]=(new_cost,new_node)
                                insert_node(new_cost,new_node,current_node)
    return None

# Returns the parent node for a given node    
def get_parent(node):
    return nodes[node][0]

# Returns a path from the end_node to the start_node
def generate_path():
    # Searching starts here
    path=[tree(end_node)]
    total_cost=nodes[path[0]][1]
    parent=get_parent(path[0])
    while parent is not None:
        path.append(parent)
        parent = get_parent(parent)
    path.reverse()
    return path,total_cost

# Saving the map
def save_map(map):
    print("\nSaving the map:")
    for i in nodes.keys():
        if nodes[i][1]!=float('inf'):
            cv.circle(map,(i[0],250-i[1]),1,(79,79,46),-1)
            out.write(map)
    for i in range(len(path)):
        cv.circle(map,(path[i][0],250-path[i][1]),1,(234,143,234),-1)
        out.write(map)
    for i in range(500):
        out.write(map)
    print("Map saved as output.mp4")
    out.release()

# Getting user inputs
def get_inputs():
    global start_node,end_node
    check_input=True
    while check_input:
        s_node=input("Note:'(5,5) is the starting point due to clearance on the walls'\nEnter the start node in the format 0 1 for (0,1): ")
        x,y=s_node.split()
        if int(x)>600 or int(y)>250 or int(x)<-1 or int(y)<-1:
            print("Please enter valid coordinates.")
        if (int(x),int(y)) in nodes.keys():
            if (int(x),int(y))!=(5,5):
                print("Please enter a valid start node(Node in obstacle place).")
            else:
                check_input=False
                start_node=(int(x),int(y))                
        else:
            check_input=False
            start_node=(int(x),int(y))
    check_input=True
    while check_input:
        f_node=input("Note:'(595,245) is the ending point due to clearance on the walls'\nEnter the end node in the format 0 1 for (0,1) : ")
        x,y=f_node.split()
        if int(x)>600 or int(y)>250 or int(x)<-1 or int(y)<-1 or (int(x),int(y))==start_node:
            print("Please enter valid coordinates.")
        if (int(x),int(y)) in nodes.keys():
            print("Please enter a valid end node(Node in obstacle place).")
        else:
            check_input=False
            end_node=(int(x),int(y))

# Main function
if __name__ == "__main__":
    start_time = time.time()
    
    # Getting maps
    map,obs_map=create_map()
    
    # Inserting obstacle nodes into the nodes dictionary
    insert_node()
    for i in range(600):
        for j in range(250):
            if obs_map[j][i]==0:
                insert_node(float('inf'),(i,j),None)

    # Getting user inputs
    get_inputs()
    
    # Generating the path and the total cost
    path,total_cost=generate_path()   
    
    # Saving the animation of the path generation
    save_map(map)
        
    end_time = time.time()
    print(f"Time taken to execute the code is: {(end_time-start_time)/60} minutes.")
    print("\nTotal cost of the path is: ",total_cost)
    print("\nPath is: ",path)
