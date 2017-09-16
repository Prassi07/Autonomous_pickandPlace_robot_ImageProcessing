import cv2
import numpy as np
from heapq import heappush,heappop
import time
import math
import copy
import serial

class node:
    # current position
    xPos = 0
    yPos = 0
    # total distance already travelled to reach the node
    distance = 0
    # priority = distance + remaining distance estimate
    priority = 0 # smaller: higher priority
    def __init__(self, xPos, yPos, distance, priority):
        self.xPos = xPos
        self.yPos = yPos
        self.distance = distance
        self.priority = priority
    def __lt__(self, other): # for priority queue
        return self.priority < other.priority
    def updatePriority(self, xDest, yDest):
        self.priority = self.distance + self.estimate(xDest, yDest) * 10 # A*
    # give better priority to going straight instead of diagonally
    def nextdistance(self, i): # i: direction
        if i % 2 == 0:
            self.distance += 10
        else:
            self.distance += 14
    
    # Estimation function for the remaining distance to the goal.
    def estimate(self, xDest, yDest):
        xd = xDest - self.xPos
        yd = yDest - self.yPos
        # Euclidian Distance
        d = math.sqrt(xd * xd + yd * yd)
        # Manhattan distance
        # d = abs(xd) + abs(yd)
        # Chebyshev distance
        # d = max(abs(xd), abs(yd))
        return(d)

# A-star algorithm.
# Path returned will be a string of digits of directions.
def pathFind(the_map, directions, dx, dy, xStart, yStart, xFinish, yFinish):
    closed_nodes_map = []    # map of closed (tried-out) nodes
    open_nodes_map = []      # map of open (not-yet-tried) nodes
    dir_map = []             # map of directions
    row = [0] * n
    for i in range(m):       # create 2d arrays
        closed_nodes_map.append(list(row))
        open_nodes_map.append(list(row))
        dir_map.append(list(row))

    pq = [[], []]            # priority queues of open (not-yet-tried) nodes
    pqi = 0                  # priority queue index
                             # create the start node and push into list of open nodes
    n0 = node(xStart, yStart, 0, 0)
    n0.updatePriority(xFinish, yFinish)
    heappush(pq[pqi], n0)
    open_nodes_map[yStart][xStart] = n0.priority # mark it on the open nodes map

                             # A* search
    while len(pq[pqi]) > 0:
        # get the current node w/ the highest priority
        # from the list of open nodes
        n1 = pq[pqi][0] # top node
        n0 = node(n1.xPos, n1.yPos, n1.distance, n1.priority)
        x = n0.xPos
        y = n0.yPos
        heappop(pq[pqi]) # remove the node from the open list
        open_nodes_map[y][x] = 0
        # mark it on the closed nodes map
        closed_nodes_map[y][x] = 1

        # quit searching when the goal state is reached
        # if n0.estimate(xFinish, yFinish) == 0:
        if x == xFinish and y == yFinish:
            # generate the path from finish to start
            # by following the directions
            path = ''
            while not (x == xStart and y == yStart):
                j = dir_map[y][x]
                c = str((j + directions / 2) % directions)
                path = c + path
                x += dx[j]
                y += dy[j]
            return path

        # generate moves (child nodes) in all possible directions
        for i in range(directions):
            xdx = x + dx[i]
            ydy = y + dy[i]
            if not (xdx < 0 or xdx > n-1 or ydy < 0 or ydy > m - 1
                    or the_map[ydy][xdx] == 1 or closed_nodes_map[ydy][xdx] == 1):
                # generate a child node
                m0 = node(xdx, ydy, n0.distance, n0.priority)
                m0.nextdistance(i)
                m0.updatePriority(xFinish, yFinish)
                # if it is not in the open list then add into that
                if open_nodes_map[ydy][xdx] == 0:
                    open_nodes_map[ydy][xdx] = m0.priority
                    heappush(pq[pqi], m0)
                    # mark its parent node direction
                    dir_map[ydy][xdx] = (i + directions / 2) % directions
                elif open_nodes_map[ydy][xdx] > m0.priority:
                    # update the priority info
                    open_nodes_map[ydy][xdx] = m0.priority
                    # update the parent direction info
                    dir_map[ydy][xdx] = (i + directions / 2) % directions
                    # replace the node
                    # by emptying one pq to the other one
                    # except the node to be replaced will be ignored
                    # and the new node will be pushed in instead
                    while not (pq[pqi][0].xPos == xdx and pq[pqi][0].yPos == ydy):
                        heappush(pq[1 - pqi], pq[pqi][0])
                        heappop(pq[pqi])
                    heappop(pq[pqi]) # remove the wanted node
                    # empty the larger size pq to the smaller one
                    if len(pq[pqi]) > len(pq[1 - pqi]):
                        pqi = 1 - pqi
                    while len(pq[pqi]) > 0:
                        heappush(pq[1-pqi], pq[pqi][0])
                        heappop(pq[pqi])       
                    pqi = 1 - pqi
                    heappush(pq[pqi], m0) # add the better node instead
    return '' # no route found
n=9
m=6
the_map=[]
row = [0] * n
for i in range(m):
    the_map.append(list(row))

def GetThreshold(img):
    '''Get the Threshold Image'''
    imghsv=cv2.cvtColor(img,cv2.COLOR_BGR2HSV)
    lower_blue = np.array([105,100,100])
    upper_blue = np.array([135,255,255])
    bluethresh = cv2.inRange(imghsv, lower_blue, upper_blue)
    lower_green =np.array([40,200,200])
    upper_green = np.array([70,255,255])
    greenthresh = cv2.inRange(imghsv, lower_green, upper_green)
    upper_red1 =np.array([10,255,255])
    lower_red1 =np.array([0,100,100])
    redthresh1 = cv2.inRange(imghsv, lower_red1, upper_red1)
    upper_red2 =np.array([179,255,255])
    lower_red2 =np.array([160,100,100])
    redthresh2 = cv2.inRange(imghsv, lower_red2, upper_red2)
    redthresh=redthresh1+redthresh2
    threshall=bluethresh+greenthresh+redthresh
    #kernel=cv2.getStructuringElement(cv2.MORPH_CROSS,(3,3))
    #thresh=cv2.morphologyEx(threshall,cv2.MORPH_CLOSE,kernel)
    #thresh=cv2.erode(thresh1,kernel,iterations=1)
    x=threshall.copy()
    '''cv2.imshow('x',bluethresh);
    cv2.waitKey(0);
    cv2.imshow('x',greenthresh);
    cv2.waitKey(0);
    cv2.imshow('x',redthresh);
    cv2.waitKey(0);'''
    cv2.imshow('x',threshall);
    cv2.waitKey(0);
    return imghsv,x
allobjects=[]
occupied_grids=[]

def GetObjectInfo(filterImage,imghsv):
    contours_board,h=cv2.findContours(filterImage,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE);
    for i in contours_board:
        peri=cv2.arcLength(i,True)
        approx=cv2.approxPolyDP(i,0.04*peri,True)
        M=cv2.moments(i)
        area=cv2.contourArea(i);
        color="white";
        if area>50:
            
            y=int(M['m10']/M['m00'])
            x=int(M['m01']/M['m00'])

            yc=int(x/80)+1;
            xc=int(y/71)+1;

            if len(approx)==3:
                shape="Triangle"
            elif len(approx) == 4 :
                (ss2, ss, w, h) = cv2.boundingRect(approx)
                ar = w / float(h)
            # a square will have an aspect ratio that is approximately
            # equal to one, otherwise, the shape is a rectangle
                shape = "square" 
            else:
                shape="Circle"

            if imghsv[x][y][0]>=105 and imghsv[x][y][0]<=140 :
                color="blue"
            elif imghsv[x][y][0]>=50 and imghsv[x][y][0]<70 :
                color="green"
            elif imghsv[x][y][0]>=0 and imghsv[x][y][0]<180 :
                   if area>1800:
                       color="obstacle"
                   else:
                       color="red"
            else:
                color="white"
                
            if color!="white" and cv2.contourArea(i)<3500:
                occupied_grids.append((xc,yc));
                the_map[yc-1][xc-1]=1
                occupied_grids.sort();
                if color!="obstacle":
                    allobjects.append(((xc,yc),color,shape,cv2.contourArea(i)))
            allobjects.sort()
def GetNewPath(path):
    newpath=''
    for i in path:
        newpath+=(chr(ord(i)+1))
    return newpath

def pickup():
    global storage
    global gripbuffer
    if storage==0:
        '''Code for pickup in storage,pass 6'''
        #ser.write(b'6');
        storage=1
    else:
        '''Code for pickup into buffer,pass 7'''
        #gripbuffer=1
        ser.write(b'7');
def drop():
    global gripbuffer
    global storage
    if gripbuffer==1:
        '''Code for pickup in storage,pass 8'''
        #ser.write(b'8');
        gripbuffer=0
    else:
        '''Code for pickup into buffer,pass 9'''
        #ser.write(b'9');
        storage=0

poss=[]
def updatePos(path):
    if len(path) > 0:
            x = xA
            y = yA
            for i in range(0,len(path)-1):
                j = int(path[i])
                x += dx[j]
                y += dy[j]    
            return [x+1,y+1]
def MoveToPosition(path):
    '''Code for instructions to move'''
    for i in (range(0,len(path)-2)):
        #ser.write(path(i))
        #ser.write(b'5');
        alksjdlasjd=1


def FaceObject(path):
    l=len(path)
    if path[l-1]==1:
       # ser.write(b'd');
        #ser.write(b'5');
       lkssnlknas=1
    elif path[l-1]==2:
        #ser.write(b'b');
        #ser.write(b'5');
        lnlas=1
    elif path[l-1]==3:
        #ser.write(b'c');
        #ser.write(b'5');
        aoiej=1
    else:
        #ser.write(b'a');
        #ser.write(b'5');
        dsmlaij=1
#ser=serial.Serial('COM6',9600)        
current_position=[5,3]
dx = [1, 0, -1, 0]
dy = [0, 1, 0, -1]
doorobjects=[];
areaobjects=[];
img=cv2.imread("test_image4.jpg");
storage=0;
gripbuffer=0;
HSV,x=GetThreshold(img)
GetObjectInfo(x,HSV)

for i in allobjects:
    if i[0][0]==1:
        doorobjects.append(i)
    else:
        areaobjects.append(i)
print(the_map)

for i in doorobjects:
    for j in areaobjects:
        if i[1]==j[1] and i[2]==j[2] and i[3]==i[3] :
            print(i);
            xA =current_position[0]-1;
            yA=current_position[1]-1;
            xB=j[0][0]-1;
            yB=j[0][1]-1;
            the_map[yA][xA]=0;
            the_map[yB][xB]=0;
            print(xA,yA,xB,yB);
            current_route=pathFind(the_map,4,dx,dy,xA,yA,xB,yB);
            the_map[yA][xA]=1;
            the_map[yB][xB]=1;
            latpath=GetNewPath(current_route);
            MoveToPosition(latpath);
            FaceObject(latpath);
            pickup()
            current_position=updatePos(current_route)
            print(current_position)
            xA=current_position[0]-1;
            yA=current_position[1]-1;
            xB=i[0][0]-1;
            yB=i[0][1]-1;
            the_map[yA][xA]=0;
            the_map[yB][xB]=0;
            print(xA,yA,xB,yB);
            current_route=pathFind(the_map,4,dx,dy,xA,yA,xB,yB);
            the_map[yA][xA]=1;
            the_map[yB][xB]=1;
            current_position=updatePos(current_route)
            print(current_position)
            MoveToPosition(GetNewPath(current_route));
            FaceObject(GetNewPath(current_route));
            drop()
            
    



































































































































































































































































































































































































































































































