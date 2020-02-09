from heapq import *
import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np
from math import sqrt 
from random import *

#Độ dời để xét các điểm liền kề (các trạng thái tiếp theo)
adj = [(-1, -1), (-1, 0), (-1, 1), (0, -1) , (0, 1), (1, -1), (1, 0), (1, 1)]


###########################################################################################################
#Kiểm tra để phát hiện đi chéo xuyên qua đa giác
def insideThePolygon(neighbor, current, grid):
    x1 , y1 = current
    x2 , y2 = neighbor
    xDiff = x2 - x1
    yDiff = y2 - y1

    if (y2 != y1) and (x2 != x1): #Nếu đi chéo
        value1 = grid[y1 + yDiff][x1]
        value2 = grid[y1][x1 + xDiff]
        #Kiểm tra 2 điểm tiếp xúc với điểm hiện tại và kiểm đang xét có phải cùng thuộc 1 đa giác?
        if (value1 != 0) and (value2 != 0) and (value1 == value2):
            return True

    return False

###########################################################################################################
#Hàm tính Heuritic (theo norm 2) khoảng cách từ điểm đang xét đếm đích
def heuristic(p, goal):
    return sqrt((goal[0] - p[0]) ** 2 + (goal[1] - p[1]) ** 2)

###########################################################################################################
#Hàm tính khoảng cách giữa 2 điểm trên mặt phẳng tọa độ
def distance(p1, p2):
    return sqrt((p2[0] - p1[0]) ** 2 + (p2[1] - p1[1]) ** 2)

###########################################################################################################
#Thuật toán tìm kiếm A*

def astar(grid, start, goal):
    closedList = [] #Tập đóng
    gValue = {start: 0} #Lưu giá trị g (khoảng cách tính từ đích):
    fValue = {start: heuristic(start, goal)} # f = g + h
    openList = [((fValue[start], start))] #Tập mở (dùng cấu trúc priority queue)
    prev = {} #Lưu điểm trước (dùng để truy vết)
    
    while openList:
        current = heappop(openList)[1] #Sử dụng priority queue để lấy ra điểm có f nhỏ nhất trong tập mở
        if current == goal: #Nếu tìm thấy đích (là trạng thái đích)
            route = []
            while current in prev:
                route.append(current)
                current = prev[current]
            return route[::-1] #Trả về lộ trình

        if current in closedList: #Nếu trong điểm đang xét trong tập đóng thì bỏ qua
            continue
        closedList.append(current) #Thêm vào tập đóng

        #Xét các điểm liền kề (các trạng thái tiếp theo)
        #Mở các đỉnh
        for i, j in adj:
            neighbor = current[0] + i, current[1] + j #Điểm liền kề (Trạng thái tiếp theo)

            if neighbor in closedList: #Nếu đã trong tập đóng thì bỏ qua
                continue

            gValueNeighbor = gValue[current] + distance(current, neighbor)  # gValue của điểm này
                           
            if grid[neighbor[1], neighbor[0]] != 0: # kiểm tra trường hợp đụng phải chướng ngại vật
                continue
            #kiểm tra xem nó sắp đi vào đa giác hay không
            if (neighbor[0] != current[0]) and (neighbor[1] != current[1]): # trường hợp nó nằm trên đường chéo
                if insideThePolygon(neighbor, current, grid):
                    continue
            
            #Nếu chưa trong tập mở hoặc đã trong tập mở mà có gValue tốt hơn thì cập nhật giá trị và thêm vào tập mở
            if  (neighbor not in [item[1] for item in openList]) or (gValueNeighbor < gValue.get(neighbor, 0)):
                gValue[neighbor] = gValueNeighbor
                fValue[neighbor] = gValueNeighbor + heuristic(neighbor, goal) # f = g + h
                prev[neighbor] = current
                heappush(openList, (fValue[neighbor], neighbor))
                
    return False

##############################################################
# changeCoordinate: thay đổi tọa độ các polygon trên bản đồ
def changeCoordinate(grid, noThisWay, quantity):
    for i in range(quantity):# chạy lần lượt các đa giác
        value=i+3
        if value in noThisWay: # kiểm tra xem đa giác này có giới hạn việc di chuyển về hướng nào không
            x_limit=noThisWay[value][0]
            y_limit=noThisWay[value][1]
            #xét những hướng x có thể đi
            if x_limit==0:
                new_X= randint(-1,1)
            elif x_limit>0:
                new_X=randint(0,1)
            else: new_X=randint(-1,0)
            # xét những hướng y có thể đi
            if y_limit==0:
                new_Y= randint(-1,1)
            elif y_limit>0:
                new_Y=randint(0,1)
            else: new_Y=randint(-1,0)
        for i in range(grid.shape[1]): #x
            for j in range(grid.shape[0]): #y
                if grid[j,i]==value:
                    grid[j,i]=0
                    grid[j+new_Y, i+new_X]=value
    

############################################
# version 4

def animation(grid, start, goal, quantity):
    openList = [(heuristic(start,goal), start)] #Tập mở (dùng cấu trúc priority queue)
    current=start
    while(1):        
        if current==goal:
            return
        # mở các trạng thái đường đi có thể
        for i,j in adj:
            neighbor= current[0]+i, current[1]+j
            if grid[neighbor[1], neighbor[0]] != 0: # kiểm tra trường hợp đụng phải chướng ngại vật
                continue
             #kiểm tra xem nó sắp đi vào đa giác hay không
            if (neighbor[0] != current[0]) and (neighbor[1] != current[1]): # trường hợp nó nằm trên đường chéo
                if insideThePolygon(neighbor, current, grid):
                    continue
            cost = heuristic(neighbor, goal)
            heappush(openList,(cost, neighbor))
        
        current=heappop(openList)[1]
        plt.scatter(current[0] + 0.5, current[1] + 0.5, color="black")
        # lúc này ta tìm xem những hướng mà biểu đồ không thể di chuyển được
        # (x, 0) là giới hạn ở việc di chuyển trục x , y thoải mái
        # (0,y) là giới hạn di chuyển ở trục y, x thoải mái
        # (x,y) là giới hạn di chuyển cả theo trục x, y
        noThisWay={}
        for i,j in adj:
            value=grid[current[1]+j,current[0]+i] #value có thể cho ta biết được đó là đa giác nào
            if 
            if value >2: # tức là điểm lúc này là đa giác
                #diffX=i
                #diffY=j
                if not value in noThisWay:
                    noThisWay[value]=(i,j)  #đa giác không thể di chuyển theo hướng x này, và hướng y này
                    continue
                # có thể trên kia hướng x, hoặc y chưa nhận giá trị 0 tức là không có cản trở ở trục y
                # Muốn cập nhật lại sự cản trở theo hướng x, hay y
                if diffX==0 and diffX != noThisWay[value][0]: 
                    noThisWay[value][0]=diffX
                if diffY==0 and diffy != noThisWay[value][1]:
                    noThisWay[value][1]=diffY
        
        # Cập nhật lại tọa độ bản đồ mới
        changeCoordinate(grid, noThisWay, quantity)
        cmap = colors.ListedColormap(["white", "black", "red", "green", "blue", "brown", "orange", "pink", "purple", "grey"])
        plt.pcolor(grid,cmap=cmap,edgecolors='k', linewidths=1)
        plt.pause(0.01)
        


       
            


###########################################################################################################
# Tìm kiếm Greedy ( Best-first search)

def greedy(grid, start, goal):
    closedList = []
    prev = {}
    fValue = {start:heuristic(start, goal)} # f = h
    openList = [((fValue[start], start))]

    while openList:
        current = heappop(openList)[1]
        if current == goal:
            route = []

            while current in prev:
                route.append(current)
                current = prev[current]
            return route[::-1]

        if current in closedList:
            continue
        closedList.append(current)

        for i, j in adj:
            neighbor = current[0] + i, current[1] + j
            fValueNeighbor = heuristic(neighbor, goal)  # f = h      
            if grid[neighbor[1],neighbor[0]] != 0: # kiểm tra trường hợp đụng phải chướng ngại vật
                continue
            
            if (neighbor[0] != current[0]) and (neighbor[1] != current[1]): # trường hợp nó nằm trên đường chéo
                if insideThePolygon(neighbor, current, grid): # kiểm tra xem điểm này có nằm trong đa giác hay chưa
                    continue
            
            #Nếu nằm trong tập đóng hoặc đang mở thì bỏ qua
            if (neighbor in closedList) or (neighbor in [item[1] for item in openList]):
                continue

            fValue[neighbor] = fValueNeighbor
            prev[neighbor] = current
            heappush(openList, (fValue[neighbor], neighbor))
    
    return False

###########################################################################################################
# Tìm kiếm mù UCS ( uniform-cost search)

def UCS(grid, start, goal):
    closedList = []
    gValue = {start:0}
    openList = [(gValue[start], start)]
    prev = {}

    while openList:
        current = heappop(openList)[1]
        if current == goal:
            route = []
            while current in prev:
                route.append(current)
                current = prev[current]
            return route[::-1]

        if current in closedList:
            continue
        closedList.append(current)

        for i, j in adj:
            neighbor = current[0] + i, current[1] + j
            gValueNeighbor = gValue[current] + distance(current, neighbor)  # khoảng cách

            if grid[neighbor[1], neighbor[0]] != 0: # kiểm tra trường hợp đụng phải chướng ngại vật
                continue

            if (neighbor[0] != current[0]) and (neighbor[1] != current[1]): # trường hợp nó nằm trên đường chéo
                if insideThePolygon(neighbor, current, grid): # kiểm tra xem điểm này có nằm trong đa giác hay chưa
                    continue

            #Nếu nằm trong tập đóng thì bỏ qua
            # đã tồn tại và đường đi tới nó hiện tại ngắn hơn đường đi mới tìm được thì không xét
            if neighbor in closedList: 
                continue

            #Nếu chưa trong tập mở hoặc đang trong tập mở mà có giá trị tốt hơn thì push vào tập mở
            if (neighbor not in [item[1] for item in openList]) or (gValueNeighbor < gValue.get(neighbor,0)):
                gValue[neighbor] = gValueNeighbor
                prev[neighbor] = current
                heappush(openList, (gValue[neighbor],neighbor))
    
    return False

###########################################################################################################
# Hàm tính chi phí đường đi. Đi thẳng thì tính chi phí bằng 1, đi chéo thì tính chi phí bằng sqrt(2)

def costOfPath(path, start):
    cost = 0
    for i in range(len(path) - 1):
        if (path[i][0] == path[i+1][0]) or ( path[i][1] == path[i+1][1]):
            cost += 1
        else:
            cost += sqrt(2)

    if (start[0] == path[0][0]) or (start[1] == path[0][1]):
        cost += 1
    else:
        cost += sqrt(2)

    return cost

###########################################################################################################
# shorttestPath algorithm (tổng chi phí đi qua các điểm đón và về đích là nhỏ nhất)
# áp dụng thuật toán astar cho mỗi lần tìm đường (đi đến điểm đón có chi phí nhỏ nhất đối với điểm đang xét)
# pickupPoint là danh sách các điểm đón
# Hàm trả về tổng chi phí và lộ trình


def shorttestPath(grid, start, goal, pickupPoint):
    visited = [] #Danh sách các điểm đón đã đi qua
    finalPath = [] #Đường đi của bài toán
    newStart = start 
    length = len(pickupPoint)
    sumCost = 0 #Tổng chi phí
    while(length > 0):
        routes = {} #Đường đi ngắn nhất từ điểm đang xét đến các điểm đón chưa đi qua
        pq = [] # Dùng priority queue để lưu điểm đón và chi phí từ điểm đang xét đến nó
        for point in pickupPoint:
            if point not in visited:
                path = astar(grid, newStart, point)# đương đi ngắn nhất từ điểm đang xét đến nó
                if path == False: #Không tìm thấy đường đi
                    return (-1, False) 
                routes[point] = path
                # ở đây phải có 1 hàm tính chi phí đường đi
                heappush(pq, (costOfPath(path, newStart)-heuristic(point, goal), point) )
        cost, pickPoint = heappop(pq) #Cho ra điểm có chi phí nhỏ nhất
        sumCost += cost
        finalPath += routes[pickPoint]
        visited.append(pickPoint) #Đánh dấu là đã đi đến
        newStart = pickPoint
        length -= 1
    
    #Tìm lộ trình từ điểm đón cuối dùng đến đích
    path = astar(grid, newStart, goal)
    if path == False:
            return (-1, False)
    sumCost += costOfPath(path, newStart)
    finalPath += path

    #Trả về tổng chi phí và lộ trình
    return (sumCost, finalPath)
    

###########################################################################################################
#Hàm trả về danh sách các điểm để nối 2 điểm p1, p2
#Hàm sử dụng thuật toán Bresenham để nối 2 điểm tạo thành đường chéo

def matchTwoPoint(p1, p2):
    res = []
    xDiff = p2[0] - p1[0]
    yDiff = p2[1] - p1[1]
    if yDiff == 0:
        for i in range(abs(xDiff) - 1):
            if xDiff > 0:
                x = p1[0] + 1 + i
            else:
                x = p1[0] - 1 - i
            y = p1[1]
            res.append((x,y))
        return res
    elif xDiff == 0:
        for i in range(abs(yDiff) - 1):
            if yDiff > 0:
                y = p1[1] + 1 + i
            else:
                y = p1[1] - 1 - i

            x = p1[0]
            res.append((x,y))
        return res
    else:
        res = bresenham(p1, p2)
    return res

###########################################################################################################
#Thuật toán Bresenham

def bresenham(p1, p2):
    x1, y1 = p1
    x2, y2 = p2

    steep = abs(y2 - y1) > abs(x2 - x1)
    if steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    if x1 > x2:
        x1 , x2 = x2, x1
        y1 , y2 = y2, y1

    Dx = x2 - x1
    Dy = abs(y2 - y1)
    err = Dx/2 - Dy
    if y1 < y2:
        ystep = 1
    else:
        ystep = -1
    y = y1
    if err < 0:
        y = y + ystep
        err = err + Dx
    res = []

    for x in range(x1 + 1, x2):
        if steep:
            res.append((y,x))
        else:
            res.append((x,y))
        err = err - Dy
        if err < 0:
            y = y + ystep
            err = err + Dx
    return res




###########################################################################################################
#Vẽ hình (tạo bản đồ)

def drawing(grid, fi, quantity):
    #tạo đường biên
    grid[0,:] = 1
    grid[(grid.shape[0]-1),:] = 1
    grid[:,0] = 1
    grid[:,(grid.shape[1]-1)] = 1
    
    #đọc vào số hình
    for k in range(quantity):
        line = fi.readline()

        token = line.split(',')
        vertices = []
        pair_Index = [] #Chỉ số các cặp đỉnh của đa giác theo chiều kim đồng hồ để xét nối với nhau


        for i in range(int(len(token) / 2)):
            x = int(token[2*i])
            y = int(token[2*i +  1])
            vertices.append((x, y))
            grid[y][x] = k + 3


        #nếu trường hợp là vẽ đoạn thẳng
        if len(vertices) == 2:
            pair_Index.append((0,1))
        else:
            for i in range(len(vertices) - 1):
                pair_Index.append((i, i + 1))
            pair_Index.append((len(vertices) - 1, 0))

        for i,j in pair_Index:

            for x, y in matchTwoPoint(vertices[i], vertices[j]):
                grid[y][x] = k + 3

    fi.close()






