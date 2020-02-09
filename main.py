import matplotlib.pyplot as plt
from matplotlib import colors
import numpy as np 
from algorithmForFindingTheWay import drawing, astar, greedy, UCS, shorttestPath, costOfPath

def processing():
    #đọc file
    fi = open('input.txt','r')
    line = fi.readline()
    token = line.split(',')
    n = int(token[1])
    m = int(token[0])
    #Tao luoi
    
    grid = np.zeros((n+1,m+1))

    line = fi.readline()
    token = line.split(',')
    start = (int(token[0]),int(token[1])) # đỉnh bắt đầu
    goal = (int(token[2]),int(token[3]))   # đỉnh kết thúc
    cost = -1 #Chi phí đường đi

    if len(token) > 4: #Nếu có các điểm đón
        pickupPoint = []
        i = 4
        while(i < len(token)):
            pickupPoint.append((int(token[i]),int(token[i+1])))
            i = i + 2

        line = fi.readline()
        token = line.split(',')
        quantity = int(token[0])
        # Thuật toán vẽ
        drawing(grid, fi, quantity)
        cost, route = shorttestPath(grid, start, goal, pickupPoint)
        #vẽ các điểm đón ra
        for i,j in pickupPoint:
            grid[j,i] = quantity + 3
    else:
        line = fi.readline()
        token = line.split(',')
        quantity = int(token[0]) # số lượng đa giác
        # Thuật toán vẽ
        drawing(grid, fi, quantity)
        # áp dụng thuật toán astar
        #route = astar(grid, start, goal)
        #route = greedy(grid, start, goal)
        route = UCS(grid, start, goal)
        if route != False: #Nếu tìm thấy đường đi
            cost = costOfPath(route, start)
    

    if route == False:
        print('---> No way found')
    else:
        print('Cost: ' + str(cost))

     
    grid[start[1]][start[0]] = 2
    grid[goal[1]][goal[0]] = 2


    #Visualize

    cmap = colors.ListedColormap(["white", "black", "red", "green", "blue", "brown", "orange", "pink", "purple", "grey"])
    plt.figure(figsize=(7,7))
    plt.pcolor(grid,cmap=cmap,edgecolors='k', linewidths=1)
    plt.xticks(np.arange(0, m + 1, step = 1))
    plt.yticks(np.arange(0, n + 1, step = 1))

    plt.scatter(start[0] + 0.5, start[1] + 0.5, marker="$S$", color="yellow", s=150)
    plt.scatter(goal[0] + 0.5, goal[1] + 0.5,  marker="$G$", color="yellow", s=150)

    #Vẽ đường đi
    if route != False:
        visited = []
        for i in range(len(route)):
            if (route[i]) in visited:
                plt.scatter(route[i][0] + 0.5, route[i][1] + 0.5, color="red")
            else:
                plt.scatter(route[i][0] + 0.5, route[i][1] + 0.5, color="black")
                visited.append(route[i])
            plt.pause(0.01)

    if cost != -1:
        plt.title('Cost: ' + str(round(cost, 2)))
    else:
        plt.title('No way found')

    plt.show()





if __name__=='__main__':
    processing()
