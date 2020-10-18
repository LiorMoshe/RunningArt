# Python3 program to find the minimum distance from a
# "1" in binary matrix.
import numpy as np
MAX = 1000
INT_MAX = (2 ** 32)

# distance matrix which stores the distance of
# nearest '1'


# Function to find the nearest '1'
def nearestOne(mat):
    # two array when respective values of newx and
    # newy are added to (i,j) it gives up, down,
    # left or right adjacent of (i,j) cell

    dist = np.zeros_like(mat, dtype=np.float64)
    newx = [-1, 0, 1, 0]
    newy = [0, -1, 0, 1]
    m = dist.shape[0]
    n = dist.shape[1]

    # queue of pairs to store nodes for bfs
    q = []

    # traverse matrix and make pair of indices of
    # cell (i,j) having value '1' and push them
    # in queue
    for i in range(m):
        for j in range(n):
            dist[i][j] = INT_MAX
            if (mat[i][j] == 1):
                # distance of '1' from itself is always 0
                dist[i][j] = 0

                # make pair and push it in queue
                q.append([i, j])

                # now do bfs traversal
    # pop element from queue one by one until it gets empty
    # pair element to hold the currently popped element
    poped = []
    while (len(q)):
        poped = q[0]
        q.pop(0)

        # coordinate of currently popped node
        x = poped[0]
        y = poped[1]

        # now check for all adjancent of popped element
        for i in range(4):

            adjx = x + newx[i]
            adjy = y + newy[i]

            # if new coordinates are within boundary and
            # we can minimize the distance of adjacent
            # then update the distance of adjacent in
            # distance matrix and push this adjacent
            # element in queue for further bfs
            if (adjx >= 0 and adjx < m and adjy >= 0 and
                    adjy < n and dist[adjx][adjy] > dist[x][y] + 1):
                # update distance
                dist[adjx][adjy] = dist[x][y] + 1
                q.append([adjx, adjy])

    for i in range(dist.shape[0]):
        for j in range(dist.shape[1]):
            if dist[i][j] == 0:
                dist[i][j] = 1
            else:
                dist[i][j] = 1 / (dist[i][j]+1)
    return dist


if __name__=="__main__":
    m = 3
    n = 4
    mat = np.array([[0, 0, 0, 1], [0, 0, 1, 1], [0, 1, 1, 0]])

    # Fills values in dist[][]
    dist = nearestOne(mat)

    # prdistance matrix
    print(dist)
