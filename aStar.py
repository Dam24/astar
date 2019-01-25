
class AStarGraph(object):
    def __init__(self):
        self.barriers = []
        self.barriers.append(
            [(2, 4), (2, 5), (2, 6), (3, 6), (4, 6), (5, 6), (5, 5), (5, 4), (5, 3), (5, 2), (4, 2), (3, 2)])

    def heuristic(self, start, goal):
        D = 1
        D2 = 1
        dx = abs(start[0] - goal[0])
        dy = abs(start[1] - goal[1])
        return D * (dx + dy) + (D2 - 2 * D) * min(dx, dy)

    def get_vertex_neighbours(self, pos):
        n = []
        neighbors = [(0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)]
        for dx, dy in neighbors:
            x2 = pos[0] + dx
            y2 = pos[1] + dy
            if x2 < 0 or x2 > 7 or y2 < 0 or y2 > 7:
                continue
            n.append((x2, y2))
        return n

    def move_cost(self, a, b):
        for barrier in self.barriers:
            if b in barrier:
                return 100
        return 1


def AStarSearch(start, end, graph):
    G = {}  # Actual movement cost to each position from the start position
    F = {}  # Estimated movement cost of start to end going via this position
    act_cost = {}
    est_cost = {}

    G[start] = 0
    F[start] = graph.heuristic(start, end)

    closed_list = []
    open_list = [start]
    came_from = {}

    while len(open_list) > 0:
        current = None
        currentFscore = None
        for pos in open_list:
            if current is None or F[pos] < currentFscore:
                currentFscore = F[pos]
                current = pos

        if current == end:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            path.reverse()
            return path, F[end]

        open_list.remove(current)
        closed_list.add(current)

        for neighbour in graph.get_vertex_neighbours(current):
            if neighbour in closed_list:
                continue
            candidateG = act_cost[current] + graph.move_cost(current, neighbour)

            if neighbour not in open_list:
                open_list.add(neighbour)
            elif candidateG >= act_cost[neighbour]:
                continue

            # Adopt this G score
            came_from[neighbour] = current
            act_cost[neighbour] = candidateG
            H = graph.heuristic(neighbour, end)
            F[neighbour] = G[neighbour] + H

    raise RuntimeError("It was not possible find the solution")


if __name__ == "__main__":
    graph = AStarGraph()
    result, cost = AStarSearch((0, 0), (4, 7), graph)
    print("route", result)
    print("cost", cost)
