#!/usr/bin/env python3
import rclpy
import pygame as pg
from heapq import *

from rclpy.node import Node
from std_msgs.msg import String

def get_circle(loc):
    return (loc[0] * TILE + TILE // 2, loc[1] * TILE + TILE // 2)

def get_rect(x, y):
    return x * TILE + 1, y * TILE + 1, TILE - 2, TILE - 2

def get_next_nodes(x, y):
    check_next_node = lambda x, y: True if 0 <= x < cols and 0 <= y < rows else False
    if direction == 4:
        ways = [-1, 0], [0, -1], [1, 0], [0, 1]
    if direction == 8:
        ways = [-1, 0], [0, -1], [1, 0], [0, 1], [-1, 1], [-1, 1], [1, -1], [1, 1]
    return [(map[y + dy][x + dx], (x + dx, y + dy)) for dx, dy in ways if check_next_node(x + dx, y + dy)]

def heuristic(a, b):
   return abs(a[0] - b[0]) + abs(a[1] - b[1])

def get_click_mouse_pos():
    x, y = pg.mouse.get_pos()
    grid_x, grid_y = x // TILE, y // TILE
    pg.draw.circle(sc, pg.Color('red'), *get_circle(grid_x, grid_y))
    click = pg.mouse.get_pressed()
    return (grid_x, grid_y) if click[0] else False

map_zero = [
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        ]
map_cmit = [
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1,99,99,99,99, 1, 1],
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1,99,99,99,99, 1, 1,99, 1, 1, 1,99,99,99, 1, 1, 1,99, 1,99,99, 1, 1,99,99, 1],
        [ 1, 1, 1,99, 1,99,99,99,99, 1, 1,99, 1, 1, 1,99,99,99, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1,99,99,99, 1, 1, 1,99, 1,99,99, 1, 1,99,99, 1],
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1,99,99, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1,99,99, 1, 1,99,99, 1],
        [ 1, 1, 1,99, 1,99,99, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1,99,99, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1,99,99, 1, 1,99,99, 1],
        [ 1, 1, 1,99,99,99,99,99, 1,99,99,99,99,99,99,99, 1,99,99,99,99,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1,99,99, 1, 1,99,99, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [99,99,99,99, 1, 1, 1, 1, 1, 1, 1,99,99,99,99,99,99,99, 1,99,99,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1,99,99,99,99, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1, 1,99,99,99,99, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1,99, 1,99, 1, 1,99,99, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1,99, 1,99, 1, 1,99,99, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        [ 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1,99, 1,99, 1,99, 1, 1,99,99, 1,99, 1, 1, 1, 1, 1, 1, 1, 1],
        ]

map = map_cmit
cols = len(map[0])
rows = len(map)
TILE = 50

direction = 8
if direction == 4:
    pg.display.set_caption('A* algorithm - 4(four) directions')
if direction == 8:
    pg.display.set_caption('A* algorithm - 8(eight) directions')

pg.init()
sc = pg.display.set_mode([cols * TILE, rows * TILE])
clock = pg.time.Clock()

start = (5, 1)
goal =  (25, 1)

bg = pg.Surface((cols * TILE, rows * TILE))
bg.fill((255,255,255))

unit_wall = pg.Surface((TILE, TILE))
unit_start = pg.Surface((TILE, TILE))
unit_goal = pg.Surface((TILE, TILE))

unit_wall = pg.Surface((TILE, TILE))
unit_start.fill((0,0,255))
unit_goal.fill((255,0,0))

for i in range(rows):
    for j in range(cols):
        if i == start[1] and j == start[0]: bg.blit(unit_start, (j * TILE, i * TILE))
        if i == goal[1] and j == goal[0]: bg.blit(unit_goal, (j * TILE, i * TILE))
        if map[i][j] > 8: bg.blit(unit_wall, (j * TILE, i * TILE))


sc.blit(bg, (0, 0))
pg.display.flip()

class Publisher(Node):
    def __init__(self):
        super().__init__('node_publisher')
        self.get_logger().warn("publisher start")

        self.count = 0
        self.queue = []
        heappush(self.queue, (0, start))
        self.cur_cost = (0, 0)
        self.cur_node = (0, 0)
        self.path_head = (0, 0)
        self.path_segment = None
        self.cost_visited = {start: 0}
        self.visited = {start: None}
        self.key_stop = 1
        
        self.graph = {}
        for y, row in enumerate(map):
            for x, col in enumerate(row):
                self.graph[(x, y)] = self.graph.get((x, y), []) + get_next_nodes(x, y)

        self.pub = self.create_publisher(String, "topic_string", 10)
        self.tim = self.create_timer(0.01, self.my_timers)

    def my_timers(self):
        msg = String()
        msg.data = 'text ' + str(self.count)
        self.count += 1
        self.pub.publish(msg)
        self.get_logger().info(msg.data)

        sc.blit(bg, (0, 0))

        [pg.draw.rect(sc, pg.Color('forestgreen'), get_rect(x, y), 1) for x, y in self.visited]
        [pg.draw.rect(sc, pg.Color('orange'), get_rect(*xy), 5) for _, xy in self.queue]

        if self.key_stop:
            if self.queue:
                self.cur_cost, self.cur_node = heappop(self.queue)
                if self.cur_node == goal:
                    self.queue = []
                    self.key_stop = 0

                next_nodes = self.graph[self.cur_node]
                for next_node in next_nodes:
                    neigh_cost, neigh_node = next_node
                    new_cost = self.cost_visited[self.cur_node] + neigh_cost

                    if neigh_node not in self.cost_visited or new_cost < self.cost_visited[neigh_node]:
                        priority = new_cost + heuristic(neigh_node, goal)
                        heappush(self.queue, (priority, neigh_node))
                        self.cost_visited[neigh_node] = new_cost
                        self.visited[neigh_node] = self.cur_node

        self.path_head, self.path_segment = self.cur_node, self.cur_node

        while self.path_segment:
            pg.draw.circle(sc, pg.Color('green'), get_circle(self.path_segment), 10)
            self.path_segment = self.visited.get(self.path_segment)

        pg.draw.circle(sc, pg.Color('blue'), get_circle(start), 10)
        pg.draw.circle(sc, pg.Color('magenta'), get_circle(self.path_head), 10)
        pg.draw.circle(sc, pg.Color('red'), get_circle(goal), 10)

        [exit() for event in pg.event.get() if event.type == pg.QUIT]

        pg.display.flip()

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()