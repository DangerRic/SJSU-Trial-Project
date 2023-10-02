# -*- coding: utf-8 -*-
"""
Created on Sat Sep 25 17:07:51 2023

@author: ricardo espinosa
"""

import pygame
import random
import math
from queue import PriorityQueue

# Constants
WIDTH = 800  # Width of the grid
HEIGHT = 600  # Height of the grid
GRID_SIZE = 20  # Size of each grid cell
NUM_ROWS = HEIGHT // GRID_SIZE
NUM_COLS = WIDTH // GRID_SIZE

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
ORANGE = (255, 165, 0)
PURPLE = (128, 0, 128)

# Initialize pygame
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
clock = pygame.time.Clock()


class Rover:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.path = []  # Initialize the path as an empty list

    def move(self):
        # Check if a path exists
        if self.path:
            # Get the next position from the path
            next_position = self.path.pop(0)
            # Update the rover position
            self.x, self.y = next_position

    def draw(self):
        pygame.draw.rect(screen, BLUE, (self.x * GRID_SIZE, self.y * GRID_SIZE, GRID_SIZE, GRID_SIZE))


class Obstacle:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def draw(self):
        pygame.draw.rect(screen, BLACK, (self.x * GRID_SIZE, self.y * GRID_SIZE, GRID_SIZE, GRID_SIZE))

#calculates distance between two points
def heuristic(p1, p2):
    x1, y1 = p1
    x2, y2 = p2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2) #uses Pyth. theorem to calc. line


def astar_algorithm(start, target, obstacles):
    open_set = PriorityQueue() #stores the positions & prioritizes them
    open_set.put((0, start)) #adds start position to the set
    came_from = {} #keeps track of the path taken
    g_score = {pos: float('inf') for pos in obstacles + [start, target]}  # dictionary that holds all positions. score is value of reaching each position
    g_score[start] = 0
    f_score = {pos: float('inf') for pos in obstacles + [start, target]}  # score is the value of reaching the endpoint from each position
    f_score[start] = heuristic(start, target)

#continuous loop until all positions have been checked
    while not open_set.empty():
        current = open_set.get()[1] #gets position with lowerst score

        if current == target: 
            path = []
#recreates the path from end point to where the rover came from
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.reverse()
            return path

#updates the scores in the algorithm by searching all position next to rover
        for dx, dy in [(0, -1), (0, 1), (-1, 0), (1, 0)]: #loop that looks at each postion
            neighbor = current[0] + dx, current[1] + dy
#checks if postitions are inside the grid and is not an obstacle
            if 0 <= neighbor[0] < NUM_COLS and 0 <= neighbor[1] < NUM_ROWS and (neighbor not in obstacles):
                tentative_g_score = g_score[current] + 1 #adds to the score value when moving to a new position
#compares scores to see if a better path is available
                if tentative_g_score < g_score.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, target)
#makes sure that neighboring position is added to the set
                    if neighbor not in [open_set_item[1] for open_set_item in open_set.queue]:
                        open_set.put((f_score[neighbor], neighbor))

    return None


def draw_grid():
    for x in range(0, WIDTH, GRID_SIZE):
        pygame.draw.line(screen, BLACK, (x, 0), (x, HEIGHT))
    for y in range(0, HEIGHT, GRID_SIZE):
        pygame.draw.line(screen, BLACK, (0, y), (WIDTH, y))


def generate_obstacles(num_obstacles):
    obstacles = []
    for _ in range(num_obstacles):
        x = random.randint(0, NUM_COLS - 1)
        y = random.randint(0, NUM_ROWS - 1)
        obstacles.append(Obstacle(x, y))
    target_position = (random.randint(0, NUM_COLS - 1), random.randint(0, NUM_ROWS - 1))

    return obstacles, target_position


def is_collision(rover, obstacles):
    for obstacle in obstacles:
        if rover.x == obstacle.x and rover.y == obstacle.y:
            return True
    return False

def evaluate_performance(steps_taken, collisions):
    efficiency = (steps_taken - collisions) / steps_taken * 100 if steps_taken > 0 else 0
 

    print("Simulation results:")
    print("Steps taken:", steps_taken)
    print("Collisions:", collisions)
    print("Efficiency: {:.2f}%".format(efficiency))

def main():
    rover = Rover(0, 0)
    obstacles, target_position = generate_obstacles(71)

    steps_taken = 0
    collisions = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                return

        if rover.x == target_position[0] and rover.y == target_position[1]:
            # Rover has reached the target position
            evaluate_performance(steps_taken, collisions)
            break

        if not rover.path:
            start = (rover.x, rover.y)
            target = target_position
            rover.path = astar_algorithm(start, target, [(obs.x, obs.y) for obs in obstacles])

        rover.move()
        steps_taken += 1
        if is_collision(rover, obstacles):
            collisions += 1     

        screen.fill(WHITE)
        draw_grid()
        for obstacle in obstacles:
            obstacle.draw()
            rover.draw()
        pygame.draw.rect(screen, GREEN, (target_position[0] * GRID_SIZE, target_position[1] * GRID_SIZE, GRID_SIZE, GRID_SIZE))
        pygame.display.flip()
        
        pygame.time.delay(int(0.15 * 1000))  # Delay in milliseconds
        clock.tick(70)

if __name__ == '__main__':
    main()
       