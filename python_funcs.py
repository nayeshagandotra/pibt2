import matplotlib
matplotlib.use('Agg')  # Set backend to non-interactive
import matplotlib.pyplot as plt
import numpy as np
import re
import sys

def detect_swap_conflicts(file_path):
    with open(file_path, 'r') as file:
        lines = file.readlines()
    
    # Parse the solution string into a list of lists of agent positions
    agent_positions = [line.strip().split(',') for line in lines if line.strip().startswith('(')]
    
    # Convert string positions to tuples of integers
    for time_step in agent_positions:
        for i, pos in enumerate(time_step):
            x, y = pos.strip('()').split(',')
            time_step[i] = (int(x), int(y))
    
    # Check for swap conflicts
    for t in range(len(agent_positions) - 1):
        current_positions = agent_positions[t]
        next_positions = agent_positions[t + 1]
        
        for i in range(len(current_positions)):
            for j in range(i + 1, len(current_positions)):
                if current_positions[i] == next_positions[j] and current_positions[j] == next_positions[i]:
                    print(f"Swap conflict detected at time step {t} between agents {i} and {j}")
                    return True
    
    print("No swap conflicts detected.")
    return False

# Function to read numbers from a text file and return them as a list
def read_numbers_from_file(file_path):
    with open(file_path, 'r') as file:
        numbers = [int(line.strip()) for line in file if line.strip().isdigit()]
    return numbers

# Function to plot timestep vs value for multiple datasets
def plot_multiple_datasets(file_paths, labels):
    plt.figure(figsize=(10, 6))

    for file_path, label in zip(file_paths, labels):
        numbers = read_numbers_from_file(file_path)
        timesteps = list(range(len(numbers)))  # Generate timesteps as indices
        plt.plot(timesteps, numbers, marker='o', linestyle='-', markersize=4, label=label)

    plt.xlabel('Timestep')
    plt.ylabel('Value')
    plt.title('Timestep vs Value')
    plt.grid(True, which='both', linestyle='--', linewidth=0.5)
    plt.legend()
    plt.savefig("costs.png", format='png')

import matplotlib.pyplot as plt
import sys

def parse_massif(file):
    time_points = []
    snapshots = []
    current_snapshot = {"heap": 0, "extra": 0, "stacks": 0}  # Default values for keys

    with open(file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('time='):
                time_points.append(int(line.split('=')[1]))
            elif line.startswith('mem_heap_B='):
                current_snapshot['heap'] = int(line.split('=')[1])
            elif line.startswith('mem_heap_extra_B='):
                current_snapshot['extra'] = int(line.split('=')[1])
            elif line.startswith('mem_stacks_B='):
                current_snapshot['stacks'] = int(line.split('=')[1])
            elif line.startswith('#-----------'):  # End of a snapshot
                snapshots.append(current_snapshot)
                current_snapshot = {"heap": 0, "extra": 0, "stacks": 0}  # Reset for next snapshot

    # Ensure the last snapshot is added if not already added
    if current_snapshot not in snapshots:
        snapshots.append(current_snapshot)

    # Synchronize time_points and snapshots lengths
    if len(time_points) > len(snapshots):
        time_points = time_points[:len(snapshots)]
    elif len(snapshots) > len(time_points):
        snapshots = snapshots[:len(time_points)]

    return time_points, snapshots

def plot_massif(time_points, snapshots, output_file):
    heap = [s.get('heap', 0) for s in snapshots]
    extra = [s.get('extra', 0) for s in snapshots]
    stacks = [s.get('stacks', 0) for s in snapshots]

    plt.stackplot(time_points, heap, extra, stacks, labels=['Heap', 'Extra', 'Stacks'])
    plt.xlabel('Time')
    plt.ylabel('Memory (bytes)')
    plt.legend(loc='upper left')
    plt.title('Massif Memory Usage')
    plt.savefig(output_file)
    print(f"Graph saved as {output_file}")

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: python massif_graph.py <massif.out.file> <output.png>")
        sys.exit(1)
    massif_file = sys.argv[1]
    output_png = sys.argv[2]

    time_points, snapshots = parse_massif(massif_file)
    plot_massif(time_points, snapshots, output_png)





