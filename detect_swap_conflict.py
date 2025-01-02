import matplotlib.pyplot as plt

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

import matplotlib.pyplot as plt

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

# Main code
if __name__ == "__main__":
    file_paths = ["costs_wskip.txt", "costs_woskip.txt"]  # Replace with the paths to your .txt files
    labels = ["with skip 1", "without skip 2"]  # Replace with appropriate labels for the datasets

    # Plot timestep vs value for multiple datasets
    plot_multiple_datasets(file_paths, labels)



