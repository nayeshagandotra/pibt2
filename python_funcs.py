import matplotlib
matplotlib.use('Agg')  # Set backend to non-interactive
import matplotlib.pyplot as plt
import numpy as np
import re
import sys
import pandas as pd
import subprocess
import csv
import os
import shutil

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

def parse_costs(file_path):
    """
    Parses a text file of costs and splits them into two DataFrames.

    Args:
        file_path (str): Path to the text file containing costs.

    Returns:
        tuple: Two pandas DataFrames (df1, df2).
    """
    with open(file_path, 'r') as file:
        costs = [int(line.strip()) for line in file if line.strip().isdigit()]

    # Create dataframes for alternating sequences
    df1 = pd.DataFrame({'Timestep': range(1, len(costs[0::2]) + 1), 'Costs': costs[0::2]})  # Odd-indexed values
    df2 = pd.DataFrame({'Timestep': range(1, len(costs[1::2]) + 1), 'Costs': costs[1::2]})  # Even-indexed values

    return df1, df2

def plot_costs(df1, df2):
    """
    Plots timestep vs. costs for two DataFrames on the same plot.

    Args:
        df1 (pd.DataFrame): First DataFrame containing 'Timestep' and 'Costs'.
        df2 (pd.DataFrame): Second DataFrame containing 'Timestep' and 'Costs'.
    """
    plt.figure(figsize=(10, 6))

    # Plot df1
    plt.plot(df1['Timestep'], df1['Costs'], label='DF1 Costs', marker='o')

    # Plot df2
    plt.plot(df2['Timestep'], df2['Costs'], label='DF2 Costs', marker='o')

    # Add labels, title, and legend
    plt.xlabel('Timestep')
    plt.ylabel('Costs')
    plt.title('Timestep vs Costs for DF1 (PIBT TSP) and DF2 (OPTI-PIBT TSP)')
    plt.xlim([0, 150])
    plt.legend()

    # Add grid and show the plot
    plt.grid(True)
    plt.show()
    plt.savefig("costs.png", dpi=300, bbox_inches='tight')

def modify_instance_file(instance_file, seed, num_agents, temp_instance_file):
    """
    Modifies the seed in the instance file and saves it to a temporary file.

    Args:
        instance_file (str): Path to the original instance file.
        seed (int): The new seed value.
        temp_instance_file (str): Path to save the modified instance file.
    """
    with open(instance_file, 'r') as infile, open(temp_instance_file, 'w') as outfile:
        for line in infile:
            # Replace the line containing "seed=" with the new seed value
            if line.startswith("seed="):
                outfile.write(f"seed={seed}\n")
            elif line.startswith("agents="):
                outfile.write(f"agents={num_agents}\n")
            else:
                outfile.write(line)

def run_experiment(modified_instance_file, output_file, solver_name):
    """
    Runs the MAPF executable with the modified instance file.

    Args:
        modified_instance_file (str): Path to the modified instance file.
        output_file (str): Path to save the output log.
        solver_name (str): Solver name to use in the experiment.

    Returns:
        tuple: stdout and stderr from the subprocess execution.
    """
    command = ['./build/mapf', '-i', modified_instance_file, '-o', output_file, '-s', solver_name, '-T', str(45000)]
    try:
        # Run the subprocess with a timeout
        result = subprocess.run(command, capture_output=True, text=True, timeout=30)
        return result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        print(f"Process timed out after {30} seconds.")
        return None, None

def check_solution(output_file):
    """
    Checks if the solution was successful by reading and parsing the output file.

    Args:
        output_file (str): Path to the output file generated by the MAPF executable.

    Returns:
        bool: True if "solved=1" is found in the output file, False otherwise.
    """
    try:
        with open(output_file, 'r') as f:
            for line in f:
                if "solved=1" in line.lower():
                    return True
    except FileNotFoundError:
        print(f"Error: Output file {output_file} not found.")
    return False


def batch_runner(instance_file, solver_name, num_runs, output_csv):
    """
    Runs multiple experiments with different seeds and logs results in a CSV file.

    Args:
        instance_file (str): Path to the original instance file.
        solver_name (str): Solver name to use in experiments.
        num_runs (int): Number of experiments to run.
        output_csv (str): Path to save results in CSV format.
    """
    results = []
    temp_instance_file = "temp_instance.txt"

    for na in range(20, 550, 20):  # Loop over different numbers of agents
        for seed in range(num_runs):  # Loop over different seeds
            # Modify the instance file with a new seed and number of agents
            modify_instance_file(instance_file, seed, na, temp_instance_file)

            # Run experiment
            output_file = f"outputs/output_{solver_name}_seed_{seed}_na{na}.txt"
            stdout, stderr = run_experiment(temp_instance_file, output_file, solver_name)

            # Check if solution succeeded by analyzing the output file
            solved = check_solution(output_file)
            results.append({"Solver": solver_name, "Num_agents": na, "Seed": seed, "Solved": solved})

            print(f"Solver: {solver_name}, Seed: {seed}, Num_agents: {na}, Solved: {solved}")
        print(f"Finished for {solver_name} with {na} agents.")

    # Write results to CSV
    with open(output_csv, mode='a', newline='') as csvfile:  # Append results for each solver
        writer = csv.DictWriter(csvfile, fieldnames=["Solver", "Num_agents", "Seed", "Solved"])
        if os.stat(output_csv).st_size == 0:  # Write header only if the file is empty
            writer.writeheader()
        writer.writerows(results)

def plot_success_rates(output_csv):
    """
    Plots success rates side by side for multiple solvers.

    Args:
        output_csv (str): Path to the CSV file containing results.
    """
    # Read the CSV file into a DataFrame
    df = pd.read_csv(output_csv)

    # Group by Solver and Num_agents to calculate success percentage
    success_rates = (
        df.groupby(['Solver', 'Num_agents'])['Solved']
        .apply(lambda x: (x == True).sum() / len(x) * 100)  # Calculate percentage of True
        .unstack(level=0)  # Pivot table to have solvers as columns
    )

    plt.figure(figsize=(12, 8))
    for solver in success_rates.columns:
        leg_label = solver
        if solver == "PIBT": leg_label = "OPTI-PIBT"
        plt.plot(success_rates.index, success_rates[solver], marker='o', linestyle='-', label=leg_label)

    # Add labels, title, and legend
    plt.xlabel('Number of Agents')
    plt.ylabel('Success Percentage (%)')
    plt.title('Success Percentage vs Number of Agents for Different Solvers')
    plt.grid(True, linestyle='--', alpha=0.7)
    plt.legend(title='Solvers')

    # Save the plot as an image
    plt.savefig('323220_comparison.png', dpi=300, bbox_inches='tight')

if __name__ == "__main__":
    original_instance_file = "instances/mapf/sample.txt"  # Original instance file path
    solvers = ["PIBT", "PIBTOLD", "HCA", "PushAndSwap"]  # List of solvers to test
    num_runs = 10  # Number of experiments per solver per number of agents
    output_csv = "results.csv"  # Output CSV file name

    # Clear the previous CSV file if it exists
    if os.path.exists(output_csv):
        os.remove(output_csv)

    # Run experiments for each solver
    for solver in solvers:
        batch_runner(original_instance_file, solver, num_runs, output_csv)

    # Plot success rates for all solvers
    plot_success_rates(output_csv)

    # File path to the text file containing costs PLOTTING COSTS
    # file_path = "costs.txt"  # Replace with the actual path to your file

    # try:
    #     # Parse the costs into two DataFrames
    #     df1, df2 = parse_costs(file_path)

    #     # Display the DataFrames
    #     print("DataFrame 1:")
    #     print(df1)
    #     print("\nDataFrame 2:")
    #     print(df2)

    #     # Plot the costs
    #     plot_costs(df1, df2)

    # except FileNotFoundError:
    #     print(f"Error: The file '{file_path}' was not found.")
    # except Exception as e:
    #     print(f"An error occurred: {e}")


