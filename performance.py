import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from datetime import datetime
import numpy as np

# Function to calculate performance metrics
def calculate_performance(environment_data, control_data):
    performance_metrics = []

    # Group control data by User ID and Case
    grouped = control_data.groupby(["User ID", "Case"])

    for (user_id, case), group in grouped:
        # Extract relevant data for this user and case
        case_data = environment_data[(environment_data["User ID"] == user_id) & (environment_data["Case"] == case)]
        ideal_path = eval(case_data["Ideal Path Coordinates"].values[0])  # Convert string to list of tuples
        force_thresholds = case_data["Force Threshold"].values

        # Calculate metrics
        time_taken = (group["Timestamp"].max() - group["Timestamp"].min()).total_seconds()
        avg_force = group["Applied Force"].mean()
        max_force = group["Applied Force"].max()
        avg_velocity = group["Velocity"].mean()
        max_velocity = group["Velocity"].max()
        avg_jerk = group["Jerk"].mean()
        max_jerk = group["Jerk"].max()
        errors = group[group["Error Event"] != "None"].shape[0]
        deviations = calculate_deviation(group, ideal_path)  # Custom function to calculate deviation
        collisions = detect_collisions(group, case_data)  # Custom function to detect collisions

        # Append metrics to the list
        performance_metrics.append({
            "User ID": user_id,
            "Case": case,
            "Time Taken (s)": time_taken,
            "Average Force (N)": avg_force,
            "Max Force (N)": max_force,
            "Average Velocity (m/s)": avg_velocity,
            "Max Velocity (m/s)": max_velocity,
            "Average Jerk (m/s³)": avg_jerk,
            "Max Jerk (m/s³)": max_jerk,
            "Errors": errors,
            "Deviations (mm)": deviations,
            "Collisions": collisions,
        })

    return pd.DataFrame(performance_metrics)

# Function to calculate deviation from the ideal path
def calculate_deviation(control_data, ideal_path):
    deviations = []
    for _, row in control_data.iterrows():
        x, z = row["X Position"], row["Z Position"]
        # Calculate the minimum distance to the ideal path
        distances = [np.sqrt((x - px) ** 2 + (z - pz) ** 2) for (px, pz) in ideal_path]
        min_distance = min(distances)  # Find the minimum distance
        deviations.append(min_distance)

    return np.mean(deviations) * 1000  # Convert to millimeters

# Function to detect collisions with critical structures
def detect_collisions(control_data, case_data):
    safe_zones = eval(case_data["Safe Zone Coordinates"].values[0])  # Convert string to list of tuples
    collisions = 0
    for _, row in control_data.iterrows():
        x, z = row["X Position"], row["Z Position"]
        for (sx, sz) in safe_zones:
            if x == sx and z == sz:
                collisions += 1
                break
    return collisions

# Function to generate visualizations
def generate_visualizations(performance_metrics, control_data):
    # Set the style for seaborn
    sns.set(style="whitegrid")

    # Plot 1: Bar Plot for Time Taken (Case-Specific)
    plt.figure(figsize=(10, 6))
    sns.barplot(x="User ID", y="Time Taken (s)", hue="Case", data=performance_metrics)
    plt.title("Time Taken to Complete Each Case")
    plt.xlabel("User ID")
    plt.ylabel("Time Taken (s)")
    plt.show()

    # Plot 2: Bar Plot for Average Force (Case-Specific)
    plt.figure(figsize=(10, 6))
    sns.barplot(x="User ID", y="Average Force (N)", hue="Case", data=performance_metrics)
    plt.title("Average Force Applied in Each Case")
    plt.xlabel("User ID")
    plt.ylabel("Average Force (N)")
    plt.show()

    # Plot 3: Bar Plot for Average Velocity (Case-Specific)
    plt.figure(figsize=(10, 6))
    sns.barplot(x="User ID", y="Average Velocity (m/s)", hue="Case", data=performance_metrics)
    plt.title("Average Velocity in Each Case")
    plt.xlabel("User ID")
    plt.ylabel("Average Velocity (m/s)")
    plt.show()

    # Plot 4: Scatter Plot for Force vs. Velocity (Case-Specific)
    plt.figure(figsize=(10, 6))
    sns.scatterplot(x="Applied Force", y="Velocity", hue="Case", data=control_data)
    plt.title("Force vs. Velocity")
    plt.xlabel("Applied Force (N)")
    plt.ylabel("Velocity (m/s)")
    plt.show()

# Function to compare performance
def compare_performance(performance_metrics):
    # Save performance metrics to a CSV file
    performance_metrics.to_csv("performance_metrics.csv", index=False)
    print("\nPerformance metrics saved to 'performance_metrics.csv'.")

# Main function
def main():
    # Load environment and control data from CSV files
    environment_data = pd.read_csv("environment_log.csv")
    control_data = pd.read_csv("control_log.csv")

    # Convert timestamp strings to datetime objects
    control_data["Timestamp"] = pd.to_datetime(control_data["Timestamp"])

    # Calculate performance metrics
    performance_metrics = calculate_performance(environment_data, control_data)

    # Generate visualizations
    generate_visualizations(performance_metrics, control_data)

    # Compare performance across users
    compare_performance(performance_metrics)

if __name__ == "__main__":
    main()