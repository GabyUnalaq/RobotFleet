import matplotlib.pyplot as plt
import numpy as np
import sys
import json

"""
This script reads metrics data from a JSON file and plots various metrics related to robot control.
To enable metrics recording, run "ctrl.record_metrics()" in your script.

Metrics Data:
timestamp: (N,)
dist_to_target: (N,)
target_reached: (M)  # timestamps for when the targets were reached
smoothness: (N,)
effort: (N,)
inter-robot_distance: (N,)
"""

SAVE_PLOT = True  # Set to False to display the plot instead of saving it
metrics_path = "metrics_FlockController.json"  # metrics_FlockController metrics_DWAFlockingController
metrics_data = None

def read_data():
    global metrics_data, metrics_path
    try:
        with open(metrics_path, 'r') as f:
            metrics_data = json.load(f)
    except FileNotFoundError:
        print(f"Metrics file '{metrics_path}' not found.")
        sys.exit(1)
    except json.JSONDecodeError:
        print(f"Error decoding JSON from '{metrics_path}'.")
        sys.exit(1)


def plot_metrics(color=None, save=False):
    global metrics_path, metrics_data
    timestamp = np.array(metrics_data["timestamp"])
    
    fig, axs = plt.subplots(2, 2, figsize=(12, 8))
    axs = axs.flatten()

    # 1. Distance to target
    axs[0].plot(timestamp, metrics_data["dist_to_target"], label="value", color=color)
    axs[0].set_title("Distance to Target")
    axs[0].set_xlabel("Time (s)")
    axs[0].set_ylabel("Distance (m)")
    axs[0].legend()

    # 2. Inter-robot distance
    axs[1].plot(timestamp, metrics_data["inter-robot_distance"], label=None, color=color)
    axs[1].set_title("Average Inter-Robot Distance")
    axs[1].set_xlabel("Time (s)")
    axs[1].set_ylabel("Distance (m)")
    axs[1].legend()

    # 3. Smoothness
    axs[2].plot(timestamp, metrics_data["smoothness"], label=None, color=color)
    axs[2].set_title("Trajectory Smoothness")
    axs[2].set_xlabel("Time (s)")
    axs[2].set_ylabel("Smoothness (Δvelocity)")
    axs[2].legend()

    # 4. Effort
    axs[3].plot(timestamp, metrics_data["effort"], label=None, color=color)
    axs[3].set_title("Control Effort")
    axs[3].set_xlabel("Time (s)")
    axs[3].set_ylabel("Total Control Magnitude")
    axs[3].legend()

    # Add a vertical line when target is reached
    target_times = metrics_data.get("target_reached", [])
    for i, target_time in enumerate(target_times):
        for ax in axs:
            if i == 0:
                ax.axvline(x=target_time, color='gray', linestyle='--', label='Target Reached')
            else:
                ax.axvline(x=target_time, color='gray', linestyle='--')
    axs[0].legend()

    plt.tight_layout()

    if SAVE_PLOT:
        save_path = metrics_path.replace(".json", "_plots.png")
        plt.savefig(save_path, dpi=300)
        print(f"Metrics plots saved to {save_path}")
    else:
        plt.show()


if __name__ == "__main__":
    read_data()
    plot_metrics()