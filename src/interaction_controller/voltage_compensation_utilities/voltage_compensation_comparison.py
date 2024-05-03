import argparse
import rosbag
import seaborn as sns
import matplotlib.pyplot as plt
import pandas as pd

def process_timestamps(timestamps):
    # Shift timestamps to start from 0
    start_time = timestamps[0]
    shifted_timestamps = [t - start_time for t in timestamps]
    return shifted_timestamps

def extract_data_from_rosbags(rosbag1, rosbag2, topic):
    # Initialize lists to store the extracted data and sources
    altitude_error = []
    voltage_comp = []
    group = []
    without_comp_time = []
    with_comp_time = []

    # Extract data from ROS Bag 1
    with rosbag.Bag(rosbag1, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            if hasattr(msg, 'vector') and hasattr(msg.vector, 'z'):
                altitude_error.append(msg.vector.z)
                voltage_comp.append('without')
                without_comp_time.append(t.to_sec())
                group.append(0)

    # Extract data from ROS Bag 2
    with rosbag.Bag(rosbag2, 'r') as bag:
        for _, msg, t in bag.read_messages(topics=[topic]):
            if hasattr(msg, 'vector') and hasattr(msg.vector, 'z'):
                altitude_error.append(msg.vector.z)
                voltage_comp.append('with')
                with_comp_time.append(t.to_sec())
                group.append(0)

    # Create a DataFrame with the data and sources
    df = pd.DataFrame({'Altitude error': altitude_error, 'Voltage Compensation': voltage_comp, "Group": group})

    # Create separate figures for each case
    without_comp_data = df[df['Voltage Compensation'] == 'without']
    without_comp_shifted_timestamps = process_timestamps(without_comp_time)
    with_comp_data = df[df['Voltage Compensation'] == 'with']
    with_comp_shifted_timestamps = process_timestamps(with_comp_time)

    # Create a figure with two subplots
    fig, axes = plt.subplots(2, 1, figsize=(15, 10))

    # Plot timestamps for the case without voltage compensation
    axes[0].plot(without_comp_shifted_timestamps, without_comp_data['Altitude error'], label='No Voltage Compensation')
    axes[0].plot(with_comp_shifted_timestamps, with_comp_data['Altitude error'], label='Voltage Compensation')
    axes[0].set_ylabel('Altitude Error')
    axes[0].set_xlabel('Time')
    axes[0].legend()
    axes[0].grid(True)

    # Plot a split violin plot using seaborn
    sns.violinplot(data=df, x='Group', y='Altitude error', hue="Voltage Compensation", split=True, ax=axes[1])
    # Remove x-ticks & x-label
    axes[1].grid(True)
    axes[1].set_xticks([])
    axes[1].set_xlabel('')

    # Adjust spacing between subplots
    plt.tight_layout()
    # Show the plot
    plt.show()


if __name__ == '__main__':
    # Create an argument parser
    parser = argparse.ArgumentParser(description='ROS Bag Data Extraction')

    # Add arguments for ROS Bag names with default values
    parser.add_argument('rosbag1', type=str, help='ROS Bag of the data without voltage compensation')
    parser.add_argument('rosbag2', type=str, help='ROS Bag of the data with voltage compensation')
    parser.add_argument('topic', type=str, help='ROS topic to plot in the comparison', default='/debug/pos_error', nargs='?')

    # Parse the command line arguments
    args = parser.parse_args()

    # Call the function to extract data and create the plots
    extract_data_from_rosbags(args.rosbag1, args.rosbag2, args.topic)
