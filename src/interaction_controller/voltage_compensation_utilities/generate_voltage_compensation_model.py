import sys
import rosbag
import rospy
import matplotlib.pyplot as plt
import argparse

def process_rosbag(rosbag_file, window_size=125, voltage_topic="/mavros/battery", thrust_topic="/mavros/setpoint_raw/attitude", max_voltage=25.2):
    # Open the bag file
    bag = rosbag.Bag(rosbag_file)

    # Variables to store the processed data
    voltage_timestamps = []
    voltage = []
    thrust_timestamps = []
    thrust = []

    # Find the start time
    start_time = None
    for topic, msg, t in bag.read_messages(topics=[voltage_topic]):
        if start_time is None:
            start_time = t
        voltage_timestamps.append((t - start_time).to_sec())
        voltage.append(msg.voltage)
    # find the maximum voltage
    voltage_normalized = [x / max_voltage for x in voltage]

    for topic, msg, t in bag.read_messages(topics=[thrust_topic]):
        if start_time is None:
            start_time = t
        thrust_timestamps.append((t - start_time).to_sec())
        thrust.append(msg.thrust)

    # Close the bag file
    bag.close()

    # Plot the voltage data, original thrust data, and filtered thrust data
    fig, ax = plt.subplots()

    # Plot voltage data
    ax.plot(voltage_timestamps, voltage_normalized, label="Voltage")

    # Plot original thrust data
    ax.plot(thrust_timestamps, thrust, label="Thrust")

    # Plot filtered thrust data
    filtered_thrust = moving_average_filter(thrust, window_size)
    ax.plot(thrust_timestamps, filtered_thrust, label="Filtered Thrust")

    # Set labels and title
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.set_title("Voltage and Thrust Data")
    # Add legend
    ax.legend()
    # Add grid
    ax.grid(True)
    # Display the plot
    plt.show()

    return voltage_timestamps, voltage, voltage_normalized, thrust_timestamps, thrust, filtered_thrust


def moving_average_filter(data, window_size):
    filtered_data = []
    for i in range(len(data)):
        if i < window_size:
            filtered_data.append(sum(data[:i + 1]) / (i + 1))
        else:
            filtered_data.append(sum(data[i - window_size + 1:i + 1]) / window_size)
    return filtered_data


def calculate_voltage_compensation_model_parameters(voltage_normalized, filtered_thrust, voltage_timestamps, thrust_timestamps):
    # Extract the first and last data points
    first_v = voltage_normalized[0]
    final_v = voltage_normalized[-1]
    first_thrust = filtered_thrust[0]
    final_thrust = filtered_thrust[-1]
    # Calculate model
    model_slope = (final_thrust - first_thrust) / (final_v - first_v)
    model_constant = first_v
    # evaluate model
    model_voltage_compensation_thrust = [model_slope * (x - model_constant) for x in voltage_normalized]
    model_thrust = [first_thrust + x for x in model_voltage_compensation_thrust]
    # Plot the model
    # Plot the voltage data, original thrust data, and filtered thrust data
    fig, ax = plt.subplots()

    # Plot voltage data
    ax.plot(voltage_timestamps, voltage_normalized, label="Voltage (measured)")

    # Plot filtered thrust data
    ax.plot(thrust_timestamps, filtered_thrust, label="Filtered Thrust (measured)")

    # Plot model thrust data
    ax.plot(voltage_timestamps, model_thrust, label="Filtered Thrust (model)")

    # Set labels and title
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Value")
    ax.set_title("Voltage and Thrust Data")
    # Add legend
    ax.legend()
    # Add grid
    ax.grid(True)
    # Display the plot
    plt.show()

    return model_slope, model_constant


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Process rosbag data.')
    parser.add_argument('rosbag_file', type=str, help='Path to the rosbag file')
    parser.add_argument('--window-size', type=int, default=125, help='Size of the moving average window (default: 125)', nargs='?')
    parser.add_argument('--voltage-topic', type=str, default="/mavros/battery", help='Voltage topic (default: /mavros/battery)', nargs='?')
    parser.add_argument('--thrust-topic', type=str, default="/mavros/setpoint_raw/attitude", help='Thrust topic (default: /mavros/setpoint_raw/attitude)', nargs='?')
    parser.add_argument('--max-voltage', type=float, default=25.2, help='Maximum voltage (default: 25.2)', nargs='?')
    args = parser.parse_args()

    rosbag_file = args.rosbag_file
    window_size = args.window_size
    voltage_topic = args.voltage_topic
    thrust_topic = args.thrust_topic
    max_voltage = args.max_voltage

    voltage_timestamps, voltage, voltage_normalized, thrust_timestamps, thrust, filtered_thrust = process_rosbag(
        rosbag_file,
        window_size=window_size,
        voltage_topic=voltage_topic,
        thrust_topic=thrust_topic,
        max_voltage=max_voltage
    )

    model_slope, model_constant = calculate_voltage_compensation_model_parameters(
        voltage_normalized,
        filtered_thrust,
        voltage_timestamps,
        thrust_timestamps
    )
    print("Model slope: {}".format(model_slope))
    print("Model_constant: {}".format(model_constant))
    print("Window_size: {}".format(args.window_size))
