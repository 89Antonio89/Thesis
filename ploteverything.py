# import xml.etree.ElementTree as ET

# co2_values = {
#     'file1': [],
#     'file2': []
# }


# def process_file(file_path, file_key):
#     tree = ET.parse(file_path)
#     root = tree.getroot()

#     # Remove namespaces
#     for elem in root.getiterator():
#         if not hasattr(elem.tag, 'find'):
#             continue
#         i = elem.tag.find('}')
#         if i >= 0:
#             elem.tag = elem.tag[i+1:]

#     # Iterate through the XML and aggregate the CO2 values
#     for tripinfo in root.findall('tripinfo'):
#         emissions = tripinfo.find('emissions')
#         if emissions is not None:
#             co2_value = emissions.get('CO2_abs')
#             if co2_value is not None:
#                 co2_values[file_key].append(float(co2_value))


# def mean(lst):
#     return sum(lst) / len(lst)


# # Process the files
# process_file('base/tripinfo.xml', 'file1')
# process_file('lighs/tripinfo.xml', 'file2')

# mean_file1 = mean(co2_values['file1'])
# mean_file2 = mean(co2_values['file2'])

# # Calculate the percentage reduction
# reduction_percentage = ((mean_file1 - mean_file2) / mean_file1) * 100

# print(
#     f"The overall reduction in CO2 emissions from file 1 to file 2 is: {reduction_percentage:.2f}%")


# import xml.etree.ElementTree as ET
# import matplotlib.pyplot as plt


# def count_vehicles(file_path):
#     """Parses an XML file and returns the count of vehicles."""
#     tree = ET.parse(file_path)
#     root = tree.getroot()

#     # Remove namespaces
#     for elem in root.getiterator():
#         if not hasattr(elem.tag, 'find'):
#             continue  # Skip if not an element (e.g. Comment)
#         i = elem.tag.find('}')
#         if i >= 0:
#             elem.tag = elem.tag[i+1:]

#     return len(root.findall('tripinfo'))


# # Provide the paths to your XML files
# file_path_1 = 'tripinfo.xml'
# file_path_2 = 'lasttripinfo.xml'

# vehicle_count_1 = count_vehicles(file_path_1)
# vehicle_count_2 = count_vehicles(file_path_2)

# # Plotting
# files = ['Before', 'After']
# counts = [vehicle_count_1, vehicle_count_2]
# print(vehicle_count_1, vehicle_count_2)
# plt.bar(files, counts, color=['blue', 'green'])
# plt.ylabel('Number of Vehicles')
# plt.title('Number of Vehicles that went past intersection')
# plt.show()


# def extract_route_durations(file_path):
#     tree = ET.parse(file_path)
#     root = tree.getroot()
#     durations = {
#         "f_1": [],
#         "f_2": [],
#         "f_3": [],
#         "f_4": [],
#         "f_5": []
#     }

#     for tripinfo in root.findall('tripinfo'):
#         vehicle_id = tripinfo.get('id')
#         route_prefix = vehicle_id.split('.')[0]
#         if route_prefix in durations:
#             durations[route_prefix].append(float(tripinfo.get('duration')))

#     total_durations = {key: sum(val) for key, val in durations.items()}
#     return total_durations


# xml_content = "lasttripinfo.xml"

# durations = extract_route_durations(xml_content)

# print("Total durations for each route:", durations)


# def extract_route_durations(xml_content):
#     root = ET.fromstring(xml_content)
#     durations = {
#         "f_1": [],
#         "f_2": [],
#         "f_3": [],
#         "f_4": [],
#     }

#     for tripinfo in root.findall('tripinfo'):
#         vehicle_id = tripinfo.get('id')
#         route_prefix = vehicle_id.split('.')[0]
#         if route_prefix in durations:
#             durations[route_prefix].append(float(tripinfo.get('duration')))

#     aggregated_durations = {key: sum(val) for key, val in durations.items()}
#     return aggregated_durations


# durations_file1 = extract_route_durations(xml_content1)
# durations_file2 = extract_route_durations(xml_content2)

# print("Durations from File 1:", durations_file1)
# print("Durations from File 2:", durations_file2)


# import xml.etree.ElementTree as ET
# import numpy as np
# import matplotlib.pyplot as plt


# def extract_route_data(file_path):
#     tree = ET.parse(file_path)
#     root = tree.getroot()
#     route_data = {
#         "f_1": [],
#         "f_2": [],
#         "f_3": [],
#         "f_4": [],
#         "f_5": [],
#     }

#     for tripinfo in root.findall('tripinfo'):
#         vehicle_id = tripinfo.get('id')
#         route_prefix = vehicle_id.split('.')[0]
#         if route_prefix in route_data:
#             route_data[route_prefix].append(
#                 float(tripinfo.get('duration')))

#     return route_data


# def mean(lst):
#     return sum(lst) / len(lst) if len(lst) != 0 else 0


# file_path_1 = 'base/tripinfo.xml'
# file_path_2 = 'lights/tripinfo.xml'

# route_data1 = extract_route_data(file_path_1)
# route_data2 = extract_route_data(file_path_2)

# mean_durations1 = {route: mean(durations)
#                    for route, durations in route_data1.items()}
# mean_durations2 = {route: mean(durations)
#                    for route, durations in route_data2.items()}

# print("Mean durations from first file:", mean_durations1)
# print("Mean durations from second file:", mean_durations2)


# def extract_durations_from_file(filename):
#     tree = ET.parse(filename)
#     root = tree.getroot()

#     # Create a dictionary to hold durations for each route
#     durations_dict = {'f_1': [], 'f_2': [], 'f_3': [], 'f_4': [], 'f_5': []}

#     for trip in root.findall('tripinfo'):
#         route_id = trip.get('id').split('.')[0]
#         print(route_id)

#         if route_id in durations_dict:
#             duration = float(trip.get('duration'))
#             durations_dict[route_id].append(duration)

#     return durations_dict


# filename_before = "base/tripinfo2.xml"  # Replace with your file path
# durations_before = extract_durations_from_file(filename_before)

# # Extract data from 'After' XML file
# filename_after = "lights/tripinfo2.xml"  # Replace with your file path
# durations_after = extract_durations_from_file(filename_after)


# fig, ax = plt.subplots(figsize=(10, 6))

# for route in durations_after:
#     y = durations_after[route]
#     x = [route] * len(y)   # Repeat the route name for all y points
#     ax.scatter(x, y, label=route, s=100, edgecolors='black', alpha=0.6)

# ax.set_ylabel('Duration (seconds)')
# ax.set_title('Durations per Route')
# ax.legend(loc='best')

# plt.tight_layout()
# plt.show()


# import xml.etree.ElementTree as ET
# import matplotlib.pyplot as plt


# def extract_data_from_file(filename):
#     tree = ET.parse(filename)
#     root = tree.getroot()

#     # Create a dictionary to hold various data for each route
#     data_dict = {'f_1': {'duration': [], 'emissions': []},
#                  'f_2': {'duration': [], 'emissions': []},
#                  'f_3': {'duration': [], 'emissions': []},
#                  'f_4': {'duration': [], 'emissions': []},
#                  'f_5': {'duration': [], 'emissions': []}}

#     # Extract data and populate the dictionary
#     for trip in root.findall('tripinfo'):
#         route_id = trip.get('id').split('.')[0]

#         if route_id in data_dict:
#             duration = float(trip.get('duration'))
#             emissions = float(trip.find('emissions').get(
#                 'CO2_abs'))  # for example
#             data_dict[route_id]['duration'].append(duration)
#             data_dict[route_id]['emissions'].append(emissions)

#     return data_dict


# def plot_data(data, metric):
#     fig, ax = plt.subplots(figsize=(10, 6))

#     for route in data:
#         y = data[route][metric]
#         x = [route] * len(y)
#         ax.scatter(x, y, label=route, s=100, edgecolors='black', alpha=0.6)

#     ax.set_ylabel(f'{metric} (units)')
#     ax.set_title(f'{metric} per Route')
#     ax.legend(loc='best')

#     plt.tight_layout()
#     plt.show()


# # List of files and their respective categories
# files = {
#     'DrivingBehavioursTest/conservative/tripinfo.xml': 'Conservative Driving',
#     'DrivingBehavioursTest/agressive/tripinfo.xml': 'Aggressive Driving',
#     'DrivingBehavioursTest/mixedDriving/tripinfo.xml': 'Mixed Driving Behavior',
#     # Add more files as needed...
# }

# # Process each file and plot
# for filename, category in files.items():
#     data = extract_data_from_file(filename)
#     for metric in ['duration', 'emissions']:
#         print(f"{category} - {metric}")
#         plot_data(data, metric)


# def extract_median_data_from_file(filename):
#     tree = ET.parse(filename)
#     root = tree.getroot()

#     durations = []
#     emissions = []

#     # Extract data
#     for trip in root.findall('tripinfo'):
#         duration = float(trip.get('duration'))
#         emission = float(trip.find('emissions').get('CO2_abs'))  # for example

#         durations.append(duration)
#         emissions.append(emission)
#     print(len(durations))
#     # Return median values
#     return np.median(durations), np.median(emissions)


# def plot_median_data(durations, emissions, labels):
#     width = 0.35  # width of the bars
#     ind = np.arange(len(labels))

#     fig, ax1 = plt.subplots(figsize=(10, 6))

#     color = 'tab:blue'
#     ax1.set_xlabel('Driving Behavior')
#     ax1.set_ylabel('Median Duration (seconds)', color=color)
#     ax1.bar(ind - width/2, durations, width, label='Duration', color=color)
#     ax1.tick_params(axis='y', labelcolor=color)

#     ax2 = ax1.twinx()  # instantiate a second axes that shares the same x-axis
#     color = 'tab:red'
#     ax2.set_ylabel('Emissions (mg)', color=color)
#     ax2.bar(ind + width/2, emissions, width, label='Emissions', color=color)
#     ax2.tick_params(axis='y', labelcolor=color)

#     ax1.set_xticks(ind)
#     ax1.set_xticklabels(labels)

#     fig.tight_layout()
#     plt.show()


# durations = []
# emissions = []
# labels = []
# files = {
#     'DrivingBehavioursTest/conservative/tripinfo.xml': 'Conservative Driving',
#     'DrivingBehavioursTest/agressive/tripinfo.xml': 'Aggressive Driving',
#     'DrivingBehavioursTest/mixedDriving/tripinfo.xml': 'Mixed Driving Behavior',
#     # Add more files as needed...
# }
# # Process each file
# for filename, label in files.items():
#     median_duration, median_emission = extract_median_data_from_file(filename)
#     durations.append(median_duration)
#     emissions.append(median_emission)
#     labels.append(label)

# # Plot the data
# plot_median_data(durations, emissions, labels)


# def extract_median_queue_time_from_file(filename):
#     tree = ET.parse(filename)
#     root = tree.getroot()

#     # Create a dictionary to hold queueing times for each lane
#     queue_times_dict = {}

#     for data in root.findall('data'):
#         for lane in data.findall('lanes/lane'):
#             lane_id = lane.get('id')
#             queue_time = float(lane.get('queueing_time'))

#             # If lane_id is already in the dictionary, append the queue_time
#             if lane_id in queue_times_dict:
#                 queue_times_dict[lane_id].append(queue_time)
#             # If lane_id is not in the dictionary, initialize a new list for it
#             else:
#                 queue_times_dict[lane_id] = [queue_time]

#     # Calculate the median queueing time for each lane
#     for lane_id in queue_times_dict:
#         queue_times_dict[lane_id] = np.median(queue_times_dict[lane_id])

#     return queue_times_dict


# from xml.etree import ElementTree
# import matplotlib.pyplot as plt
# import numpy as np
# # Extract data from XML files (assuming 3 different files for 3 driving behaviors)
# filename_conservative = "DrivingBehavioursTest/conservative/queue.xml"
# filename_aggressive = "DrivingBehavioursTest/agressive/queue.xml"
# filename_mixed = "DrivingBehavioursTest/mixedDriving/queue.xml"

# tree_conservative = ElementTree.parse(filename_conservative)
# tree_aggressive = ElementTree.parse(filename_aggressive)
# tree_mixed = ElementTree.parse(filename_mixed)


# def get_median_queue_times(tree):
#     queue_times = []

#     for data in tree.findall('data'):
#         for lane in data.findall('lanes/lane'):
#             queue_times.append(float(lane.get('queueing_time')))

#     return np.median(queue_times) if queue_times else 0


# median_time_conservative = get_median_queue_times(tree_conservative)
# median_time_aggressive = get_median_queue_times(tree_aggressive)
# median_time_mixed = get_median_queue_times(tree_mixed)

# # Plotting
# labels = ['Conservative', 'Aggressive', 'Mixed']
# values = [median_time_conservative, median_time_aggressive, median_time_mixed]

# plt.bar(labels, values, color=['blue', 'red', 'green'])
# plt.ylabel('Median Queueing Time')
# plt.title('Median Queueing Time by Driving Behavior')
# plt.show()


import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt


# def extract_metrics_from_file(filename):
#     tree = ET.parse(filename)
#     root = tree.getroot()

#     num_cars = len(root.findall('.//tripinfo'))
#     durations = [float(trip.get('duration'))
#                  for trip in root.findall('.//tripinfo')]
#     mean_duration = np.mean(durations) if durations else 0

#     co2_emissions = [float(trip.find('emissions').get('CO2_abs')) for trip in root.findall(
#         './/tripinfo') if trip.find('emissions') is not None and trip.find('emissions').get('CO2_abs') is not None]
#     total_co2 = sum(co2_emissions)

#     return num_cars, mean_duration, total_co2


# # List of your simulation files
# files = ['base/tripinfo.xml', 'base/tripinfo1.xml', 'base/tripinfo5.xml',
#          'base/tripinfo2.xml', 'base/tripinfo3.xml', 'base/tripinfo4.xml']
# num_cars_all = []
# mean_durations_all = []
# total_co2_all = []

# for file in files:
#     num_cars, mean_duration, total_co2 = extract_metrics_from_file(file)
#     num_cars_all.append(num_cars)
#     mean_durations_all.append(mean_duration)
#     total_co2_all.append(total_co2)

# # Plotting
# plt.figure(figsize=(15, 6))

# # Plotting the number of cars
# plt.subplot(1, 3, 1)
# plt.boxplot(num_cars_all, vert=False)
# plt.title("Number of Cars")
# plt.yticks([])

# # Plotting the durations
# plt.subplot(1, 3, 2)
# plt.boxplot(mean_durations_all, vert=False)
# plt.title("Mean Duration")
# plt.yticks([])

# # Plotting the CO2 emissions
# plt.subplot(1, 3, 3)
# plt.boxplot(total_co2_all, vert=False)
# plt.title("Total CO2 Emissions")
# plt.yticks([])

# plt.tight_layout()
# plt.show()


def plot_heatmap(vehicle_counts, traffic_light_coords):
    # Extract data for heatmap
    x = []
    y = []
    s = []

    for tl_id, count in vehicle_counts.items():
        coord = traffic_light_coords.get(tl_id)
        if coord:
            x.append(coord[0])
            y.append(coord[1])
            s.append(count)

    # Extract traffic light positions
    tl_x = [coord[0] for coord in traffic_light_coords.values()]
    tl_y = [coord[1] for coord in traffic_light_coords.values()]

    # Create the heatmap
    plt.figure(figsize=(10, 10))
    plt.scatter(x, y, c=s, s=np.array(s)*10, cmap='YlOrRd',
                alpha=0.6, edgecolors="grey", linewidth=0.5)

    # Mark traffic light locations with large symbols (e.g., blue diamonds)
    plt.scatter(tl_x, tl_y, c='blue', s=100,
                marker='D', label='Traffic Lights')

    plt.colorbar(label="Vehicle Count")
    plt.xlabel('X Coordinate')
    plt.ylabel('Y Coordinate')
    plt.title('Vehicle Counts near Traffic Lights')
    plt.legend(loc="upper right")

    plt.show()


coordinates = {9808: (23.32722282409668, -154.39999389648438), 9809: (43.89999771118164, -158.1999969482422), 9810: (42.54999923706055, 100.19999694824219), 9811: (41.39999771118164, -78.0999984741211), 9812: (41.849998474121094, -100.19999694824219), 9813: (20.25, -99.9000015258789), 9814: (22.75, -215.14999389648438), 9815: (21.25, -78.8499984741211), 9816: (-39.599998474121094, -78.0999984741211), 9817: (-38.849998474121094, -101.29999542236328), 9818: (-62.79999923706055, -101.79999542236328), 9819: (-61.14999771118164, -78.0999984741211), 9820: (-113.14999389648438, -78.94999694824219), 9821: (-112.79999542236328, -101.29999542236328), 9822: (-139.34999084472656, -100.8499984741211), 9823: (-140.39999389648438, -78.0999984741211), 9824: (-113.14999389648438, 11.59999942779541), 9825: (-113.89999389648438, -9.649999618530273), 9826: (-139.34999084472656, -10.300000190734863), 9827: (-140.39999389648438, 12.449999809265137), 9828: (-38.54999923706055, 11.59999942779541), 9829: (-38.599998474121094, -9.649999618530273), 9830: (-60.69999694824219, -9.5), 9831: (-59.599998474121094, 12.449999809265137), 9832: (40.25, 78.5999984741211), 9833: (-38.64999771118164, 100.04999542236328),
               9834: (-37.45000076293945, 78.0), 9835: (-61.25, 76.69999694824219), 9836: (-61.349998474121094, 101.25), 9837: (-112.29999542236328, 101.0999984741211), 9838: (-113.39999389648438, 78.0), 9839: (-139.8000030517578, 77.8499984741211), 9840: (-138.8000030517578, 100.04999542236328), 9841: (-179.34999084472656, 12.449999809265137), 9842: (-179.25, -9.699999809265137), 9843: (-143.03887939453125, -150.01956176757812), 9844: (18.600000381469727, 79.75), 9845: (-201.39999389648438, -10.300000190734863), 9846: (-201.5500030517578, 12.449999809265137), 9847: (-112.29999542236328, 159.39999389648438), 9848: (-138.5, 137.5500030517578), 9849: (-138.8000030517578, 159.5500030517578), 9850: (-112.96646881103516, -128.8490447998047), 9851: (-112.76590728759766, -150.9499969482422), 9852: (40.849998474121094, 216.34999084472656), 9853: (19.149999618530273, 101.69999694824219), 9854: (17.25, 177.39999389648438), 9855: (15.649999618530273, 197.9499969482422), 9856: (46.349998474121094, -178.5), 9857: (47.95000076293945, -197.34999084472656), 9858: (40.89999771118164, 10.5), 9859: (40.25, -11.09999942779541), 9860: (18.600000381469727, -9.949999809265137), 9861: (19.149999618530273, 12.0)}

vehicle_counts = {9819: 56, 9808: 260, 9810: 213, 9818: 198, 9835: 245, 9845: 180, 9851: 256, 9850: 260, 9849: 113, 9816: 80,
                  9843: 30, 9846: 97, 9829: 43, 9817: 37, 9827: 57, 9834: 24, 9828: 22, 9833: 26, 9830: 14, 9841: 32, 9842: 8, 9836: 5}
plot_heatmap(vehicle_counts, coordinates)
