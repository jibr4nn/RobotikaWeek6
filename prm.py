#!/usr/bin/env python3

import rospy
import random
import math
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# Fungsi untuk menghitung jarak Euclidean antara dua titik
def euclidean_distance(node1, node2):
    return math.sqrt((node1[0] - node2[0]) ** 2 + (node1[1] - node2[1]) ** 2)

# Fungsi untuk menghubungkan node yang berada dalam jarak tertentu
def connect_nodes(nodes, max_distance):
    edges = []
    for i, node1 in enumerate(nodes):
        for j, node2 in enumerate(nodes):
            if i != j and euclidean_distance(node1, node2) <= max_distance:
                edges.append((node1, node2))
    return edges

# Fungsi untuk menghasilkan node acak
def generate_random_nodes(num_nodes, x_limit, y_limit):
    nodes = []
    for _ in range(num_nodes):
        x = random.uniform(0, x_limit)
        y = random.uniform(0, y_limit)
        nodes.append((x, y))
    return nodes

# Fungsi untuk mempublikasikan node dan edges sebagai MarkerArray di Rviz
def visualize_prm(nodes, edges):
    marker_array = MarkerArray()

    # Marker untuk node
    node_marker = Marker()
    node_marker.header.frame_id = "map"
    node_marker.type = Marker.SPHERE_LIST
    node_marker.scale.x = 0.2
    node_marker.scale.y = 0.2
    node_marker.scale.z = 0.2
    node_marker.color.r = 0.0
    node_marker.color.g = 1.0
    node_marker.color.b = 0.0
    node_marker.color.a = 1.0
    node_marker.id = 0  # Set ID untuk node

    # Inisialisasi orientasi untuk node marker
    node_marker.pose.orientation.x = 0.0
    node_marker.pose.orientation.y = 0.0
    node_marker.pose.orientation.z = 0.0
    node_marker.pose.orientation.w = 1.0

    for node in nodes:
        pt = Point()
        pt.x = node[0]
        pt.y = node[1]
        pt.z = 0
        node_marker.points.append(pt)
    marker_array.markers.append(node_marker)

    # Marker untuk edges
    edge_marker = Marker()
    edge_marker.header.frame_id = "map"
    edge_marker.type = Marker.LINE_LIST
    edge_marker.scale.x = 0.05
    edge_marker.color.r = 1.0
    edge_marker.color.g = 0.0
    edge_marker.color.b = 0.0
    edge_marker.color.a = 1.0
    edge_marker.id = 1  # Set ID untuk edges

    # Inisialisasi orientasi untuk edge marker
    edge_marker.pose.orientation.x = 0.0
    edge_marker.pose.orientation.y = 0.0
    edge_marker.pose.orientation.z = 0.0
    edge_marker.pose.orientation.w = 1.0

    for edge in edges:
        pt1 = Point()
        pt1.x = edge[0][0]
        pt1.y = edge[0][1]
        pt1.z = 0
        pt2 = Point()
        pt2.x = edge[1][0]
        pt2.y = edge[1][1]
        pt2.z = 0
        edge_marker.points.append(pt1)
        edge_marker.points.append(pt2)

    marker_array.markers.append(edge_marker)

    # Marker untuk jalur (Path)
    path_marker = Marker()
    path_marker.header.frame_id = "map"
    path_marker.type = Marker.LINE_STRIP  # Gunakan LINE_STRIP untuk jalur
    path_marker.scale.x = 0.1  # Lebar jalur
    path_marker.color.r = 1.0
    path_marker.color.g = 1.0
    path_marker.color.b = 0.0
    path_marker.color.a = 1.0
    path_marker.id = 2  # Set ID untuk path

    # Inisialisasi orientasi untuk path marker
    path_marker.pose.orientation.x = 0.0
    path_marker.pose.orientation.y = 0.0
    path_marker.pose.orientation.z = 0.0
    path_marker.pose.orientation.w = 1.0  # Quaternion identitas

    # Menambahkan titik jalur dari edges yang terhubung
    for edge in edges:
        pt = Point()
        pt.x = edge[0][0]
        pt.y = edge[0][1]
        pt.z = 0
        path_marker.points.append(pt)

    marker_array.markers.append(path_marker)
    return marker_array

if __name__ == "__main__":
    rospy.init_node('prm_visualization')
    pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)

    # Set parameter untuk PRM
    num_nodes = 20
    x_limit = 10
    y_limit = 10
    max_distance = 3

    # Generate nodes dan edges
    nodes = generate_random_nodes(num_nodes, x_limit, y_limit)
    edges = connect_nodes(nodes, max_distance)

    # Publish marker array ke Rviz
    rate = rospy.Rate(1)  # Publikasikan pada 1 Hz
    while not rospy.is_shutdown():
        marker_array = visualize_prm(nodes, edges)
        pub.publish(marker_array)
        rate.sleep()
