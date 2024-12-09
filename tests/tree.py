import networkx as nx
import matplotlib.pyplot as plt

# Create an empty graph
G = nx.Graph()

# Read the edge list data from the file
with open("graph_data.txt", "r") as file:
    for line in file:
        node1, node2 = line.strip().split()
        G.add_edge(node1, node2)

# Define manual node positions
node_positions = {
    "node1": (0, 0),
    "node2": (1, 1),
    "node3": (2, 0),
    "node4": (2,-1),
    # Define positions for all nodes...
}

# Visualize the graph with manual positions
nx.draw(G, pos=node_positions, with_labels=True, node_size=100, node_color="lightblue", font_size=6, font_color="black")
plt.axis("off")
plt.show()
