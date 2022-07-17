from tkinter import *
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.lines import Line2D
from sklearn.decomposition import PCA
import json
import math

class draw_result():

    def draw_trucks(self,nodes,costs,K,truck,P, locations,pallet, pallet_weight,coordinate_of_location,edge_list, objective):
        self.NODES=nodes
        self.costs=costs
        self.K=K
        self.truck=truck
        self.P=P
        self.pallet=pallet
        self.pallet_weight=pallet_weight
        self.coordinate_of_location=coordinate_of_location

        print(K)
        print(truck)

        f = open("Colors.json")
        colors = json.loads(f.read())

        self.pallet_colors = []
        for i in colors["palletColors"]:
            self.pallet_colors.append(i)

        self.route_color = []
        for i in colors["routeColors"]:
            self.route_color.append(i)

        pallet_size_width = 75
        pallet_size_length = 75

        top = Tk()
        height = 1200
        width = 2000

        top_frame = Frame(top, width=width, height=height)
        top_frame.pack(expand=True, fill=BOTH)

        frame_left = Frame(top_frame, width=width * 2 / 5, height=height, bg="white")
        frame_left.pack(side=LEFT, fill="y", expand=True)

        frame_right = Frame(top_frame, width=width * 3 / 5, height=height, bg="white")
        frame_right.pack(side=LEFT, fill=BOTH, expand=True)

        c = Canvas(frame_left, bg='#FFFFFF', width=width * 2 / 5, height=height,
                   scrollregion=(0, 0, width * 2 / 5, 200 + self.K * (pallet_size_length * 2 + 100)))

        vbar = Scrollbar(frame_left, orient=VERTICAL)
        vbar.pack(side=RIGHT, fill="y")
        vbar.config(command=c.yview)

        c.config(yscrollcommand=vbar.set)
        c.pack(fill=BOTH, expand=True)

        top.geometry(str(width) + "x" + str(height))

        figure1 = plt.Figure(figsize=(100, 100), dpi=100)
        plot = figure1.add_subplot()
        G = nx.MultiDiGraph()

        for i in range(self.NODES):
            G.add_node(i)

        trucks = []
        for i in range(self.NODES):
            for j in range(self.NODES):
                count = 0
                for el in edge_list:
                    if str(i) == el[1] and str(j) == el[2]:
                        count += 1
                        G.add_edge(i, j, color=self.route_color[int(el[0])], count=count)
                        trucks.append(int(el[0]))

        trucks = list(set(trucks))
        X = self.costs

        pca = PCA(n_components=2)
        X2d = pca.fit_transform(X)
        pos = X2d

        M = G.number_of_edges()
        N = G.number_of_nodes()

        labels = {}
        for i in range(self.NODES):
            labels[i] = i

        c.create_text(
            (width * 2 / 5) / 2,
            50, text="COSTO : " + str(objective), font=('Helvetica', '10'))

        nodes = nx.draw_networkx_nodes(G, pos, node_color="white", edgecolors="black", ax=plot)
        labels = nx.draw_networkx_labels(G, pos, labels=labels, ax=plot)
        for edge in G.edges(data=True):
            print(edge)
            nx.draw_networkx_edges(G,
                                   pos,
                                   arrowstyle="->",
                                   arrowsize=10,
                                   width=2,
                                   ax=plot,
                                   edgelist=[(edge[0], edge[1])],
                                   edge_color=f'{edge[2]["color"]}',
                                   connectionstyle=f'arc3, rad = {0.02 * edge[2]["count"]}')

        legend_elements = []
        for i in trucks:
            legend_elements.append(Line2D([0], [0], color=self.route_color[i], lw=4, label='truck' + str(i)))

        plot.legend(handles=legend_elements, loc='best')

        graph = FigureCanvasTkAgg(figure1, frame_right)
        graph.get_tk_widget().pack(side=LEFT, fill=BOTH)

        x_left = 100

        for k in range(self.K):
            total_weigth = 0
            truck_size_length = self.P[k] / 2 * pallet_size_length
            truck_size_width = pallet_size_width * 2

            c.create_rectangle(x_left,
                               100 + (k * (truck_size_width + 100)),
                               x_left + truck_size_length,
                               100 + (k * (truck_size_width + 100)) + truck_size_width)

            gravity_center_numerator_x = self.truck[k]["weightEmptyTruck"] * (self.P[k] / 2 * self.pallet[0]["width"]) / 2
            gravity_center_denominator_x = self.truck[k]["weightEmptyTruck"]

            gravity_center_numerator_y = self.truck[k]["weightEmptyTruck"] * (2 * self.pallet[0]["length"]) / 2
            gravity_center_denominator_y = self.truck[k]["weightEmptyTruck"]

            weight = {}
            weight["axle1"] = 0
            weight["axle2"] = 0

            for p in range(self.P[k]):
                for loc in locations:
                    if loc[0] == str(k) and loc[3] == str(p):
                        c.create_rectangle(x_left + (p % (self.P[k] / 2)) * pallet_size_width,
                                           100 + (k * (truck_size_width + 100)) + pallet_size_length * math.floor(
                                               p / (self.P[k] / 2)),
                                           x_left + ((p % (self.P[k] / 2)) + 1) * pallet_size_width,
                                           100 + (k * (truck_size_width + 100)) + pallet_size_length * (
                                               math.floor(p / (self.P[k] / 2) + 1)),
                                           fill=self.pallet_colors[int(loc[1])])

                        total_weigth += self.pallet_weight[int(loc[1])][int(loc[2])]
                        weight["axle1"] += self.pallet_weight[int(loc[1])][int(loc[2])] * (
                                    self.truck[k]["distanceAxle2"] - self.coordinate_of_location[k][p][0]) / (
                                                       self.truck[k]["distanceAxle2"] - self.truck[k]["distanceAxle1"])
                        weight["axle2"] += self.pallet_weight[int(loc[1])][int(loc[2])] * (
                                    self.coordinate_of_location[k][p][0] - self.truck[k]["distanceAxle1"]) / (
                                                       self.truck[k]["distanceAxle2"] - self.truck[k]["distanceAxle1"])

                        text = "POS=" + str(p) + "\n" + "Cust=" + loc[1] + "\n" + "T=" + loc[2] + "\n" + "W=" + str(
                            self.pallet_weight[int(loc[1])][int(loc[2])])

                        c.create_text(
                            x_left + (p % (self.P[k] / 2)) * pallet_size_width + pallet_size_width / 2,
                            100 + (k * (truck_size_width + 100)) + pallet_size_length * math.floor(
                                p / (self.P[k] / 2)) + pallet_size_length / 2,
                            text=text, font=('Helvetica', '8'))

                        gravity_center_numerator_x += self.coordinate_of_location[k][p][0] * \
                                                      self.pallet_weight[int(loc[1])][int(loc[2])]
                        gravity_center_denominator_x += self.pallet_weight[int(loc[1])][int(loc[2])]

                        gravity_center_numerator_y += self.coordinate_of_location[k][p][1] * \
                                                      self.pallet_weight[int(loc[1])][
                                                          int(loc[2])]
                        gravity_center_denominator_y += self.pallet_weight[int(loc[1])][int(loc[2])]

            gravity_center_x = (gravity_center_numerator_x / gravity_center_denominator_x) * truck_size_length / (
                        self.P[k] / 2 * self.pallet[0]["width"])
            gravity_center_y = (gravity_center_numerator_y / gravity_center_denominator_y) * truck_size_width / (
                        2 * self.pallet[0]["length"])

            c.create_line(x_left + truck_size_length - gravity_center_x - 10,
                          100 + (k * (truck_size_width + 100)) + gravity_center_y - 10,
                          x_left + truck_size_length - gravity_center_x + 10,
                          100 + (k * (truck_size_width + 100)) + gravity_center_y + 10, fill="red")

            c.create_line(x_left + truck_size_length - gravity_center_x + 10,
                          100 + (k * (truck_size_width + 100)) + gravity_center_y - 10,
                          x_left + truck_size_length - gravity_center_x - 10,
                          100 + (k * (truck_size_width + 100)) + gravity_center_y + 10, fill="red")

            c.create_text(x_left,
                          75 + (k * (truck_size_width + 100)), anchor="w",
                          text="TRUCK " + str(k) + ": peso carico->" + str(total_weigth) + ",  peso asse1->" + str(
                              weight["axle1"]) + ",  peso asse2->" + str(weight["axle2"]))

            c.create_rectangle(
                x_left + truck_size_length / 2 + self.truck[k]["deviation1x"] * truck_size_length / self.truck[k]["length"],
                100 + (k * (truck_size_width + 100)) + truck_size_width / 2 - self.truck[k][
                    "deviation1y"] * truck_size_width / self.truck[k]["width"],
                x_left + truck_size_length / 2 - self.truck[k]["deviation2x"] * truck_size_length / self.truck[k]["length"],
                100 + (k * (truck_size_width + 100)) + truck_size_width / 2 + self.truck[k][
                    "deviation2y"] * truck_size_width / self.truck[k]["width"])
        top.mainloop()