import gurobipy as gb
import numpy as np
import math
import os
import csv
import re
from Draw_Solution import draw_result

class solve_pallet_loading_vehicle_routing:

    def __init__(self,path):
        self.truck = []
        self.pallet = {}
        name=os.path.basename(path)

        m = re.search('Inst(.+?)P', name)
        if m:
            self.NODES = int(m.group(1))+1

        actual_row = 0
        with open(path, 'r') as f:  # Read lines separately
            reader = csv.reader(f, delimiter=' ')
            rows =  list(reader)
            self.K=int(rows[actual_row][1])
            actual_row+=1
            for i in range(actual_row,actual_row+self.K):
                truck = {}
                truck["length"] = int(rows[i][1])
                truck["width"] = int(rows[i][2])
                truck["height"] = int(rows[i][3])
                truck["maxCargoWeigth"] = int(rows[i][4])
                truck["maxWeigthAxle1"] = int(rows[i][5])
                truck["distanceAxle1"] = int(rows[i][6])
                truck["maxWeigthAxle2"] = int(rows[i][7])
                truck["distanceAxle2"] = int(rows[i][8])
                truck["weightEmptyTruck"] = 4000
                truck["geometricCenterX"] = truck["length"]/2
                truck["geometricCenterY"] = truck["width"]/2

                truck["deviation1x"] = truck["distanceAxle2"]-truck["length"]/2
                truck["deviation2x"] = truck["length"]/2
                truck["deviation1y"] = truck["width"]/8
                truck["deviation2y"] = truck["width"]/8
                self.truck.append(truck)
                actual_row += 1


            types_of_pallets = int(rows[actual_row][1])
            actual_row += 1

            self.pallet=[]
            for _ in range(types_of_pallets):
                pallet={}
                pallet["length"] = int(rows[actual_row][1])
                pallet["width"] = int(rows[actual_row][2])
                pallet["height"] = int(rows[actual_row][3])
                self.pallet.append(pallet)
                actual_row+=1



            self.P = []

            for i in range(self.K):
                self.P.append(math.floor(self.truck[i]["length"] / self.pallet[0]["width"]) * math.floor(
                self.truck[i]["width"] / self.pallet[0]["length"]))

            requests = int(rows[actual_row][1])
            self.pallet_request = np.zeros(self.NODES, dtype=np.uint32)
            self.pallet_weight = [[] for _ in range(self.NODES)]
            self.pallet_height = [[] for _ in range(self.NODES)]

            actual_row += 1
            for _ in range(requests):
                self.pallet_request[int(rows[actual_row][1])] += 1
                self.pallet_weight[int(rows[actual_row][1])].append(int(rows[actual_row][2]))
                self.pallet_height[int(rows[actual_row][1])].append(self.pallet[int(rows[actual_row][3])-1]["height"])
                actual_row += 1

        self.costs = np.zeros((self.NODES, self.NODES), dtype=np.uint32)

        actual_row+=1

        for i in range(self.NODES):
            parsed_row=rows[actual_row+i][0].split("\t")
            for j in range (self.NODES):
                self.costs[i,j]=int(parsed_row[j])


    def solve_problem(self,show_result=1):
        self.coordinate_of_location=[]
        for k in range(self.K):
            self.coordinate_of_location.append(np.zeros((self.P[k], 2), dtype=np.float64))

            for p in range(self.P[k]):
                distance_x = ((p % (self.P[k] / 2)) + 0.5) * self.pallet[0]["width"]
                distance_y = (math.floor(p / (self.P[k] / 2)) + 0.5) * (self.pallet[0]["length"])
                self.coordinate_of_location[k][p][0] = distance_x
                self.coordinate_of_location[k][p][1] = distance_y


        problem = gb.Model("pallet-loading/routing")
        problem.setParam('LogToConsole', 0)
        problem.modelSense = gb.GRB.MINIMIZE
        problem.setParam('TimeLimit', 20 * 60)


        Y = problem.addVars([(k) for k in range(self.K)], vtype=gb.GRB.BINARY, name="Y")
        X = problem.addVars([(k, i, j) for j in range(self.NODES) for i in range(self.NODES) for k in range(self.K)], vtype=gb.GRB.BINARY,
                            name="X")

        Z = problem.addVars(
            [(k, i, t, p) for i in range(self.NODES) for k in range(self.K) for p in range(self.P[k]) for t in range(self.pallet_request[i]) ],
            vtype=gb.GRB.BINARY,
            name="Z")

        F = problem.addVars([(k, i, j) for j in range(self.NODES) for i in range(self.NODES) for k in range(self.K)],
                            vtype=gb.GRB.INTEGER,
                            name="F")

        L = problem.addVars([(k, p) for k in range(self.K) for p in range(self.P[k])], vtype=gb.GRB.BINARY, name="L")
        R = problem.addVars([(k, p) for k in range(self.K) for p in range(self.P[k])], vtype=gb.GRB.BINARY, name="R")
        S = problem.addVars([(k, c) for k in range(self.K) for c in range(int(self.P[k] / 2))], vtype=gb.GRB.BINARY, name="S")



        # OBJECTIVE FUNCTION
        #1
        problem.setObjective(
            gb.quicksum(self.costs[i, j] * X[k, i, j] for i in range(self.NODES) for j in range(self.NODES) if j != i for k in range(self.K))
        )

        # block same point travel
        for k in range(self.K):
            for i in range(self.NODES):
                problem.addConstr(X[k, i, i] == 0)

        # each truck start from depot:
        #2
        for k in range(self.K):
            problem.addConstr(gb.quicksum(X[k, 0, i] for i in range(1, self.NODES)) == Y[k])

        # if a truck arrives at node, it has to leave it:
        #3
        for k in range(self.K):
            for i in range(1, self.NODES):
                problem.addConstr(
                    gb.quicksum(X[k, j, i] for j in range(self.NODES) if j != i) ==
                    gb.quicksum(X[k, i, j] for j in range(self.NODES) if j != i))

        # if a truck k travel from i to j, then it is used in the solution
        #4
        for k in range(self.K):
            for j in range(self.NODES):
                for i in range(self.NODES):
                    problem.addConstr(Y[k] >= X[k, i, j])

        # each costumer is visited at most once by each route
        #5
        for k in range(self.K):
            for i in range(1, self.NODES):
                problem.addConstr(
                    gb.quicksum(X[k, i, j] for j in range(self.NODES) if j != i) <= 1)

        # flow from costumer to depot must be zero
        #6
        for k in range(self.K):
            for i in range(1, self.NODES):
                problem.addConstr(F[k, i, 0] == 0)

        # max flow has to be less than Costumers
        #7
        for k in range(self.K):
            for i in range(1, self.NODES):
                problem.addConstr(F[k, 0, i] <= (self.NODES - 1))

        # flow arriving at node i should be equal to the flow leaving node i +1 if truck visit node i
        #8
        for k in range(self.K):
            for i in range(1, self.NODES):
                problem.addConstr(gb.quicksum(F[k, j, i] for j in range(self.NODES) if j != i) ==
                                  gb.quicksum(F[k, i, j] for j in range(self.NODES) if j != i) +
                                  gb.quicksum(X[k, i, j] for j in range(self.NODES) if j != i))

        # No flow through an arch if the route does not use the arc
        #9
        for k in range(self.K):
            for i in range(self.NODES):
                for j in range(self.NODES):
                    if i != j:
                        problem.addConstr((self.NODES - 1) * X[k, i, j] >= F[k, i, j])

        # for each position in a truck at most one pallet
        #10
        for k in range(self.K):
            for p in range(self.P[k]):
                problem.addConstr(
                    gb.quicksum(Z[k, i, t, p] for i in range(1, self.NODES) for t in range(self.pallet_request[i])) <= 1)

        # each pallet only once
        #11
        for i in range(1, self.NODES):
            for t in range(self.pallet_request[i]):
                problem.addConstr(gb.quicksum(Z[k, i, t, p] for k in range(self.K) for p in range(self.P[k])) == 1)

        # weigth of each truck cannot exceed max weigth
        #12
        for k in range(self.K):
            problem.addConstr(gb.quicksum(
                self.pallet_weight[i][t] * Z[k, i, t, p] for i in range(1, self.NODES) for t in range(self.pallet_request[i]) for p in
                range(self.P[k])) <= self.truck[k]["maxCargoWeigth"])

        # heigth of pallets in a truck cannot exceed heigth
        #12b
        for k in range(self.K) :
            for p in range(self.P[k]):
                for i in range(1,self.NODES):
                    for t in range(self.pallet_request[i]):
                        problem.addConstr( self.pallet_height[i][t] * Z[k, i, t, p]  <= self.truck[k]["height"])

        # the sum of force exerted by pallet should be lower or equal to max force of each axle
        #13
        for k in range(self.K):
            problem.addConstr(gb.quicksum(
                (self.pallet_weight[i][t] * Z[k, i, t, p]) * (self.truck[k]["distanceAxle2"] - self.coordinate_of_location[k][p][0]) for i in
                range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) <= self.truck[k]["maxWeigthAxle1"] * (
                                      self.truck[k]["distanceAxle2"] - self.truck[k]["distanceAxle1"]))

        #14
        for k in range(self.K):
            problem.addConstr(gb.quicksum(
                (self.pallet_weight[i][t] * Z[k, i, t, p]) * (self.coordinate_of_location[k][p][0] - self.truck[k]["distanceAxle1"]) for i in
                range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) <= self.truck[k]["maxWeigthAxle2"] * (
                                      self.truck[k]["distanceAxle2"] - self.truck[k]["distanceAxle1"]))

        # constraint on gravityCenter
        #15
        for k in range(self.K):
            problem.addConstr(self.truck[k]["weightEmptyTruck"] * self.truck[k]["geometricCenterX"] +
                              gb.quicksum(
                                  self.coordinate_of_location[k][p][0] * self.pallet_weight[i][t] * Z[k, i, t, p] for i in
                                  range(1, self.NODES)
                                  for t in range(self.pallet_request[i]) for p in range(self.P[k])) <=
                              (gb.quicksum(
                                  self.pallet_weight[i][t] * Z[k, i, t, p] for i in range(1, self.NODES) for t
                                  in range(self.pallet_request[i]) for p in range(self.P[k])) + self.truck[k]["weightEmptyTruck"]) *
                              (self.truck[k]["geometricCenterX"] + self.truck[k]["deviation1x"]))
        #16
        for k in range(self.K):
            problem.addConstr(self.truck[k]["weightEmptyTruck"] * self.truck[k]["geometricCenterX"] +
                              gb.quicksum(self.coordinate_of_location[k][p][0] * self.pallet_weight[i][t] * Z[k, i, t, p] for i in
                                          range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) >=
                              (gb.quicksum(
                                  self.pallet_weight[i][t] * Z[k, i, t, p] for i in range(1, self.NODES)
                                  for t in range(self.pallet_request[i]) for p in range(self.P[k])) + self.truck[k]["weightEmptyTruck"]) *
                              (self.truck[k]["geometricCenterX"] - self.truck[k]["deviation2x"]))

        #17
        for k in range(self.K):
            problem.addConstr(self.truck[k]["weightEmptyTruck"] * self.truck[k]["geometricCenterY"] +
                              gb.quicksum(self.coordinate_of_location[k][p][1] * self.pallet_weight[i][t] * Z[k, i, t, p] for i in
                                          range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) <=
                              (gb.quicksum(self.pallet_weight[i][t] * Z[k, i, t, p] for i in
                                           range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) + self.truck[k][
                                   "weightEmptyTruck"]) *
                              (self.truck[k]["geometricCenterY"] + self.truck[k]["deviation1y"]))

        #18
        for k in range(self.K):
            problem.addConstr(self.truck[k]["weightEmptyTruck"] * self.truck[k]["geometricCenterY"] +
                              gb.quicksum(self.coordinate_of_location[k][p][1] * self.pallet_weight[i][t] * Z[k, i, t, p] for i in
                                          range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) >=
                              (gb.quicksum(self.pallet_weight[i][t] * Z[k, i, t, p] for i in
                                           range(1, self.NODES) for t in range(self.pallet_request[i]) for p in range(self.P[k])) + self.truck[k][
                                   "weightEmptyTruck"]) *
                              (self.truck[k]["geometricCenterY"] - self.truck[k]["deviation2y"]))

        # dinamically stable cargo
        # 19
        for k in range(self.K):
            for p in range(1, self.P[k]):
                if p != self.P[k] / 2:
                    problem.addConstr(
                        L[k, p] >= (gb.quicksum(Z[k, i, t, p] for i in range(1, self.NODES) for t in range(self.pallet_request[i])) -
                                    gb.quicksum(
                                        Z[k, i, t, p - 1] for i in range(1, self.NODES) for t in range(self.pallet_request[i]))))
        # 20
        for k in range(self.K):
            for p in [0, int(self.P[k] / 2)]:
                problem.addConstr(
                    L[k, p] >= gb.quicksum(Z[k, i, t, p] for i in range(1, self.NODES) for t in range(self.pallet_request[i])))
        # 21
        for k in range(self.K):
            for p in range(self.P[k] - 1):
                if p != int(self.P[k] / 2) - 1:
                    problem.addConstr(
                        R[k, p] >= (gb.quicksum(Z[k, i, t, p] for i in range(1, self.NODES) for t in range(self.pallet_request[i])) -
                                    gb.quicksum(
                                        Z[k, i, t, p + 1] for i in range(1, self.NODES) for t in range(self.pallet_request[i]))))

        # 22
        for k in range(self.K):
            for p in [self.P[k] / 2 - 1, self.P[k] - 1]:
                problem.addConstr(
                    R[k, p] >= gb.quicksum(Z[k, i, t, p] for i in range(1, self.NODES) for t in range(self.pallet_request[i])))
        # 23
        for k in range(self.K):
            for c in range(int(self.P[k] / 2)):
                problem.addConstr(
                    S[k, c] >= (gb.quicksum(
                        Z[k, i, t, c] for i in range(1, self.NODES) for t in range(self.pallet_request[i])) -
                                gb.quicksum(
                                    Z[k, i, t, c + self.P[k] / 2] for i in range(1, self.NODES) for t in range(self.pallet_request[i]))))
        # 24
        for k in range(self.K):
            for c in range(int(self.P[k] / 2)):
                problem.addConstr(
                    S[k, c] >= (gb.quicksum(
                        Z[k, i, t, c + (self.P[k] / 2)] for i in range(1, self.NODES) for t in range(self.pallet_request[i])) -
                                gb.quicksum(
                                    Z[k, i, t, c] for i in range(1, self.NODES) for t in range(self.pallet_request[i]))))
        # 25
        for k in range(self.K):
            problem.addConstr(
                gb.quicksum(L[k, p] for p in range(int(self.P[k] / 2))) <= 1)
        # 26
        for k in range(self.K):
            problem.addConstr(
                gb.quicksum(L[k, p] for p in range(int(self.P[k] / 2), self.P[k])) <= 1)
        # 27
        for k in range(self.K):
            problem.addConstr(
                gb.quicksum(R[k, p] for p in range(int(self.P[k] / 2))) <= 1)
        # 28
        for k in range(self.K):
            problem.addConstr(
                gb.quicksum(R[k, p] for p in range(int(self.P[k] / 2), self.P[k])) <= 1)
        # 29
        for k in range(self.K):
            problem.addConstr(
                gb.quicksum(S[k, c] for c in range(int(self.P[k] / 2))) <= 1)

        # if truck travel from i to j has to delever at least one j's pallet
        #33
        for k in range(self.K):
            for j in range(1, self.NODES):
                problem.addConstr(gb.quicksum(X[k, i, j] for i in range(self.NODES)) <=
                                  gb.quicksum(Z[k, j, t, p] for t in range(self.pallet_request[j]) for p in range(self.P[k])))

        # if on truck is loaded j's pallet the rout of truck must visit j
        #34
        for k in range(self.K):
            for j in range(1, self.NODES):
                problem.addConstr(gb.quicksum(Z[k, j, t, p] for t in range(self.pallet_request[j]) for p in range(self.P[k])) <=
                                  gb.quicksum(X[k, j, i] * self.pallet_request[j] for i in range(self.NODES)))

        # sequential loading constraint
        #35
        for k in range(self.K):
            for p1 in range(self.P[k]):
                for p2 in range(self.P[k]):
                    if (p2 % (self.P[k] / 2)) > (p1 % (self.P[k] / 2)):
                        for i in range(1, self.NODES):
                            for j in range(1, self.NODES):
                                problem.addConstr(gb.quicksum(Z[k, i, t, p1] for t in range(self.pallet_request[i])) +
                                                  gb.quicksum(Z[k, j, t, p2] for t in range(self.pallet_request[j])) <=
                                                  2 - X[k, i, j])

        problem.optimize()


        if show_result == 1:
            list = []
            for var in problem.getVars():
                if "X" in var.VarName:
                    if var.xn > 0:
                        list.append(self.find_edge_from_variable_name(var.varName))



            list2 = []
            for var in problem.getVars():
                if "Z" in var.VarName:
                    if var.xn > 0:
                        list2.append(self.split_variable_name_Z(var.varName))
            draw = draw_result()
            draw.draw_trucks(nodes=self.NODES, costs=self.costs, K=self.K, truck=self.truck,P=self.P,pallet=self.pallet,pallet_weight=self.pallet_weight,coordinate_of_location=self.coordinate_of_location, locations=list2, edge_list=list, objective=problem.getObjective().getValue())

        return problem.Runtime, problem.numVars, problem.numConstrs,problem.numNZs,problem.getObjective().getValue(), problem.MIPgap


    def find_edge_from_variable_name(self, name):
        s1 = '['
        s2 = ']'
        name = name[name.index(s1) + 1: name.index(s2)]
        name = name.split(",")
        return [name[0], name[1], name[2]]


    def split_variable_name_Z(self, position_location):
        s1 = '['
        s2 = ']'
        position_location = position_location[position_location.index(s1) + 1: position_location.index(s2)]
        position_location = position_location.split(",")
        return [position_location[0], position_location[1], position_location[2], position_location[3]]
