
from TruckLoadingClass import solve_pallet_loading_vehicle_routing as ProblemSolver

solver = ProblemSolver("MyInstancies/standard/palletFrom250-750/Inst5Pallet3Truck2Req25.csv")

solver.solve_problem(1)


