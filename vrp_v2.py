from pathlib import Path

import pandas as pd
from ortools.constraint_solver import pywrapcp
from ortools.constraint_solver import routing_enums_pb2


def create_data_model(num_vehicles: int = 4, vehicle_capacity: int = 100) -> dict:
    data_dir = Path(__file__).resolve().parent / "data"

    demand_df = pd.read_csv(data_dir / "Demand_List.csv")
    dist_df = pd.read_csv(data_dir / "Dist_Matrix.csv")

    demands = demand_df["Demand"].astype(int).tolist()
    n_locations = len(demands)

    flat_distances = dist_df["Distance"].astype(int).tolist()
    expected_size = n_locations * n_locations
    if len(flat_distances) != expected_size:
        raise ValueError(
            f"Distance matrix size mismatch: got {len(flat_distances)}, expected {expected_size}"
        )

    distance_matrix = [
        flat_distances[i * n_locations : (i + 1) * n_locations]
        for i in range(n_locations)
    ]

    return {
        "distance_matrix": distance_matrix,
        "demands": demands,
        "vehicle_capacities": [vehicle_capacity] * num_vehicles,
        "num_vehicles": num_vehicles,
        "depot": 0,
    }


def print_solution(data, manager, routing, solution):
    print(f"Objective: {solution.ObjectiveValue()}")
    total_distance = 0
    total_load = 0

    for vehicle_id in range(data["num_vehicles"]):
        index = routing.Start(vehicle_id)
        route_distance = 0
        route_load = 0
        route_nodes = []

        while not routing.IsEnd(index):
            node_index = manager.IndexToNode(index)
            route_nodes.append(node_index)
            route_load += data["demands"][node_index]
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, vehicle_id)

        route_nodes.append(manager.IndexToNode(index))
        route_path = " -> ".join(str(node) for node in route_nodes)

        print(f"Route for vehicle {vehicle_id}: {route_path}")
        print(f"Distance of the route: {route_distance}")
        print(f"Load of the route: {route_load}\n")

        total_distance += route_distance
        total_load += route_load

    print(f"Total distance of all routes: {total_distance}")
    print(f"Total load of all routes: {total_load}")


def main():
    data = create_data_model()

    manager = pywrapcp.RoutingIndexManager(
        len(data["distance_matrix"]), data["num_vehicles"], data["depot"]
    )
    routing = pywrapcp.RoutingModel(manager)

    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return data["distance_matrix"][from_node][to_node]

    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

    def demand_callback(from_index):
        from_node = manager.IndexToNode(from_index)
        return data["demands"][from_node]

    demand_callback_index = routing.RegisterUnaryTransitCallback(demand_callback)
    routing.AddDimensionWithVehicleCapacity(
        demand_callback_index,
        0,
        data["vehicle_capacities"],
        True,
        "Capacity",
    )

    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
    )

    solution = routing.SolveWithParameters(search_parameters)
    if solution is None:
        raise RuntimeError("No solution found. Try increasing vehicle count or capacity.")

    print_solution(data, manager, routing, solution)


if __name__ == "__main__":
    main()
