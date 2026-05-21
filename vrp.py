import geopandas as gpd
import matplotlib.pyplot as plt
import pandas as pd
import requests
import numpy as np
from pathlib import Path

DATA_DIR = Path(__file__).parent / "project_files"
OSRM_URL = "http://localhost:5001"

clients = gpd.read_file(DATA_DIR / "Clients_CET587_2021.shp")
streets = gpd.read_file(DATA_DIR / "Seattle_Streets.shp")

clients = clients.set_crs(epsg=2926, allow_override=True).to_crs(epsg=4326)
streets = streets.to_crs(epsg=4326)

depot = clients[clients["Route"] == 0]
clients = clients[clients["Route"] != 0]

fig, ax = plt.subplots(figsize=(10, 10))
streets.plot(ax=ax, color="lightgray", linewidth=0.5, label="Streets")
clients.plot(ax=ax, color="red", markersize=10, label="Clients")
depot.plot(ax=ax, color="green", markersize=15, marker="*", label="Depot")
ax.legend()
ax.set_title("VRP Data Layers")
plt.tight_layout()
plt.show()

# Build distance matrix via OSRM /table endpoint
# Combine depot + clients so depot is index 0
all_locations = gpd.GeoDataFrame(
    pd.concat([depot, clients], ignore_index=True), crs=clients.crs
)
coords = ";".join(
    f"{row.geometry.x},{row.geometry.y}" for _, row in all_locations.iterrows()
)
response = requests.get(f"{OSRM_URL}/table/v1/driving/{coords}?annotations=distance")
response.raise_for_status()

distance_matrix = np.array(response.json()["distances"])
print(f"Distance matrix shape: {distance_matrix.shape}")
print(f"Sample distance (loc 0 → loc 1): {distance_matrix[0][1]:.1f} meters")

# --- OR-Tools VRP Solver ---
from ortools.constraint_solver import routing_enums_pb2, pywrapcp

NUM_VEHICLES = 6
VEHICLE_CAPACITY = 200
DEPOT_INDEX = 0

# number of is in cases of coffee (case = 12 1lb bags with a total of 1.5 cubic feet)
demands = [int(row["Number_of"]) for _, row in all_locations.iterrows()]
demands[0] = 0  # depot has no demand

dist_matrix_int = distance_matrix.astype(int).tolist()

manager = pywrapcp.RoutingIndexManager(len(dist_matrix_int), NUM_VEHICLES, DEPOT_INDEX)
routing = pywrapcp.RoutingModel(manager)

def distance_callback(from_index, to_index):
    return dist_matrix_int[manager.IndexToNode(from_index)][manager.IndexToNode(to_index)]

transit_idx = routing.RegisterTransitCallback(distance_callback)
routing.SetArcCostEvaluatorOfAllVehicles(transit_idx)

def demand_callback(from_index):
    return demands[manager.IndexToNode(from_index)]

demand_idx = routing.RegisterUnaryTransitCallback(demand_callback)
routing.AddDimensionWithVehicleCapacity(
    demand_idx, 0, [VEHICLE_CAPACITY] * NUM_VEHICLES, True, "Capacity"
)

search_params = pywrapcp.DefaultRoutingSearchParameters()
search_params.first_solution_strategy = routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC
search_params.time_limit.seconds = 30

print("Solving VRP...")
solution = routing.SolveWithParameters(search_params)

if solution:
    print(f"Objective (total distance): {solution.ObjectiveValue()} meters")
    for v in range(NUM_VEHICLES):
        index = routing.Start(v)
        stops, load, dist = [], 0, 0
        while not routing.IsEnd(index):
            node = manager.IndexToNode(index)
            stops.append(node)
            load += demands[node]
            prev = index
            index = solution.Value(routing.NextVar(index))
            dist += routing.GetArcCostForVehicle(prev, index, v)
        print(f"Vehicle {v}: {len(stops)-1} stops, load={load}, distance={dist}m")
else:
    print("No solution found.")
