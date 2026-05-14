import ortools 
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import numpy as np
import pandas as pd
from pathlib import Path

def main():
    Demand = pd.read_csv("data/Demand_List.csv")
    Dist_Matrix = pd.read_csv("data/Dist_Matrix.csv")

    ## Demand dataframe to list
    Demand_Matrix = Demand["Demand"].tolist() 
    print(Demand_Matrix)

    ## Destination Dataframe to list
    Distance_Matrix = Dist_Matrix["Distance"].tolist() 
    print(Distance_Matrix)


if __name__ == "__main__":
    main()