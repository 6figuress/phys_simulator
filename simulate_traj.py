import json
import os
from simulator import createSimulator, getSimulator

def readJson(path):
    points = []
    with open(os.path.join("./trajectories", path), "r") as file:
        data = json.load(file)["modTraj"]
        for i in data:
            points.append(i["positions"])
    return points


filename = "traj.json"

path = readJson(filename)

sim = createSimulator(gui=False, log=True)

res = sim.validateTrajectory(path)

print(f"\n\n========= Trajectory is {'VALID' if res else 'INVALID'} ===========\n")