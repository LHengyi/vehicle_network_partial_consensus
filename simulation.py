import os, sys
import traci
import setting
import random
from xml.dom import minidom
import math
import vehicle

if 'SUMO_HOME' in os.environ:
    tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
    sys.path.append(tools)
else:   
    sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "sumo-gui.exe"
sumoCmd = [sumoBinary, "-c", "lane_change.sumocfg"]
Vehicles = []
N = 4
cur_vehicle_list = []
all_vehicles = []

def route_gen(arrRate, route_file):
	random.seed(setting.seed_value)
	departTime = 0
	startingLanes = [0,1,2]
	endingLanes = [0,1,2]
	with open(route_file,'w') as outfile:
		print('<routes>',file=outfile)
		print('<route id = "Route1" edges = "edge1 edge2" />',file=outfile)

		for i in range(N):
			departTime += -(math.log(1.0 -random.random())/arrRate)
			departLane = random.choice(startingLanes)
			arrivalLane = random.choice([i for i in endingLanes if i != departLane])
			print('<vehicle id="%s" route="%s" depart="%f" \
					departSpeed="10" departPos="base" color="1,0,0" departLane="%d" \
					arrivalLane="%d"/>' % ("V" + str(i), "Route1",\
					 departTime, departLane,arrivalLane),file=outfile) #default vehicle model
			v = Vehicle("V"+str(i), ["edge1","edge2"], departTime, 10, departLane, arrivalLane)
			all_vehicles.append(v)
		print('</routes>',file=outfile)
	all_vehicles = sorted(all_vehicles,key=lambda v: v.departTime)

def run():
	traci.start(sumoCmd)
	step = 0
	edges = ['edge1_0', 'edge1_1', 'edge1_2', 'edge2_0', 'edge2_1','edge2_2']
	while traci.simulation.getMinExpectedNumber() != 0:
		traci.simulationStep()
		partial_consensus()
		step+=1
	traci.close()

def partial_consensus():
	# find all vehicles in simulation
	inserted_vehicle = traci.simulation.getDepartedIDList()
	if len(inserted_vehicle) > 1:
		for v in inserted_vehicle:
			cur_vehicle_list.append(v)
	else:
		cur_vehicle_list.append(inserted_vehicle[0])

	arrived_list = traci.simulation.getArrivedIDList()
	inserted_vehicle = [v for v in inserted_vehicle if v not in arrived_list]

	for v in all_vehicles:
		#get current lane index, 0 for rightmost lane
		if v.name in inserted_vehicle:
			cur_lane_idx = traci.vehicle.getLaneIndex(v)
		else:
			continue
		#get destination lane index
		arrive_lane_idx = v.arriveLane

		# get direction right: -1   left: 1     current lane: 0
		if arrive_lane_idx > cur_lane_idx:
			direction = 1
		else if arrive_lane_idx < cur_lane_idx:
			direction = -1
		else:
			direction = 0

		#fine preceding vehicle, follower on each adjacent lane
		ego_position = traci.vehicle.getPosition(v.name) # value in [m,m]
		if direction != 0:
			





if __name__ == "__main__":
	setting.init()
	# route_gen(setting.arrRate,"lane_change.rou.xml")
	run()
	