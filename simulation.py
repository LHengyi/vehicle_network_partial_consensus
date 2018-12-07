import os, sys
import traci
import setting
import random
from xml.dom import minidom
import math
from vehicle import *
from collections import defaultdict
if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:
	sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "sumo.exe"
sumoCmd = [sumoBinary, "-c", "lane_change.sumocfg"]
# global Vehicles
Vehicles = []
# global N
N = 4
# global cur_vehicle_list
cur_vehicle_list = []
# global all_vehicles
all_vehicles = defaultdict()

def route_gen(arrRate, route_file):
	global all_vehicles
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
			all_vehicles["V"+str(i)] = (v)
		print('</routes>',file=outfile)
	# all_vehicles = sorted(all_vehicles.items(),key=lambda k_v: k_v[1].departTime)

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
	consensus_level = [0.5, 0.5]#possibility of global consensus and partial consensus (2 different dicision)
	# find all vehicles in simulation
	global cur_vehicle_list, all_vehicles
	inserted_vehicle = traci.simulation.getDepartedIDList()
	# inserted_vehicle = [x[0] for x in inserted_vehicle]
	if inserted_vehicle:
		if len(inserted_vehicle) > 1:
			for v in inserted_vehicle:
				cur_vehicle_list.append(v)
		else:
			cur_vehicle_list.append(inserted_vehicle[0])
	

	arrived_list = traci.simulation.getArrivedIDList()

	if __debug__:
		print("Arrived vehicle list:\t", arrived_list)
	if __debug__:
		print("Current vehicles on road:\t", cur_vehicle_list)
	cur_vehicle_list = [v for v in cur_vehicle_list if v not in arrived_list]

	for (k,v) in all_vehicles.items():
		#get current lane index, 0 for rightmost lane
		if v.name in cur_vehicle_list:
			cur_lane_idx = traci.vehicle.getLaneIndex(v.name)
			cur_lane_ID = traci.vehicle.getLaneID(v.name)
		else:
			continue
		#get destination lane index
		arrive_lane_idx = v.arriveLane

		# get direction right: -1   left: 1     current lane: 0
		if arrive_lane_idx > cur_lane_idx:
			direction = 1
		elif arrive_lane_idx < cur_lane_idx:
			direction = -1
		else:
			direction = 0

		#fine preceding vehicle, follower on each adjacent lane
		ego_position = traci.vehicle.getPosition(v.name) # value in [m,m]
		local_group = [v.name]
		if direction != 0:
			lane_vehicle_list = traci.lane.getLastStepVehicleIDs(cur_lane_ID)
			cur_pre = None
			cur_follow = None
			for lane_v in lane_vehicle_list:
				v_pos = traci.vehicle.getPosition(lane_v)
				# find direct preceding vehicle
				if v_pos[0] > ego_position[0]:
					if cur_pre:
						cur_pre_pos = traci.vehicle.getPosition(cur_pre)
						if cur_pre_pos[0] > v_pos[0]:
							cur_pre = lane_v
					else:
						cur_pre = lane_v
				# find direct follower
				if v_pos[0] < ego_position[0]:
					if cur_follow:
						cur_follow_pos = traci.vehicle.getPosition(cur_follow)
						if cur_follow_pos[0] < v_pos[0]:
							cur_follow = lane_v
					else:
						cur_follow = lane_v
			if cur_pre:#may be preceding vehicle is on next
				local_group.append(cur_pre)
			else:
				vehicle_roadID = traci.vehicle.getRoadID(v.name)
				vehicle_route = traci.vehicle.getRoute(v.name)
				if vehicle_roadID != vehicle_route[-1]:
					next_lane_ID = vehicle_route[-1] + "_" + str(cur_lane_idx)#only two edge
					next_lane_vehicle_list = traci.lane.getLastStepVehicleIDs(next_lane_ID)
					if next_lane_vehicle_list:
						cur_pre = next_lane_vehicle_list[0]

			if cur_follow:
				local_group.append(cur_follow)
			# direct adjacent lane
			if cur_lane_idx + direction in range(0,3):
				target_lane_idx = cur_lane_idx + direction
				target_lane_ID = (cur_lane_ID + '.')[:-1]
				target_lane_ID = target_lane_ID[:-1] + str(target_lane_idx)
				target_lane_vlist = traci.lane.getLastStepVehicleIDs(target_lane_ID)#########################wrong
				cur_pre = None
				cur_follow = None
				for lane_v in target_lane_vlist:
					v_pos = traci.vehicle.getPosition(lane_v)
					if v_pos[0] > ego_position[0]:
						if cur_pre:
							cur_pre_pos = traci.vehicle.getPosition(cur_pre)
							if cur_pre_pos[0] > v_pos[0]:
								cur_pre = lane_v
						else:
							cur_pre = lane_v
					# find direct follower
					if v_pos[0] < ego_position[0]:
						if cur_follow:
							cur_follow_pos = traci.vehicle.getPosition(cur_follow)
							if cur_follow_pos[0] < v_pos[0]:
								cur_follow = lane_v
						else:
							cur_follow = lane_v
				if cur_pre:
					local_group.append(cur_pre)
				else:
					vehicle_roadID = traci.vehicle.getRoadID(v.name)
					vehicle_route = traci.vehicle.getRoute(v.name)
					if vehicle_roadID != vehicle_route[-1]:
						next_lane_ID = vehicle_route[-1] + "_" + str(target_lane_idx)#only two edge
						next_lane_vehicle_list = traci.lane.getLastStepVehicleIDs(next_lane_ID)
						if next_lane_vehicle_list:
							cur_pre = next_lane_vehicle_list[0]
				if cur_follow:
					local_group.append(cur_follow)

			# # vehicles on lane cur_lane_idx+2
			if cur_lane_idx + 2*direction in range(0,3):
				target_lane_idx = cur_lane_idx + 2*direction
				target_lane_ID = (cur_lane_ID + '.')[:-1]
				target_lane_ID = target_lane_ID[:-1] + str(target_lane_idx)
				target_lane_vlist = traci.lane.getLastStepVehicleIDs(target_lane_ID)###########################wrong
				cur_pre = None
				cur_follow = None
				for lane_v in target_lane_vlist:
					v_pos = traci.vehicle.getPosition(lane_v)
					if v_pos[0] > ego_position[0]:
						if cur_pre:
							cur_pre_pos = traci.vehicle.getPosition(cur_pre)
							if cur_pre_pos[0] > v_pos[0]:
								cur_pre = lane_v
						else:
							cur_pre = lane_v
					# find direct follower
					if v_pos[0] < ego_position[0]:
						if cur_follow:
							cur_follow_pos = traci.vehicle.getPosition(cur_follow)
							if cur_follow_pos[0] < v_pos[0]:
								cur_follow = lane_v
						else:
							cur_follow = lane_v
				if cur_pre:
					local_group.append(cur_pre)
				else:
					vehicle_roadID = traci.vehicle.getRoadID(v.name)
					vehicle_route = traci.vehicle.getRoute(v.name)
					if vehicle_roadID != vehicle_route[-1]:
						next_lane_ID = vehicle_route[-1] + "_" + str(target_lane_idx)#only two edge
						next_lane_vehicle_list = traci.lane.getLastStepVehicleIDs(next_lane_ID)
						if next_lane_vehicle_list:
							cur_pre = next_lane_vehicle_list[0]
				if cur_follow:
					local_group.append(cur_follow)

		#making decision
		if len(local_group) == 1:
			posibility = 0
		else:
			posibility = random.random()
		if posibility < consensus_level[0]:#global consensus
			for local_v in local_group:
				all_vehicles[local_v].decision = (v.name,direction)
		else:# 2 decision
			other_member = [x for x in local_group if x != v.name]
			attacker = random.choice(other_member)
		if __debug__:
			print("The local group of ego vehicle ", v.name, ":\t" ,local_group)
	




if __name__ == "__main__":
	setting.init()
	route_gen(setting.arrRate,"lane_change.rou.xml")
	run()
	