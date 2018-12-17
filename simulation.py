import os, sys
import traci
import setting
import optparse
import random
from xml.dom import minidom
import math
from vehicle import *
from collections import defaultdict
import numpy as np


if 'SUMO_HOME' in os.environ:
	tools = os.path.join(os.environ['SUMO_HOME'], 'tools')
	sys.path.append(tools)
else:
	sys.exit("please declare environment variable 'SUMO_HOME'")

sumoBinary = "sumo-gui.exe"
sumoCmd = [sumoBinary, "-c", "lane_change.sumocfg"]
# global Vehicles
Vehicles = []
# global N
N = 40
# global cur_vehicle_list
cur_vehicle_list = []
# global all_vehicles
all_vehicles = defaultdict()

def safety_distance(ego_vehicle,preceding_vehicle):
	ego_cur_v = traci.vehicle.getSpeed(ego_vehicle)
	ego_max_dec = traci.vehicle.getDecel(ego_vehicle)
	coop_cur_v = traci.vehicle.getSpeed(cooperative_vehicle)
	coop_max_dec = traci.vehicle.getDecel(cooperative_vehicle)
	g_safe = 0.1 * ego_cur_v + math.pow(ego_cur_v,2)/(2*ego_max_dec) - math.pow(coop_cur_v,2)/(2*coop_max_dec) + 1.0
	return g_safe

def find_pred(ego_vehicle, target_lane_idx):
	ego_pos = traci.vehicle.getPosition(ego_vehicle)
	target_lane_ID = (cur_lane_ID + '.')[:-1]
	target_lane_ID = target_lane_ID[:-1] + str(farest_lane_idx)
	vehicles_on_target = traci.lane.getLastStepVehicleIDs(target_lane_ID)
	pre = None
	for target_v in vehicles_on_target:
		target_v_pos = traci.vehicle.getPosition(target_v)
		if target_v_pos[0] > ego_pos[0]:
			if pre:
				pre_pos = traci.vehicle.getPosition(pre)
				if pre_pos[0] > target_v_pos[0]:
					pre = target_v
			else:
				pre = target_v
	if not pre:
		ego_roadID = traci.vehicle.getRoadID(ego_vehicle)
		ego_route = traci.vehicle.getRoute(ego_vehicle)
		if ego_roadID != ego_route[-1]:
			next_lane_ID = vehicle_route[-1] + "_" + str(target_lane_idx)#only two edge
			next_lane_vehicle_list = traci.lane.getLastStepVehicleIDs(next_lane_ID)
			if next_lane_vehicle_list:
				pre = next_lane_vehicle_list[0]
	return pre
def find_follow(ego_vehicle, target_lane_idx):
	ego_pos = traci.vehicle.getPosition(ego_vehicle)
	target_lane_ID = (cur_lane_ID + '.')[:-1]
	target_lane_ID = target_lane_ID[:-1] + str(farest_lane_idx)
	vehicles_on_target = traci.lane.getLastStepVehicleIDs(target_lane_ID)
	follower = None
	for target_v in vehicles_on_target:
		target_v_pos = traci.vehicle.getPosition(target_v)
		if target_v_pos[0] < ego_pos[0]:
			if follower:
				cur_follow_pos = traci.vehicle.getPosition(follower)
				if cur_follow_pos[0] < target_v_pos[0]:
					follower = target_v
			else:
				follower = target_v
	return follower
		

def safety_level_decision(ego_vehicle, local_group, decision_pool, level = 1):
	global all_vehicles
	ego_decision = all_vehicles[ego_vehicle].decision
	ego_intention = lane_change_intention(ego_vehicle)
	if ego_intention == 0:
		all_vehicles[ego_vehicle].decision = (ego_vehicle,0)
	if level == 1:#assume global consensus
		for local_v in local_group:
			all_vehicles[local_v].decision = ego_decision
	elif level == 2:
		ego_pos = traci.vehicle.getPosition(ego_vehicle)
		cur_lane_idx = traci.vehicle.getLaneIndex(ego_vehicle)
		cur_lane_ID = traci.vehicle.getLaneID(ego_vehicle)
		cur_pre = find_pred(ego_vehicle,cur_lane_idx)
		if(cur_lane_idx == 0 or cur_lane_idx == 2):
			# ego_intention = lane_change_intention(ego_vehicle)
			target_lane_idx = cur_lane_idx + ego_intention
			target_lane_pre = find_pred(ego_vehicle,target_lane_idx)
			next_next_lane_pre = find_pred(ego_vehicle,cur_lane_idx+2*ego_intention)
			all_vehicles[target_lane_pre].decision = (None,0)
			all_vehicles[next_next_lane_pre].decision = (None,0)
			target_lane_follower = find_follow(ego_vehicle,target_lane_idx)
			next_next_lane_follower = find_follow(ego_vehicle,cur_lane_idx+2*ego_intention)
			all_vehicles[target_lane_follower].decision = (None,0)
			all_vehicles[next_next_lane_follower].decision = (None,0)

def lane_change_intention(ego_vehicle):
	global all_vehicles
	cur_lane_idx = traci.vehicle.getLaneIndex(ego_vehicle)
	return all_vehicles[ego_vehicle].arriveLane - cur_lane_idx



def route_gen(arrRate, route_file):
	global all_vehicles
	random.seed(setting.seed_value)
	departTime = 0
	startingLanes = [0,1,2]
	endingLanes = [0,1,2]
	with open(route_file,'w') as outfile:
		print('<routes>',file=outfile)
		print('<route id = "Route1" edges = "edge1 edge2" />',file=outfile)
		if setting.partial_consensus:
			type1 = Vtype("type1", 1.0, 3.0, 0.5, 5, 2.5, 10, "passenger","IDM")
			print('<vType id="%s" carFollowModel="%s"/>' %(type1.name,type1.car_following_model),file=outfile)
		else:
			type1 = Vtype("type1", 1.0, 3.0, 0.5, 5, 2.5, 10, "passenger","Krauss")
			print('<vType id="%s" carFollowModel="%s" tau="1.0"/>' %(type1.name,type1.car_following_model),file=outfile)
		for i in range(N):
			departTime += -(math.log(1.0 -random.random())/arrRate)
			departLane = random.choice(startingLanes)
			arrivalLane = random.choice([i for i in endingLanes if i != departLane])
			# print('<vehicle id="%s" type= "type1" route="%s" depart="%f" \
			# 		departSpeed="10" departPos="base" color="1,0,0" departLane="%d" \
			# 		arrivalLane="%d"/>' % ("V" + str(i), "Route1",\
			# 		 departTime, departLane,arrivalLane),file=outfile) #default vehicle model
			print('<vehicle id="%s" type= "type1" route="%s" depart="%f" \
					departSpeed="10" departPos="base" color="1,0,0" />' % ("V" + str(i), "Route1",\
					 departTime),file=outfile)
			v = Vehicle("V"+str(i), ["edge1","edge2"], departTime, 10, departLane, arrivalLane)
			all_vehicles["V"+str(i)] = (v)
		print('</routes>',file=outfile)
	# all_vehicles = sorted(all_vehicles.items(),key=lambda k_v: k_v[1].departTime)

def run(pc_level,max_decision = 2):
	traci.start(sumoCmd)
	step = 0
	edges = ['edge1_0', 'edge1_1', 'edge1_2', 'edge2_0', 'edge2_1','edge2_2']
	while traci.simulation.getMinExpectedNumber() != 0:
		traci.simulationStep()

		global all_vehicles
		arrived_list = traci.simulation.getArrivedIDList()
		currentTime = traci.simulation.getCurrentTime() * 0.001
		for arr_v in arrived_list:
			all_vehicles[arr_v].leaveTime = currentTime
		if setting.partial_consensus:
			partial_consensus(pc_level, max_decision)
		step+=1
	traci.close()

def partial_consensus(pc_level=None,pool_size = 2):
	assert pool_size >= 1
	if pc_level:
		consensus_level = pc_level
	else:
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
	currentTime = traci.simulation.getCurrentTime() * 0.001
	for arr_v in arrived_list:
		all_vehicles[arr_v].leaveTime = currentTime

	if __debug__:
		print("Arrived vehicle list:\t", arrived_list)
	if __debug__:
		print("Current vehicles on road:\t", cur_vehicle_list)
	cur_vehicle_list = [v for v in cur_vehicle_list if v not in arrived_list]
	for cur_v in cur_vehicle_list:
		traci.vehicle.setLaneChangeMode(cur_v,256)

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

		#find preceding vehicle, follower on each adjacent lane
		ego_position = traci.vehicle.getPosition(v.name) # value in [m,m]
		local_group = [v.name]
		if direction != 0:
			traci.vehicle.setColor(v.name,(255,128,0))
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
				target_lane_vlist = traci.lane.getLastStepVehicleIDs(target_lane_ID)
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
				target_lane_vlist = traci.lane.getLastStepVehicleIDs(target_lane_ID)
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
		else:#no lane-changing,
			traci.vehicle.setColor(v.name,(255,0,0))
			lane_vehicle_list = traci.lane.getLastStepVehicleIDs(cur_lane_ID)
			cur_pre = None
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
			if not cur_pre:
				vehicle_roadID = traci.vehicle.getRoadID(v.name)
				vehicle_route = traci.vehicle.getRoute(v.name)
				if vehicle_roadID != vehicle_route[-1]:
					next_lane_ID = vehicle_route[-1] + "_" + str(cur_lane_idx)#only two edge
					next_lane_vehicle_list = traci.lane.getLastStepVehicleIDs(next_lane_ID)
					if next_lane_vehicle_list:
						cur_pre = next_lane_vehicle_list[0]
			if cur_pre:
				next_speed = CACC_check(v.name, cur_pre)
				delta = traci.simulation.getDeltaT()
				traci.vehicle.slowDown(v.name,next_speed, delta)
			else:
				traci.vehicle.setSpeed(v.name,-1)
			continue

			# traci.vehicle.setSpeed(v.name,-1)
		if __debug__:
			if direction != 0:
				print("The current lane change intention of ego vehicle ",v.name, "is: ", direction ,"\tThe local group is ", ":\t" ,local_group)
		#making decision
		ego_decision = (v.name,direction)
		all_vehicles[v.name].decision = ego_decision
		if len(local_group) == 1:
			posibility = 0
		else:
			posibility = random.random()
		if __debug__:
			print("possibility: ", posibility)
		if posibility <= consensus_level[0]:#global consensus
			for local_v in local_group:
				all_vehicles[local_v].decision = ego_decision
		else:# pool_size decision
			other_member = [x for x in local_group if x != v.name]
			if len(other_member) >= pool_size-1:
				attackers = random.choices(other_member, k = pool_size-1)#the randomly chosen vehicle must have different decision
			else:
				continue#no other member, not even a direct preceding vehicle
			decision_poll = [ego_decision]
			victims = random.choices(other_member, k = pool_size-1)
			for vic in victims:
				vic_lane_idx = traci.vehicle.getLaneIndex(vic)
				fake_decision = random.choice([1,-1])
				if vic_lane_idx + fake_decision not in range(0,3):
					fake_decision = vic_lane_idx - fake_decision
				decision_poll.append((vic,fake_decision))
			if __debug__:
				print("Decision poll: ", decision_poll)
			all_vehicles[v.name].decision = ego_decision
			for atk_idx in range(0,len(attackers)):
				all_vehicles[attackers[atk_idx]].decision = decision_poll[atk_idx+1]

			for local_v in [x for x in other_member if x not in attackers]:
				all_vehicles[local_v].decision = random.choice(decision_poll)
		if __debug__:
			for local_v in local_group:
				print(local_v, " make decision:\t", all_vehicles[local_v].decision)
		safety_checker(v.name,local_group)
	if __debug__:
		print("\n\n")

def safety_checker(ego_vehicle,local_group):
	ego_decision = all_vehicles[ego_vehicle].decision
	min_speed = 100
	for local_v in [x for x in local_group if x != ego_vehicle]:
		if ego_decision == all_vehicles[local_v].decision:
			# if __debug__:
			# 	print(ego_vehicle," and ", local_v, " have same decision")
			min_speed = min(min_speed ,CACC_check(ego_vehicle, local_v))
		else:
			# if __debug__:
			# 	print(ego_vehicle," and ", local_v, " have different decision")
			min_speed = min(min_speed,human_driver_check(ego_vehicle, local_v))
	delta = traci.simulation.getDeltaT()
	traci.vehicle.slowDown(ego_vehicle,min_speed, delta)


def CACC_check(ego_vehicle, cooperative_vehicle):
	K_sc = 0.4#speed control gain 0.4 s-1
	K_a = 0.66# acceleration gain 0.66 s-1
	K_v = 0.5#speed gain s-1
	K_g = 1.5#gap gain 4.08 s-2 in the paper but not suitable in this simulation
	g_min = 2.0# min space gap m
	t_g = 0.55# desired time gap s
	ego_cur_v = traci.vehicle.getSpeed(ego_vehicle)
	ego_max_dec = traci.vehicle.getDecel(ego_vehicle)
	coop_cur_v = traci.vehicle.getSpeed(cooperative_vehicle)
	coop_max_dec = traci.vehicle.getDecel(cooperative_vehicle)
	#get the maximum deceleration of preceding vehicle
	ego_pos = traci.vehicle.getPosition(ego_vehicle)
	coop_pos = traci.vehicle.getPosition(cooperative_vehicle)
	#currently we consider the influence of preceding vehicle first, the influence of the follower is not considered
	if coop_pos[0] > ego_pos[0]:#cooperative_vehicle is the preceding vehicle of the ego vehicle
		g_safe = 0.1 * ego_cur_v + math.pow(ego_cur_v,2)/(2*ego_max_dec) - math.pow(coop_cur_v,2)/(2*coop_max_dec) + 1.0

		#the speed control mode (SC)
		ego_lane_id = traci.vehicle.getLaneID(ego_vehicle)
		desired_speed = traci.lane.getMaxSpeed(ego_lane_id)
		ego_max_speed = traci.vehicle.getMaxSpeed(ego_vehicle)
		desired_speed = min(ego_max_speed,desired_speed)
		acc_v = K_sc*(desired_speed - ego_cur_v)
		if __debug__:
			print("acc_v:\t%f = %f *(%f - %f)"%(acc_v,K_sc,desired_speed,ego_cur_v))
		#the gap control model (GC)
		coop_cur_acc = traci.vehicle.getAcceleration(cooperative_vehicle)

		acc_g = K_a*coop_cur_acc + K_v*(coop_cur_v - ego_cur_v) + K_g*(coop_pos[0] - ego_pos[0] - g_min - ego_cur_v*t_g)
		if __debug__:
			print("acc_g:\t%f = %f * %f + %f*(%f - %f) + %f*(%f - %f - %f - %f*%f)" %(acc_g,K_a,coop_cur_acc,K_v,coop_cur_v,ego_cur_v,K_g,coop_pos[0],ego_pos[0],g_min,ego_cur_v,t_g))

		acc_des = min(acc_g, acc_v)# we ignore the actuation time lag, assume acc_des will be the applied acceleration
		acc_des = min(acc_des,traci.vehicle.getAccel(ego_vehicle))
		if acc_des < 0:
			acc_des = max(acc_des, traci.vehicle.getDecel(ego_vehicle))
		#get delta
		delta = traci.simulation.getDeltaT()
		target_speed = ego_cur_v + acc_des * delta
		if __debug__:
			if target_speed < 0:
				target_speed = 0.01
		return target_speed
		# traci.vehicle.slowDown(ego_vehicle,ego_cur_v + acc_des * delta, delta)
	return 100


def human_driver_check(ego_vehicle, hd_vehicle):
	gain_alpha = 0.05# s-1
	gain_beta = 0.15# s-1
	# tau = 1#human delay 1s
	ego_cur_v = traci.vehicle.getSpeed(ego_vehicle)
	ego_cur_acc = traci.vehicle.getAcceleration(ego_vehicle)
	ego_pos = traci.vehicle.getPosition(ego_vehicle)
	hd_cur_v = traci.vehicle.getSpeed(hd_vehicle)
	hd_cur_acc = traci.vehicle.getAcceleration(hd_vehicle)
	hd_pos = traci.vehicle.getPosition(hd_vehicle)
	ego_lane_id = traci.vehicle.getLaneID(ego_vehicle)
	desired_speed = traci.lane.getMaxSpeed(ego_lane_id)
	ego_max_speed = traci.vehicle.getMaxSpeed(ego_vehicle)
	vehicle_length = traci.vehicle.getLength(ego_vehicle)
	if hd_pos[0] > ego_pos[0]:
		dis_headway = (hd_pos[0] - hd_cur_v) - (ego_pos[0] - ego_cur_v) - vehicle_length
		range_error = gain_alpha * (range_policy(dis_headway,ego_max_speed) - 0 if ego_cur_v - ego_cur_acc < 0 else ego_cur_v - ego_cur_acc)
		if __debug__:
			print("%f = %f * (%f -%f)" %(range_error,gain_alpha,range_policy(dis_headway,ego_max_speed), 0 if ego_cur_v - ego_cur_acc < 0 else ego_cur_v - ego_cur_acc))
		hd_pre_v = 0 if hd_cur_v - hd_cur_acc < 0 else  hd_cur_v - hd_cur_acc
		ego_pre_v = 0 if ego_cur_v - ego_cur_acc < 0 else ego_cur_v - ego_cur_acc
		sat_error = gain_beta * (saturation_function(hd_pre_v,ego_max_speed) - ego_pre_v)
		if __debug__:
			print("%f = %f * (%f - %f)"%(sat_error,gain_beta,saturation_function(hd_pre_v,ego_max_speed),ego_pre_v))
		acc = range_error + sat_error
		acc = min(acc,traci.vehicle.getAccel(ego_vehicle))
		if acc < 0:
			acc = max(acc, traci.vehicle.getDecel(ego_vehicle))
		delta = traci.simulation.getDeltaT()
		target_speed = ego_cur_v + acc * delta
		if __debug__:
			if target_speed < 0:
				target_speed = 0.01
		return target_speed
		# traci.vehicle.slowDown(ego_vehicle,ego_cur_v + acc * delta, delta)
	return 100



def range_policy(distance, v_max):
	lower_bound = 5 #m
	gradient = 1# 1 s-1
	speed = 0
	upper_bound = v_max/gradient + lower_bound
	if distance <= lower_bound:
		speed = 0
	elif  distance > lower_bound and distance < upper_bound:
		speed = (distance - lower_bound) * gradient
	else:
		speed = v_max
	return speed

def saturation_function(speed, v_max):
	if speed <= v_max:
		return speed
	else:
		return v_max
	
def get_result(output_file):
	output = open(output_file,'w')
	for k,v in all_vehicles.items():
		print(v.leaveTime - v.departTime,file=output)




if __name__ == "__main__":
	#parse input
	setting.init()
	optParser = optparse.OptionParser()
	optParser.add_option("--seed", action="store", dest="seed", default=101, type="int", help="the seed for random generation")
	optParser.add_option("--output", action="store", dest="output", default="summary.txt", type="string", help="the output file to record the simulation")
	optParser.add_option("--pc", action="store_true", default=False, help="if set, considering partial consensus in the simulation")
	optParser.add_option("--pc-comp",action="store",dest = "pc_comp",default=5,type="float",help="the partial consensus level")
	optParser.add_option("--rate",action="store",dest="rate",default=3,type="float",help="vehicle arrive rate")
	optParser.add_option("--max_decision",action="store",dest="max_decision",default=2,type="int",help="The maximum number of disagreement")
	options, args = optParser.parse_args()

	setting.partial_consensus = options.pc
	setting.seed_value = options.seed
	pc_comp = options.pc_comp
	setting.arrRate = options.rate/10
	setting.max_decision = options.max_decision
	# pc_component = range(0,1,0.1)
	route_gen(setting.arrRate,"lane_change.rou.xml")
	# for pc_i in np.arange(0,1.1,0.1)
	run([1-pc_comp/10,pc_comp/10],setting.max_decision)#global,partial
	get_result(options.output)