import os
import sys
import random
import math
import setting

class Vehicle(object):
	def __init__(self, name, route, departTime, departSpeed, departLane, arriveLane):
		# assert type(vtype) is Vtype, "vtype is not class Vtype"
		self.name = name
		# self.vtype = vtype #class Vtype
		self.route = route
		self.routeEdges = []
		self.departTime = departTime
		self.departSpeed = departSpeed
		self.departLane = departLane
		self.arriveLane = arriveLane
		self.real_leave_lane = None
		self.leaveTime = 0
		self.mID = 0
		self.arrived = False
		self.arriveTime = 0
		self.decision = (None,-1)
		#self.leaveIntersection = LeaveIntersection()
		if setting.log == True:
			self.timelog = []
			self.locationlogx = []
			self.locationlogy = []
			self.statelog = []
			self.degree = []
			self.location = []
			self.speed = []

class Vtype(object):
    """defines the vehicle types"""
    def __init__(self, name, accel, decel, sigma, length, minGap, maxSpeed, guiShape,cfm):
        self.name = name
        self.accel = accel
        self.decel = decel
        self.sigma = sigma
        self.length = length
        self.minGap = minGap
        self.maxSpeed = maxSpeed
        self.guiShape = guiShape
        self.car_following_model = cfm