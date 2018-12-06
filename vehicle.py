import os
import sys
import random
import math
import setting

class Vehicle(object):
	def __init__(self, name, route, departTime, departSpeed, departLane, arriveLane):
        assert type(vtype) is Vtype, "vtype is not class Vtype"
        self.name = name
        # self.vtype = vtype #class Vtype
        self.route = route
        self.routeEdges = []
        self.departTime = departTime
        self.departSpeed = departSpeed
        self.departLane = departLane
        self.arriveLane = arriveLane
        self.leaveTime = 0
        self.mID = 0
        self.arrived = False
        self.arriveTime = 0
        #self.leaveIntersection = LeaveIntersection()
        if setting.log == True:
            self.timelog = []
            self.locationlogx = []
            self.locationlogy = []
            self.statelog = []
            self.degree = []
            self.location = []
            self.speed = []