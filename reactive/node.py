# this class describes the wireless node and it is used by readFile.py
# this script is part of the reactive algorithm
# by Kalypso Magklara (kalypso.magklara(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)


from __future__ import division
__metaclass__ = type    #new style classes, default after version 3
import bs
import robot
import math

#import sys
#sys.setrecursionlimit(1500)
NMAX_ENERGY = 20000     #node max energy in J
INACTIVE = 0.00000001   #10 nJ/bit
PACKETSIZE = 8192       #1024 byte --> 8192 bit
A11 = 0.00000005        #50 nJ/bit
A12 = 0.0000000001      #100 pJ/bit
A2 = 0.0000001          #100 nJ/bit
Asense = 0.0000001      #100 nJ/bit
                        #1 packet (1 kB) per 10 sec
                        #if no aggregation then it sends as many packets as it receives
                        #if aggregation is used then number of packets to send is
                        #__                     __
                        #| info size * rcvnodes  |
                        #| --------------------- |
                        #|      packetsize       |
#grid is square with x = 141m -> diagonal =del 199.4 m if robot speed 0.9m/sec -> needs 221 sec to go across

def calculateDependents(node, todo, temp = []):
    '''node must be a leaf in the node tree, recover next node closer to root add the leaf or the leafs of the neighbour
    copy the leafs to be removed in temp and go to next neighbour'''
    #print 'node id '+str(node.getId())
    if todo == 'add':
        #print 'adding the following to its neigh'
        for neigh in node.getNeighbours():
            #print 'neigh id '+str(neigh.getId())
            if neigh.getHops() == 0:
                break
            elif (neigh.getHops() < node.getHops()) & (neigh not in Node.deadNodes):
                #print neigh.getId()
                if node.getStatus() == 'sensor':
                    if node not in neigh.getDependents():
                        neigh.addDependents(node)
                        #print 'node to the neigh because it is a sensor'
                        neigh.calcConsumption()
                        neigh.calcEnergyThreshold()
                    calculateDependents(neigh, 'add')
                elif node.getStatus() == 'cds':
                    for dep in node.getDependents():
                        if dep not in neigh.getDependents():
                            neigh.addDependents(dep)
                            #print dep.getId()
                            neigh.calcConsumption()
                            neigh.calcEnergyThreshold()
                    calculateDependents(neigh, 'add')
    elif todo == 'rm':
#        print str(node.getId()) +' is removing the following neighs'
        for neigh in node.getNeighbours():
            if neigh.getHops() == 0:
                break
            elif neigh.getHops() < node.getHops():
#                print neigh.getId()
                if len(temp) == 0:
                    if node.getStatus() == 'sensor':
                        if node in neigh.getDependents():
                            neigh.rmDependents(node)
                            if node not in temp:
                                temp.append(node)
                            neigh.calcConsumption()
                            neigh.calcEnergyThreshold()
                        calculateDependents(neigh, 'rm', temp)
                    elif node.getStatus() == 'cds':
                        for dep in node.getDependents():
                            if dep in neigh.getDependents():
                                neigh.getDependents().remove(dep) #neigh.rmDependents(dep)
                                if dep not in temp:
                                    temp.append(dep)
                                neigh.calcConsumption()
                                neigh.calcEnergyThreshold()
                        calculateDependents(neigh, 'rm', temp)
                else:
                    for dep in temp:
                        if dep in neigh.getDependents():
                            neigh.getDependents().remove(dep) #neigh.rmDependents(dep)
                            neigh.calcConsumption()
                            neigh.calcEnergyThreshold()
                    calculateDependents(neigh, 'rm', temp)
        del temp[:]

def recalculateDependents(node):
    '''called when dead node is replaced, checks all neighbours, adds their dependents
    if neighbour is sensor adds neighbour'''
    #print str(node.getId())+' will recalculateDependents and add the following'
    changesmade = 0
    for neigh in node.getNeighbours():
        #print 'checking neibour no '+str(neigh.getId())
        if neigh.getHops() == 0:
            continue
        elif (neigh.getStatus() == 'sensor') & (neigh not in Node.deadNodes):
            if neigh not in node.getDependents():
#                print neigh.getId()
                node.addDependents(neigh)
                changesmade += 1
        elif (neigh.getHops() > node.getHops()) & (neigh not in Node.deadNodes):
            for dep in neigh.getDependents():
                if dep not in node.getDependents():
#                    print dep.getId()
                    node.addDependents(dep)
                    changesmade += 1
    if changesmade > 0:
#        print 'changes has been made'
#        print 'consumption before '+str(node.getConsumption())
#        print 'threshold before '+str(node.getThreshold())
        node.calcConsumption()
        node.calcEnergyThreshold()
#        print 'consumption after '+str(node.getConsumption())
#        print 'threshold after '+str(node.getThreshold())
#        print 'now will call calculateDependents'
        calculateDependents(node, 'add')
    if node.getStatus() == 'sensor':
        node.calcConsumption()
        node.calcEnergyThreshold()
        calculateDependents(node, 'add')

def calcPath(node, sensor):
    for i in node.getNeighbours():
        if i.getHops() < node.getHops():
            sensor.setPath(i)
            calcPath(i, sensor)

def distance(x1,y1,x2,y2):
    x = (x1-x2)**2 + (y1-y2)**2
    return math.sqrt(x)

class Node:
    nid = -1
    cdsNodes = []
    inactiveNodes = []
    sensorNodes = []
    deadNodes = []
    output_path = ''

    def __init__(self, coordx = -1, coordy = -1, status = 'inactive'):
        self.energy = NMAX_ENERGY
        self.coordinates = [coordx, coordy]
        Node.nid += 1           #increase counter of nodes
        self.id = int(Node.nid) #set id of this node equal to the counter
        self.status = status
        self.dependents = []
        self.hops = 0
        self.neighbours = []
        self.alarm = False
        self.threshold = 0
        self.consumption = 0
        self.exhaustionTime = 0
        self.replacedBy = None
        self.timeDown = 0
        self.timeDie = -1
        self.timeRecover = -1
        self.path = []          #only for sensors
        self.tx = 0             #energy consumption = a11 + a12 * (d**a), tx = d**a
        if self.status == 'cds':
            self.cds = True
            Node.cdsNodes.append(self)
        elif self.status == 'inactive':
            self.cds = False
            Node.inactiveNodes.append(self)
        elif self.status == 'sensor':
            self.cds = False
            Node.sensorNodes.append(self)

    def getCoordinates(self):
        return self.coordinates

    def setCoordinates(self, coord):
        self.coordinates = list(coord)

    def getEnergy(self):
        return self.energy

    def setEnergy(self, en):
        self.energy = en
        
    def getPath(self):
        return self.path

    def setPath(self, nd):
        self.path.append(nd)
    
    def rmPath(self, nd):
        self.path.remove(nd)
    
    def getTimeDown(self):
        return self.timeDown

    def setTimeDown(self, t):
        self.timeDown = t
    
    def addTimeDown(self):
        self.timeDown += 1
    
    def getTimeDie(self):
        return self.timeDie

    def setTimeDie(self, t):
        self.timeDie = t
        
    def getTimeRecover(self):
        return self.timeRecover

    def setTimeRecover(self, t):
        self.timeRecover = t
        
    def getTx(self):
        return self.tx

    def setTx(self, t):
        self.tx = t
    
    def getConsumption(self):
        return self.consumption

    def setConsumption(self, c):
        self.consumption = c
    
    def getThreshold(self):
        return self.threshold

    def setThreshold(self, t):
        self.threshold = t
        
    def getHops(self):
        return self.hops

    def setHops(self, h):
        self.hops = h
    
    def setAlarm(self, a):
        self.alarm = a

    def sendAlarm(self, time, ttd):
        #print 'def sendAlarm'
        #print str(self.id) + ' sends alarm'
        self.alarm = True
        robot.Robot.alarms[self] = ttd

    def getNeighbours(self):
        return self.neighbours

    def setNeighbours(self, n):
        #print 'def setNeighbours'
        if n not in self.neighbours:
            self.neighbours.append(n)

    def rmNeighbours(self, node):
        #print 'def rmNeighbours'
        self.neighbours.remove(node)

    def getDependents(self):
        return self.dependents
    
    def setDependents(self, d):
        self.dependents = d

    def addDependents(self, node):
        #print 'def addDependents'
        if node not in self.dependents:
            self.dependents.append(node)

    def rmDependents(self, node):
        #print 'def rmDependents'
        if node in self.dependents:
            self.dependents.remove(node)

    def getId (self):
        return self.id

    def getAllCds(self):
        return Node.cdsNodes

    def getAllInactive(self):
        return Node.inactiveNodes

    def getAllSensors(self):
        return Node.sensorNodes
    
    def getStatus(self):
        return self.status

    def setStatus(self, value):
        #print 'def setStatus ' + str(self.id)
        #print value
        self.status = value
        if value == 'cds':
            if self not in Node.cdsNodes:
                #print 'not in cds appending'
                Node.cdsNodes.append(self)
            if self in Node.inactiveNodes:
                Node.inactiveNodes.remove(self)
            elif self in Node.sensorNodes:
                Node.sensorNodes.remove(self)
        elif value == 'inactive':
            if self not in Node.inactiveNodes:
                #print 'not in inactive appending'
                Node.inactiveNodes.append(self)
            if self in Node.cdsNodes:
                Node.cdsNodes.remove(self)
            elif self in Node.sensorNodes:
                Node.sensorNodes.remove(self)
        elif value == 'sensor':
            if self not in Node.sensorNodes:
                #print 'not in sensor appending'
                Node.sensorNodes.append(self)
            if self in Node.cdsNodes:
                Node.cdsNodes.remove(self)
            elif self in Node.inactiveNodes:
                Node.inactiveNodes.remove(self)
        else:
            if self in Node.cdsNodes:
                Node.cdsNodes.remove(self)
            elif self in Node.inactiveNodes:
                Node.inactiveNodes.remove(self)
            elif self in Node.sensorNodes:
                Node.sensorNodes.remove(self)
            if self not in Node.deadNodes:
                Node.deadNodes.append(self)

    def updateEnergy(self, time):
        #print 'def updateEnergy'
        #check if energy is below zero and then set it to zero and never change it again
        if self not in Node.deadNodes:
            count = 0
            if self.status == 'inactive':
                self.energy -= self.consumption
            if self.status == 'sensor':
                self.energy -= self.consumption #transmitting & sensing energy
            if self.status == 'cds':
                self.energy -= self.consumption #transmitting & receiving energy
            if(self.energy < self.threshold) & (not self.alarm):
                #print 'will send alarm...'
                ttd = math.ceil(self.energy/self.consumption)
                self.sendAlarm(time, ttd)
                count += 1
            if self.energy <= 0:
                self.energy = 0
                self.die(time)
            return count
        else:
            return 0
    
    def transmissionPower(self):
        'Etx = a11 + a12 * d ** a: this function calculates a'
        #print 'def transmissionPower'
        for x in self.getNeighbours():
            if self.hops > x.getHops():
                x.getId()
                my = self.coordinates
                his = x.getCoordinates()
                d = distance(my[0], my[1], his[0], his[1])
                if 0 <= d < 10:
                     self.tx = d**2
                elif 10 <= d < 20:
                    self.tx = d**2.5
                elif 20 <= d < 30:
                    self.tx = d**3
                elif 30 <= d < 40:
                    self.tx = d**3.5
                elif 40 <= d < 50:
                    self.tx = d**4
                    
    def calcConsumption(self):
        #print 'def calcConsumption for node '+ str(self.id)
        dr = self.calculateDatarate() #datarate in bits
        #print 'datarate '+ str(dr)
        if self.status == 'inactive':
            self.consumption = INACTIVE
        if self.status == 'sensor':
            en = A11 + A12 * self.tx
            self.consumption = en * dr + Asense * dr
            if self.consumption <= A11:
                self.consumption = INACTIVE
        if self.status == 'cds':
            en = A11 + A12 * self.tx
            self.consumption = en * dr + A2 * dr
            if self.consumption <= A11:
                self.consumption = INACTIVE
        #print 'cons ' + str(self.consumption)
         
    def calcEnergyThreshold(self):
        #print 'def calcEnergyThreshold for node '+ str(self.id)
        dist = distance(self.coordinates[0], self.coordinates[1], bs.BS.coordinates[0], bs.BS.coordinates[1])
        time = dist / robot.Robot.speed
        time += time * 0.1 * 2
        self.threshold = self.consumption * time
        #print self.threshold
        
    def calculateDatarate(self):
        #print 'def calculateDatarate for node '+ str(self.id)
        receive = len(self.dependents)
        #print receive
        #print self.status
        #without aggregation
        if (receive == 0) & (self.status == 'sensor'):
            self.datarate = PACKETSIZE*1/10 #every 10 seconds sends one packet
            #it only send its own data
        elif (receive == 0) & (self.status != 'sensor'):
            self.datarate = 0
        else:
            self.datarate = PACKETSIZE*receive/10 #every 10 seconds sends one packet
            
            #receives PACKETSIZE*receive/10 and forwards the same data
##        #with aggregation
##        if receive ==0:
##            self.datarate = PACKETSIZE*1/10
##            #it only send its own data
##        else:
##            self.datarate = (PACKETSIZE*receive/10) + math.ceil(INFOSIZE*len(self.neighbours)/PACKETSIZE)
##            #receives PACKETSIZE*receive/10 and forwards the same data
        return self.datarate

    def die(self, time):
        #print 'def die'
#        print str(self.id) + ' ' + str(self.status)+' died'
        self.timeDie = time
        if bs.BS.coverage < 0:
            #print 'something is wrong with the coverage counter'
            pass
        else:
            if self.status == 'sensor':
                bs.BS.coverage -= 1
            self.exhaustionTime = time
            calculateDependents(self, 'rm')
            #find neighbours and remove itself from their lists
#            for x in self.neighbours:
#                if (self in x.neighbours) & (x.getHops() < self.hops):
#                    x.neighbours.remove(self)
            Node.deadNodes.append(self)
            if self in Node.cdsNodes:
                Node.cdsNodes.remove(self)
            elif self in Node.sensorNodes:
                Node.sensorNodes.remove(self)
            del self.dependents[:]
            self.replacedBy = None
        recoverTime = open(Node.output_path+'recoverTime.txt', 'a')
        recoverTime.write(repr(self.id).ljust(4)+'down'.ljust(10)+repr(self.coordinates).ljust(18)+repr(time).ljust(8)+'\n')
        recoverTime.close()
