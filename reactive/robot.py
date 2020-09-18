# this class describes the robot and it is used by readFile.py
# this script is part of the reactive algorithm
# by Kalypso Magklara (kalypso.magklara(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)


from __future__ import division
__metaclass__ = type    #new style classes, default after version 3
import math
import node
import bs
from sympy import Matrix, solve_linear_system
from sympy.abc import x, y

MAX_PICKUP_RANGE = 100  #meters, the range in which the robot will compare the energy of active and inactive nodes
SPEED = 0.9             #m/sec
RMAX_ENERGY = 388800    #Batteries: 12.8V LIFEPO4, 10 AH, Power supply 18V / 220V
SENSING_RANGE = 50      #in meters
SLOTS = 1               #robot can carry
RECHARGE = 14400        #sec
#RECHARGE = 1	        #sec
CONSUMPTION = 24.836    #Max Speed: 0.9 - consumption: 24.836W in 1 sec --> 24.836J
IDLE = 8.568            #Consumption power in idle is: 8.568 W
                        #Transmission consumption 1 dBm -> 7.223318W .... 15 dBm -> 7.546071W

LOST_ENERGY = 0

def quadratic(x1, y1, x2, y2, dist):
    ''''y = ax + b so will solve it using the sympy module
    example >>> system = Matrix(( (1, 4, 2), (-2, 1, 14)))
    >>> solve_linear_system(system, x, y)
    {x: -6, y: 2}
    now solve distance: d**2 = (x3-x1)**2 + (y3-y1)**2 =>
    x3**2 - 2*x1x3 + (x1**2 - d**2 / (a**2 +1)) :a from y = ax + b'''
    if x1 != x2:
        system = Matrix(( (x1, 1, y1), (x2, 1, y2)))
        d = solve_linear_system(system, x, y)
        z = float(d[x])
        w = float(d[y])
        a = 1
        b = -2 * x1
        c = x1**2 - (dist**2 / (z**2+1))
        diak = b**2 - (4*a *c)
        if x1 < x2:
            if (x1 <= ((-b + math.sqrt(diak))/ (2*a)) <= x2):
                x3 = ((-b + math.sqrt(diak))/ (2*a))
            else:
                x3 = ((-b - math.sqrt(diak))/ (2*a))
            y3 = z * x3 + w
            if (distance(x3, y3, x2, y2) <= dist):
                return (x2, y2)
            else:
                return (x3, y3)
        elif x2 < x1:
            if (x2 <= ((-b + math.sqrt(diak))/ (2*a)) <= x1):
                x3 = ((-b + math.sqrt(diak))/ (2*a))
            else:
                x3 = ((-b - math.sqrt(diak))/ (2*a))
            y3 = z * x3 + w
            if (distance(x3, y3, x2, y2) <= dist):
                return (x2, y2)
            else:
                return (x3, y3)
    elif x1 == x2:
        if y1 < y2:
            y3 = y1+dist
            if (distance(x2, y3, x2, y2) <= dist):
                return (x2, y2)
            else:
                return (x2, y3)
        else:
            y3 = y1-dist
            if (distance(x2, y3, x2, y2) <= dist):
                return (x2, y2)
            else:
                return (x2, y3)

def distance(x1,y1,x2,y2):
    x = (x1-x2)**2 + (y1-y2)**2
    return math.sqrt(x)

class Robot:
    rid = -1
    alarms = {}
    speed = SPEED
    output_path = ''

    def __init__(self, coordx = -1, coordy = -1, move = False):
        self.energy = RMAX_ENERGY
        self.coordinates = [coordx, coordy]
        self.slots = SLOTS
        Robot.rid += 1
        self.id = Robot.rid
        self.range = SENSING_RANGE
        self.move = move
        self.recharge = False
        self.activeSensed = []      #id = x, y, energy
        self.inactiveSensed = []    #id = x, y, energy
        self.sensorSensed = []      #id = x, y, energy
        self.targetnode = None
        self.target = []
        self.temptarget = None      #used to pickup an inactive node
        self.carry = []

    def getactivesensed(self):
        return self.activeSensed
    
    def getinactivesensed(self):
        return self.inactiveSensed
    
    def getsensorsensed(self):
        return self.sensorSensed
    
    def getCarriedNode(self):
        return self.carry
    
    def gettargetnode(self):
        return self.targetnode
    
    def gettemptarget(self):
        return self.temptarget
    
    def getRecharge(self):
        return self.recharge

    def getCoordinates(self):
        return self.coordinates

    def setCoordinates(self, coordx, coordy):
        self.coordinates = [coordx, coordy]

    def getMove(self):
        return self.move

    def setMove(self, move):
        self.move = move

    def getEnergy(self):
        return self.energy

    def setEnergy(self, en):
        self.energy = en

    def getTarget(self):
#        if self.targetnode != None:
#            print 'exists'
#        else: print 'only coords set here'
        return self.target

    def setTarget(self, coordx = -1, coordy = -1, node=None,):
#        print 'def setTarget'
        if node:
            self.targetnode = node
            self.target = node.getCoordinates()
        else:
            self.target = [coordx, coordy]
            self.targetnode = None

    def sense(self):
#        print 'def sense'
        #iterate dictionary of coordinates of all nodes
        #if euclidean distance between robot and node < range then node is sensed
        current = self.coordinates
        for x in node.Node.cdsNodes :
            trgt = x.getCoordinates()
            #print 'trgt' + str(trgt)
            if (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (x not in self.activeSensed):
                self.activeSensed.append(x)
                #print 'sensing cds'
        for x in node.Node.inactiveNodes:
            trgt = x.getCoordinates()
            #print 'trgt' + str(trgt)
            if (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (x not in self.inactiveSensed):
                self.inactiveSensed.append(x)
                #print 'sensing inactive'
        for x in node.Node.sensorNodes:
            trgt = x.getCoordinates()
            #print 'trgt' + str(trgt)
            if (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (x not in self.sensorSensed):
                self.sensorSensed.append(x)
                #print 'sensing active'

    def updatePosition(self, time):
        ''''will use a steady speed and add something to the current value
        depending on the direction of the robot
        my direction is from (begin[0], begin[1]) to (end[0], end[1])
        ################################################################
        maybe the distance covered -----> quadratic(x1,y1,x2,y2,distance)
        should be equal to the robots sensing range (or 1.5) so at the end
        of every update i can call the sense function....
        #################################################################'''
#        print 'def updatePosition'
        if self.recharge == False:
            self.sense()
            if (len(self.carry) < SLOTS) & (self.target != list(bs.BS.coordinates)):
                #print 'not carrying any nodes'
                temp = MAX_PICKUP_RANGE
                for i in self.inactiveSensed:
                    current = self.coordinates
                    trgt = i.getCoordinates()
                    if distance(current[0], current[1], trgt[0], trgt[1]) < temp:
                        self.temptarget = i
                        temp = distance(current[0], current[1], trgt[0], trgt[1])
            if self.target == list(bs.BS.coordinates):
                self.temptarget = None
                if self.coordinates == list(bs.BS.coordinates):
                    self.recharge = True
                    self.recharging()
                    return 0
                else:
                    begin = self.coordinates
                    end = self.target
                    self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                    if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                        self.recharge = True
                        return distance(begin[0], begin[1], end[0], end[1])
                    else:
                        return SPEED
            elif self.temptarget:
                if self.coordinates == self.temptarget.getCoordinates():
                    #print 'will pickup here'
                    self.pickup(self.temptarget)
                    self.temptarget = None
                    return 0
                else:
                    begin = self.coordinates
                    end = self.temptarget.getCoordinates()
                    self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                    if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                        return distance(begin[0], begin[1], end[0], end[1])
                    else:
                        return SPEED
            elif self.targetnode:
                if (self.coordinates == self.target) & (self.targetnode != None):
                    #print 'will deliver here'
                    self.deliver(time)
                    return 0
                else:
                    begin = self.coordinates
                    end = self.target
                    self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                    if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                        return distance(begin[0], begin[1], end[0], end[1])
                    else:
                        return SPEED
        else:
            self.recharging()
            return 0
        

    def updateEnergy(self, dist):
#        print 'def updateEnergy'
        if dist > 0 :
            self.energy -= (CONSUMPTION * dist) / 0.9
        else:
            self.energy -= IDLE

    def checkEnergy(self):
#        print 'def checkEnergy'
        current = self.coordinates
        #print current
        trgt = self.target
        #print trgt
        base = bs.BS.coordinates
        #print base
        needed = CONSUMPTION * (distance(current[0], current[1], trgt[0], trgt[1]) + distance(trgt[0], trgt[1], base[0], base[1]) + 2 * MAX_PICKUP_RANGE)
        if self.energy <= needed:
            #print 'not enough energy'
            self.target = list(bs.BS.coordinates)
            self.targetnode = None

    def checkPriorities(self):
#        print 'def checkPriorities'
        if self.target == list(bs.BS.coordinates):
            return 0
        else:
            current = self.targetnode
            for x in Robot.alarms.keys():
                if current == None:
                    current = x     #keep new because there was no target before
                else:
                    cc = current.getCoordinates()   #current (node) coordinates
                    xc = x.getCoordinates()         #x (node) coordinates
                    rc = self.coordinates           #robot coordinates
                    if current in node.Node.deadNodes:
                        if (x in node.Node.deadNodes) & (current.exhaustionTime > x.exhaustionTime):
                            current = x
                            continue
                    else:
                        if (distance(rc[0], rc[1], cc[0], cc[1]) <= distance(rc[0], rc[1], xc[0], xc[1])) & (Robot.alarms[current] < Robot.alarms[x]):
                            pass
                        else:
                            current = x
            self.targetnode = current
            self.target = current.getCoordinates()
        #print 'current target is ' + str(self.targetnode.getId())

    def pickup(self, nd):
#        print 'def pickup'
        if len(self.carry) < SLOTS:
#            print str(nd.getId()) + ' is picked up'
            self.carry.append(nd)
            if nd in node.Node.inactiveNodes:
                node.Node.inactiveNodes.remove(nd)
            if nd in self.inactiveSensed:
                self.inactiveSensed.remove(nd)
            if nd.replacedBy:
                i = nd.replacedBy
#                print i.getId()
                if i.replacedBy == nd:
                    i.replacedBy = None
                    nd.replacedBy = None
        #update history, inactiveNodes, inactiveSensed

    def deliver(self, time):
#        print 'def deliver'
        if len(self.carry) > 0:
            nd = self.carry.pop()
#            print str(self.targetnode.getId()) + ' is being replaced by ' + str(nd.getId())
#            print 'remaining energy of target node = '+str(self.targetnode.getEnergy())
            nd.setStatus(str(self.targetnode.getStatus()))
            self.targetnode.replacedBy = nd
            nd.setHops(self.targetnode.getHops())
            nd.setCoordinates(self.targetnode.getCoordinates())
            
            for nei in self.targetnode.getNeighbours():
                nd.setNeighbours(nei)
#                print 'copy neighbour '+str(nei.getId())
            nd.setTx(self.targetnode.getTx())
            for i in nd.getNeighbours():
#                print 'neighbour '+str(i.getId())
                if nd not in i.getNeighbours():
                    i.setNeighbours(nd)
                if self.targetnode in i.getNeighbours():
                    i.getNeighbours().remove(self.targetnode)
                if i.replacedBy:
#                    print str(i.getId())+' was replaced by '+str(i.replacedBy)
                    if i in nd.getNeighbours():
                    	nd.getNeighbours().remove(i)
                    nd.setNeighbours(i.replacedBy)
            if self.targetnode.getStatus() == 'sensor':
                bs.BS.coverage += 1
            
            
            
#            print 'cons old ' + str(self.targetnode.getConsumption())
#            print 'thres old ' + str(self.targetnode.getThreshold())
#            print 'tx = '+str(self.targetnode.getTx())
#            for dep in self.targetnode.getDependents():
#                print dep.getId()
#            print 'dependents = '+str(len(self.targetnode.getDependents()))
#            for dep in self.targetnode.getNeighbours():
#                print dep.getId()
#            print
            
            node.recalculateDependents(nd)
            if self.targetnode not in node.Node.deadNodes:
                self.targetnode.setStatus(None)
                self.targetnode.setAlarm(False)
                self.targetnode.setTx(0)
                self.targetnode.setThreshold(0)
                self.targetnode.setConsumption(0)
                self.targetnode.transmissionPower()
                self.targetnode.calcConsumption()
                self.targetnode.calcEnergyThreshold()
                global LOST_ENERGY
                LOST_ENERGY = LOST_ENERGY + self.targetnode.getEnergy()
            del self.targetnode.getDependents()[:]
            for i in self.targetnode.getPath():
                nd.setPath(i)
                self.targetnode.rmPath(i)
            
#            print 'cons after ' + str(nd.getConsumption())
#            print 'thres after ' + str(nd.getThreshold())
#            print 'tx = '+str(nd.getTx())
#            for dep in nd.getDependents():
#                print dep.getId()
#            print 'dependents = '+str(len(nd.getDependents()))
#            for dep in nd.getNeighbours():
#                print dep.getId()
            
            
            #if target had died then set the recovery time
            if self.targetnode in node.Node.deadNodes:
                self.targetnode.setTimeRecover(time)
                recoverTime = open(Robot.output_path+'recoverTime.txt', 'a')
                recoverTime.write(repr(nd.getId()).ljust(4)+'up'.ljust(10)+repr(nd.getCoordinates()).ljust(18)+repr(time).ljust(8)+'\n')
                recoverTime.close()
                if self.targetnode in node.Node.sensorNodes:
                    node.Node.sensorNodes.remove(self.targetnode)
            #check the path of every sensor and replace targenode with nd
            for i in node.Node.sensorNodes:
                if self.targetnode in i.getPath():
                    i.rmPath(self.targetnode)
                    i.setPath(nd)
            if self.targetnode in Robot.alarms.keys():
                del Robot.alarms[self.targetnode]
            if self.targetnode in self.activeSensed:
                self.activeSensed.remove(self.targetnode)
            if self.targetnode in self.sensorSensed:
                self.sensorSensed.remove(self.targetnode)
            if nd not in self.activeSensed:
                self.activeSensed.append(nd)
            
            del self.targetnode.getNeighbours()[:]
            self.targetnode = None
        else:
            max = 1000
            for i in self.inactiveSensed:
                if distance(i.getCoordinates()[0], i.getCoordinates()[1], self.coordinates[0], self.coordinates[1]) < max:
                    max = distance(i.getCoordinates()[0], i.getCoordinates()[1], self.coordinates[0], self.coordinates[1])
                    self.temptarget = i
                

    def getId (self):
        return self.id

    def recharging(self):
        #check if cannot move and is at the bs then recharge
#        print 'def recharging'
        #print 'before '+str(self.energy)
        self.energy += RMAX_ENERGY/RECHARGE
        if self.energy >= RMAX_ENERGY:
            self.energy = RMAX_ENERGY
            del self.target[:]
            self.targetnode = None
            self.temptarget = None
            self.recharge = False
        #print 'after '+str(self.energy)
