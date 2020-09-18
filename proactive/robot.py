# this class describes the robot and it is used by readFile.py
# this script is part of the proactive algorithm
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
k = 5                   #energy of inactive is more than k times the energy of active node

LOST_ENERGY = 0

def quadratic(x1, y1, x2, y2, dist):
    '''y = ax + b so will solve it using the sympy module
    >>> system = Matrix(( (1, 4, 2), (-2, 1, 14)))
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
        self.activeSensed = {}      #id = (ttd,energy,consumption)
        self.inactiveSensed = {}    #id = (ttd,energy,consumption)
        self.sensorSensed = {}      #id = (ttd,energy,consumption)
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
        #print 'def setTarget'
        if node:
            self.targetnode = node
            self.target = node.getCoordinates()
        else:
            self.target = [coordx, coordy]
            self.targetnode = None
            
    def gettargetnode(self):
        return self.targetnode
    
    def gettemptarget(self):
        return self.temptarget
    
    def sense(self):
        #print 'def sense'
        '''iterate dictionary of coordinates of all nodes
        if euclidean distance between robot and node < range then node is sensed'''
        current = self.coordinates
        for x in node.Node.cdsNodes :
            if x.getId() !=0 :
                trgt = x.getCoordinates()
                if (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (self.activeSensed.get(x)==None):
                    self.activeSensed[x] = (x.getEnergy()/x.getMaxConsumption(), x.getEnergy(), x.getMaxConsumption())
                    #print 'sensing active ' + str(x.getId())
                elif (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (self.activeSensed.get(x)!=None):
                    #print 'sensing active ' + str(x.getId())
                    del self.activeSensed[x]
                    self.activeSensed[x] = (x.getEnergy()/x.getMaxConsumption(), x.getEnergy(), x.getMaxConsumption())
        for x in node.Node.inactiveNodes:
            trgt = x.getCoordinates()
            #print 'trgt' + str(trgt)
            if (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (self.inactiveSensed.get(x)==None):
                self.inactiveSensed[x] = (x.getEnergy()/x.getConsumption(), x.getEnergy(), x.getConsumption())
                #print 'sensing inactive ' + str(x.getId())
            elif (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (self.inactiveSensed.get(x)!=None):
                #print 'sensing inactive ' + str(x.getId())
                del self.inactiveSensed[x]
                self.inactiveSensed[x] = (x.getEnergy()/x.getConsumption(), x.getEnergy(), x.getConsumption())
        for x in node.Node.sensorNodes:
            trgt = x.getCoordinates()
            #print 'trgt' + str(trgt)
            if (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (self.activeSensed.get(x)==None):
                self.activeSensed[x] = (x.getEnergy()/x.getMaxConsumption(), x.getEnergy(), x.getMaxConsumption())
                #print 'sensing active ' + str(x.getId())
            elif (distance(current[0], current[1], trgt[0], trgt[1]) < self.range) & (self.activeSensed.get(x)!=None):
                #print 'sensing active ' + str(x.getId())
                del self.activeSensed[x]
                self.activeSensed[x] = (x.getEnergy()/x.getMaxConsumption(), x.getEnergy(), x.getMaxConsumption())

    def updatePosition(self,time):
        '''will use a steady speed and move maximum SPEED meters
        direction is from (begin[0], begin[1]) to (end[0], end[1])'''
        #print 'def updatePosition'
        if self.recharge == False:
            self.sense()
            if self.target == list(bs.BS.coordinates):
                #print 'target is the bs'
                self.temptarget = None
                if self.coordinates == list(bs.BS.coordinates):
                    self.recharge = True
                    self.recharging()
                    self.updateSensedDictionaties()
                    return 0
                else:
                    begin = self.coordinates
                    end = self.target
                    self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                    if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                        self.updateSensedDictionaties()
                        return distance(begin[0], begin[1], end[0], end[1])
                    else:
                        self.updateSensedDictionaties()
                        return SPEED
            elif self.temptarget:
                #print 'there is a temptarget'
                if self.coordinates == self.temptarget.getCoordinates():
                    #print 'will pickup here'
                    self.pickup(self.temptarget)
                    self.temptarget = None
                    self.updateSensedDictionaties()
                    #self.target = self.targetnode.getCoordinates()
                    return 0
                else:
                    begin = self.coordinates
                    end = self.temptarget.getCoordinates()
                    self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                    if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                        self.updateSensedDictionaties()
                        return distance(begin[0], begin[1], end[0], end[1])
                    else:
                        self.updateSensedDictionaties()
                        return SPEED
            elif self.targetnode:
                #print 'there is a targetnode'
                if (self.coordinates == self.target):
                    #print 'will deliver here'
                    self.deliver(time)
                    self.updateSensedDictionaties()
                    return 0
                else:
                    begin = self.coordinates
                    end = self.target
                    self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                    if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                        self.updateSensedDictionaties()
                        return distance(begin[0], begin[1], end[0], end[1])
                    else:
                        self.updateSensedDictionaties()
                        return SPEED
            else:
                #print 'there is no target'
                '''from the active sensed nodes pick the one with min ttd
                then check its threshold and then every inactive who has more than k times the 
                active node's energy and replace'''
                min_ttd = 10000000000
                for i in self.activeSensed.keys():
                    #print 'checking active node with id '+ str(i.getId()) + ' ttd= '+str(self.activeSensed[i][0])
                    if self.activeSensed[i][0] < min_ttd:
                        min_ttd = self.activeSensed[i][0]
                        #print 'ttd is smaller than min '+str(i.getId()) + ' has energy and threshold:'
                        #print i.getEnergy()
                        #print i.getThreshold()
                        if i.getEnergy() < i.getThreshold():
#                            print 'energy is smaller than threshold must replace'
                            self.targetnode = i
                            #print self.targetnode
                            self.target = i.getCoordinates()
                            min_dist = self.range*5
                            for j in self.inactiveSensed.keys():
                                if distance(j.getCoordinates()[0], j.getCoordinates()[1], i.getCoordinates()[0], i.getCoordinates()[1]) < min_dist:
                                    min_dist = distance(j.getCoordinates()[0], j.getCoordinates()[1], i.getCoordinates()[0], i.getCoordinates()[1]) 
                                    self.temptarget = j
                        else:
                            if len(self.carry) < SLOTS:
                                #print 'energy was good but will check inactives'
                                for j in self.inactiveSensed.keys():
                                    #print 'checking inactive node with id '+ str(j.getId()) + ' ttd=' + str(self.activeSensed[i][0])
                                    if distance(j.getCoordinates()[0], j.getCoordinates()[1], i.getCoordinates()[0], i.getCoordinates()[1]) < MAX_PICKUP_RANGE:
                                        #print 'inactive ' + str(j.getId())+ ' energy: '+str(self.inactiveSensed[j][1]) + ' ttd: ' + str(self.inactiveSensed[j][0])
                                        #print 'active ' + str(i.getId())+ ' energy: '+str(self.activeSensed[i][1]) + ' ttd: ' + str(self.activeSensed[i][0])
                                        if self.inactiveSensed[j][1] > k * self.activeSensed[i][1]:
#                                            print 'setting new target because '+ str(self.inactiveSensed[j][1])+ ' is bigger than '+ str(k * self.activeSensed[i][1])
                                            self.temptarget = j
                                            self.targetnode = i
                                            self.target = i.getCoordinates()
                begin = self.coordinates
                end = self.target
                self.coordinates = list(quadratic(begin[0], begin[1], end[0], end[1], SPEED))
                if distance(begin[0], begin[1], end[0], end[1]) < SPEED:
                    self.updateSensedDictionaties()
                    return distance(begin[0], begin[1], end[0], end[1])
                else:
                    self.updateSensedDictionaties()
                    return SPEED
        else:
            self.recharging()
            self.updateSensedDictionaties()
            return 0
    
    def updateSensedDictionaties(self):
        #print 'reduce ttd and energy according to stored consumption'
        #print 'active'
        for i in self.activeSensed.keys():
            #print i.getId()
            #print 'real numbers ' + str(i.getConsumption()) + ' consum, ' + str(i.getEnergy())+' energy'
            #print 'stored numbers '+str(self.activeSensed[i][2])+ ' consum, '+ str(self.activeSensed[i][1])+' energy ' + str(self.activeSensed[i][0]) + ' ttd'
            tmp_ttd = self.activeSensed[i][0] - 1
            tmp_en = self.activeSensed[i][1] - self.activeSensed[i][2]
            temp_cons = self.activeSensed[i][2]
            del self.activeSensed[i]
            self.activeSensed[i] = (tmp_ttd, tmp_en, temp_cons)
            #print 'after reduction'
            #print 'stored numbers '+str(self.activeSensed[i][2])+ ' consum, '+ str(self.activeSensed[i][1])+' energy ' + str(self.activeSensed[i][0]) + ' ttd'
        #print 'inactive'
        for j in self.inactiveSensed.keys():
            #print j.getId()
            #print 'real numbers ' + str(j.getConsumption()) + ' consum, ' + str(j.getEnergy())+' energy'
            #print 'stored numbers '+str(self.inactiveSensed[j][2])+ ' consum, '+ str(self.inactiveSensed[j][1])+' energy ' + str(self.inactiveSensed[j][0]) + ' ttd'
            tmp_ttd = self.inactiveSensed[j][0] - 1
            tmp_en = self.inactiveSensed[j][1] - self.inactiveSensed[j][2]
            temp_cons = self.inactiveSensed[j][2]
            del self.inactiveSensed[j]
            self.inactiveSensed[j] = (tmp_ttd, tmp_en, temp_cons)
            #print 'after reduction'
            #print 'stored numbers '+str(self.inactiveSensed[j][2])+ ' consum, '+ str(self.inactiveSensed[j][1])+' energy ' + str(self.inactiveSensed[j][0]) + ' ttd'

    def updateEnergy(self, dist):
        #print 'def updateEnergy'
        if dist > 0 :
            self.energy -= (CONSUMPTION * dist) / 0.9
        else:
            self.energy -= IDLE

    def checkEnergy(self):
        #print 'def checkEnergy'
        current = self.coordinates
        trgt = self.target
        base = bs.BS.coordinates
        needed = CONSUMPTION * (distance(current[0], current[1], trgt[0], trgt[1]) + distance(trgt[0], trgt[1], base[0], base[1]) +(2* MAX_PICKUP_RANGE))
        if self.energy <= needed:
            #print 'not enough energy'
            self.target = list(bs.BS.coordinates)
            self.targetnode = None

    def pickup(self, nd):
        #print 'def pickup'
        if len(self.carry) < SLOTS:
            #print str(nd.getId()) + ' is picked up'
            self.carry.append(nd)
            if nd in node.Node.inactiveNodes:
                node.Node.inactiveNodes.remove(nd)
            if self.inactiveSensed.get(nd):
                del self.inactiveSensed[nd]
            if nd.replacedBy:
                i = nd.replacedBy
#                print i.getId()
                if i.replacedBy == nd:
                    i.replacedBy = None
                    nd.replacedBy = None
        #update history, inactiveNodes, inactiveSensed

    def deliver(self, time):
        #print 'def deliver'
        if len(self.carry) > 0:
            nd = self.carry.pop()
#            print str(self.targetnode.getId()) + ' is being replaced by ' + str(nd.getId())
            #print self.targetnode.getStatus()
            nd.setStatus(self.targetnode.getStatus())
            self.targetnode.replacedBy = nd
            nd.setHops(self.targetnode.getHops())
            nd.setCoordinates(self.targetnode.getCoordinates())
            
            for nei in self.targetnode.getNeighbours():
                nd.setNeighbours(nei)
#                print 'copy neighbour '+str(nei.getId())
            nd.setTx(self.targetnode.getTx())
            for i in nd.getNeighbours():
                #print i.getId()
                if nd not in i.getNeighbours():
                    i.setNeighbours(nd)
                if self.targetnode in i.getNeighbours():
                    i.getNeighbours().remove(self.targetnode)
                if i.replacedBy:
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
                if self.targetnode.getEnergy() < self.targetnode.getThreshold():
                    self.targetnode.setStatus(None)
                    global LOST_ENERGY
                    LOST_ENERGY = LOST_ENERGY + self.targetnode.getEnergy()
                else:
                    self.targetnode.setStatus('inactive')
                self.targetnode.setAlarm(False)
                self.targetnode.setTx(0)
                self.targetnode.setThreshold(0)
                self.targetnode.setConsumption(0)
                self.targetnode.transmissionPower()
                self.targetnode.calcConsumption()
                self.targetnode.calcEnergyThreshold()
            del self.targetnode.getDependents()[:]
            for i in self.targetnode.getPath():
                nd.setPath(i)
                self.targetnode.rmPath(i)
            nd.setMaxConsumption(self.targetnode.getMaxConsumption())
            
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
            if self.activeSensed.get(self.targetnode):
                del self.activeSensed[self.targetnode]
            if self.sensorSensed.get(self.targetnode):
                del self.sensorSensed[self.targetnode]
            if self.activeSensed.get(nd) == None:
                self.activeSensed[nd] = (nd.getEnergy()/nd.getMaxConsumption(), nd.getEnergy(), nd.getMaxConsumption())
            else:
                del self.activeSensed[nd]
                self.activeSensed[nd] = (nd.getEnergy()/nd.getMaxConsumption(), nd.getEnergy(), nd.getMaxConsumption())
            self.targetnode = None
        else:
            max = MAX_PICKUP_RANGE
            for i in self.inactiveSensed.keys():
                if distance(i.getCoordinates()[0], i.getCoordinates()[1], self.coordinates[0], self.coordinates[1]) < max:
                    max = distance(i.getCoordinates()[0], i.getCoordinates()[1], self.coordinates[0], self.coordinates[1])
                    self.temptarget = i
                

    def getId (self):
        return self.id

    def recharging(self):
        #check if cannot move and is at the bs then recharge
        #print 'def recharging'
        self.energy += RMAX_ENERGY/RECHARGE
        if self.energy >= RMAX_ENERGY:
            self.energy = RMAX_ENERGY
            del self.target[:]
            self.recharge = False
