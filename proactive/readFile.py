# this is the main class
# this script is part of the proactive algorithm
# by Kalypso Magklara (kalypso.magklara(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)


from __future__ import division
import re
import operator
import node
import robot
import bs
import math
import sys
import subprocess
import random
import os

MAX_TIME = 50000          #seconds


def distance(x1,y1,x2,y2):
    x = (x1-x2)**2 + (y1-y2)**2
    return math.sqrt(x)

def maptest(x):
    #removes whitespaces and keeps only the numbers
    return re.split('[^0-9]', x.strip())

def mapint(x):
    #takes a list with two strings and returns a list with two integers
    return [int(x[0]), int(x[1])]


if (len(sys.argv) < 5):
    print 'readfile.py <terrain.txt> <node_number> <sensor_percent> <scenario> <reactive/proactive>'
else:
    nn = sys.argv[2]
    sp = sys.argv[3]
    scenario = sys.argv[4]
    algo = sys.argv[5]
    
    output_path = str(algo)+'/results/'+str(nn)+'_'+str(sp)+'_'+str(scenario)+'/'
#    output_path = 'results/'+str(nn)+'_'+str(sp)+'_'+str(scenario)+'/'
    if not os.path.exists(output_path):
        os.makedirs(output_path)
    node.Node.output_path = output_path
    robot.Robot.output_path = output_path
    
    f = open(sys.argv[1])
    ######read terrain######
    terrain = f.readline()
    terrain = re.split('[^0-9]', terrain)
    while '' in terrain:
            terrain.remove('')
    terrain = map(int, terrain)
    #print terrain
    
    ######read nodes######
    nodes = re.split(':', f.readline()[:-2])
    nodes = re.split(']', nodes[1])
    for i in range(len(nodes)):
        nodes[i] = nodes[i].replace('[', '')
        nodes[i] = nodes[i].strip()
    #print len(nodes)
    n=[]
    for i in range(len(nodes)):
        temp = re.split('[^0-9]', nodes[i])
        while '' in temp:
            temp.remove('')
        temp = map(int, temp)
        n.append(temp)
    #getcount = operator.itemgetter(0)
    #map(getcount, n)
    #n=sorted(n, key = getcount)
    n=sorted(n, key = operator.itemgetter(0))
    #print n
    
    ######read cds nodes and their hop######
    cds = re.split(':', f.readline()[:-2])
    cds = re.split(']', cds[1])
    cds = map(maptest,cds)
    for i in cds:
        while '' in i:
            i.remove('')
    cds = map(mapint, cds)
    #print cds
    
    ######read sensing nodes and their hop######
    sensing = re.split(':', f.readline()[:-2])
    sensing = re.split(']', sensing[1])
    sensing = map(maptest,sensing)
    for i in sensing:
        while '' in i:
            i.remove('')
    sensing = map(mapint, sensing)
    #print sensing
    
    ######read bs station coords######
    bss = f.readline()
    bss = re.split('[^0-9]', bss)
    while '' in bss:
        bss.remove('')
    bss = map(int, bss)
    #print bss
    
    ######read graph######
    graph = re.split(':', f.readlines()[2])
    graph = re.split(',', graph[1][:-1])
    graph = map(maptest, graph)
    graph = map(mapint, graph)
    #print graph
    
    f.close()
    
    ######################## create nodes, robot, bs ###########################
    GRID = terrain[0]/10
    bs.BS.coverage = len(sensing)
    
    #create nodes
    nd = []
    for i in n:
        nd.append(node.Node(i[1]/10,i[2]/10)) #converting decimeter to meter
    #    print nd[i[0]].getId()
    #    print nd[i[0]].getCoordinates()
    
    #connect neighbours from graph
    for i in graph:
        x1 = i[0]
        x2 = i[1]
        nd[x1].setNeighbours(nd[x2])
        nd[x2].setNeighbours(nd[x1])
    
    #set the status and hopes of all cds
    for i in cds:
        y = i[0]
        nd[y].setStatus('cds')
        nd[y].setHops(i[1])
        #print nd[y].getId()
        #print nd[y].getHops()
    
    #set the status and hopes of all sensors
    for i in sensing:
        y = i[0]
        nd[y].setStatus('sensor')
        nd[y].setHops(i[1])
        #print nd[y].getId()
        #print nd[y].getHops()
        
    #create base station
    base = bs.BS(bss[0]/10, bss[1]/10)
    
    #create robot
    rob = robot.Robot(bss[0]/10, bss[1]/10)
    
    #calculate how many sensors depend from each cds
    for i in sensing:
        node.calculateDependents(nd[i[0]],'add')
    
    #set transmission power for each node depending on the distance from the receiver
    for i in node.Node.sensorNodes:
        i.transmissionPower()
    for i in node.Node.cdsNodes:
        i.transmissionPower()
        
    #set consumption for all nodes
    for i in nd[1:]:
        i.calcConsumption()
        i.setMaxConsumption(i.getConsumption())
#        print str(i.getId()) + ' cons is '+ str(i.getConsumption())
#        print i.getMaxConsumption()
        
    #set energy threshold for cds and sensor nodes
    for i in node.Node.sensorNodes:
        i.calcEnergyThreshold()
    for i in node.Node.cdsNodes:
        i.calcEnergyThreshold()
    
    covered_points = []
    cp_downtime = []
    for i in node.Node.sensorNodes:
        covered_points.append(i.getCoordinates())
        cp_downtime.append(0)
        
    for i in sensing:
        node.calcPath(nd[i[0]], nd[i[0]])
#        print str(nd[i[0]].getId())+ " path length " + str(nd[i[0]].getPath()) + ' hops '+ str(nd[i[0]].getHops())
    
    #open file for statistics
    robotEnergy = open(output_path+'robotEnergy.txt', 'w')
    towrite = '#time'.ljust(8)+'energy'.ljust(8)+'\n'
    robotEnergy.write(towrite)
    recoverTime = open(output_path+'recoverTime.txt', 'w')
    recoverTime.write('#id'.ljust(4)+'down/up'.ljust(10)+'coordinates'.ljust(18)+'time'.ljust(8)+str(len(nd))+'\n')
    recoverTime.write('#max_time '+str(MAX_TIME)+'\n')
    ctowrite = "# "
    for i in nd:
        ctowrite += str(i.getCoordinates())
    recoverTime.write(str(ctowrite)+'\n')
    recoverTime.close()
#    print '________________________________________________________________'
#    print '#------------------------START SIMULATION------------------------'
#    print '________________________________________________________________'
    ######################## start simulation ###########################
    covered_distance = robot.Robot.speed
    for time in range(1,MAX_TIME):
#        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
#        print '                 time: %04d'%time
#        print '~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~'
        for j in nd[1:]:
            j.updateEnergy(time) 
#            if ((j.getStatus() == 'sensor') | (j.getStatus() == 'cds'))&(j not in node.Node.deadNodes):
#                print '                               '+str(j.getId()) + ' energy level is ' +   str(j.getEnergy())
#                print 'consumption = '+str(j.getConsumption())
#                print 'threshold = '+str(j.getThreshold())
#                print 'tx = '+str(j.getTx())
#                print 'dependents = '+str(len(j.getDependents()))
        
        if (rob.gettemptarget() != None) | (rob.gettargetnode() != None):
#            print #'do not change your target'
            pass
        elif (len(rob.getTarget()) == 0) | ((rob.getCoordinates() == rob.getTarget()) & (rob.getTarget() != list(base.getCoordinates()))):
            #print 'no target or already there, pick new'
            coordx = random.uniform(0.0,GRID)
            coordy = random.uniform(0.0,GRID)
            rob.setTarget(coordx, coordy)
        #print 'target is ' + str(rob.getTarget())
        rob.checkEnergy()
        covered_distance = rob.updatePosition(time)
#        if rob.gettemptarget() != None:
#            print 'temp coords ' + str(rob.gettemptarget().getCoordinates())
#        if rob.gettargetnode() != None:
#            print 'target coords '+ str(rob.gettargetnode().getCoordinates())
        if rob.getRecharge() == False:
            rob.updateEnergy(covered_distance)
        #print rob.getEnergy()
        
        #write time and robot energy to file
#        print 'robot coordinates are '+str(rob.getCoordinates())
        towrite = repr(time).ljust(8)+repr(rob.getEnergy()).ljust(8)+'\n'
        robotEnergy.write(towrite)
        
        #calculate if any sensors are disconnected then add to their timeDown and write to file
        for i in node.Node.sensorNodes:
            enum = 0
            if i in node.Node.deadNodes:
                enum += 1
            else:
                for j in i.getPath():
                    if j in node.Node.deadNodes:
                        enum += 1
                        #print j.getId()
            if enum > 0:
                for k in range(len(covered_points)):
                    if covered_points[k] == i.getCoordinates():
                        cp_downtime[k] += 1
                    
        
        #############################################################################
        #will print coordinates of nodes and robot to a file and import them to perl#
        #     to get an image of the network and the robot's moves every second     #
        #############################################################################
        nodePosition = open(output_path+'nodes.txt', 'w')
        robotPosition = open(output_path+'robot.txt', 'a')
        #first print id and coordinate of inactive nodes in nodePosition
        nodePosition.write('# inactive nodes: ')
        for i in node.Node.inactiveNodes:
            nodePosition.write(str(i.getId())+' ['+str(int(i.getCoordinates()[0]*10))+' ' +str(int(i.getCoordinates()[1]*10))+'] ')
        nodePosition.write('\n')
        #second print id and coordinate of sensing nodes in nodePosition
        nodePosition.write('# sensing nodes: ')
        for i in node.Node.sensorNodes:
            nodePosition.write(str(i.getId())+' ['+str(int(i.getCoordinates()[0]*10))+' ' +str(int(i.getCoordinates()[1]*10))+'] ')
        nodePosition.write('\n')
        #then print id and coordinate of cds nodes in nodePosition
        nodePosition.write('# cds nodes: ')
        for i in node.Node.cdsNodes:
            nodePosition.write(str(i.getId())+' ['+str(int(i.getCoordinates()[0]*10))+' ' +str(int(i.getCoordinates()[1]*10))+'] ')
        nodePosition.write('\n')
        nodePosition.write('# robot position:')
        #append time and robot coordinates in robotPosition
        robotPosition.write(' '+str(time)+' ['+str(int(rob.getCoordinates()[0]*10))+' ' +str(int(rob.getCoordinates()[1]*10))+'] ')
        robotPosition.close()
        #reopen robot file for reading and copy its contents in nodePosition
        robotPosition = open(output_path+'robot.txt', 'r')
        nodePosition.write(robotPosition.readline())
        nodePosition.close()
        robotPosition.close()
        #use perl script to read nodePosition and create image
        if (time % 500 == 0) | (time == 1):
              tmp_out = output_path+'nodes.txt'
              subprocess.call(["../draw_instance.pl", str(sys.argv[1]),  tmp_out])
        #remove robot movement
        if time % 500 == 0:
             robotPosition = open(output_path+'robot.txt', 'w')
             robotPosition.close()
    robotEnergy.close()
    
    print str('#id').ljust(8),
    print str('disconnection time').ljust(10)
    for i in range(len(covered_points)):
        print str(i+1).ljust(8),
        print str(cp_downtime[i]).ljust(10)
    
    lst_enrg = open(output_path+'lostEnergy.txt', 'w')
    towrite = 'Energy lost = '+str(robot.LOST_ENERGY)
    lst_enrg.write(towrite)
    lst_enrg.close()
