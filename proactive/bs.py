# this class describes the base station and it is used by readFile.py
# this script is part of the proactive algorithm
# by Kalypso Magklara (kalypso.magklara(at)inria.fr)
#
# Distributed under the GPLv3 (see LICENSE file)

__metaclass__ = type    #new style classes, default after version 3

RECHARGE = 14400        #min

class BS:
    bsid = -1
    coverage = 1
    coordinates = []

    def __init__(self, coordx = -1, coordy = -1):
        self.coord = (coordx, coordy)
        BS.coordinates = list(self.coord)
        BS.bsid += 1
        self.id = BS.bsid

    def getCoordinates(self):
        return self.coord

    def getId (self):
        return self.id
    
    def setCoverage(self, cov):
        BS.coverage = cov
