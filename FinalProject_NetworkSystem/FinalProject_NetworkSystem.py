from vpython import * 
import math
import numpy as np
import copy
import random
import matplotlib.pyplot as plt
import queue
from enum import Enum

# class syntax
class EventType(Enum):
    PacketSpawn = 1
    RouterToLink = 2
    LinkToRouter = 3
    TransmissionDone = 4
    ProcDone = 5

phpd =0.002
maxQSize=6

#-----Classes-----
#Packet
class Packet:
    def __init__(self, t, s, d, p, b):
        self.timestamp = t #time source sent it at
        self.Source = s #address of source
        self.Destination = d # address of destination
        self.Payload = p
        self.NumberOfBits = b
        self.To = d
        self.From = s

    def getTo(self):
        return self.To
    def getFrom(self):
        return self.From
    def getDestination(self):
        return self.Destination
    def getPayload(self):
        return self.Payload

    def setTo(self, t):
        self.To = t
    def setFrom(self, f):
        self.From = f

    def getNumberOfBits(self):
        return self.NumberOfBits
    def getWireNum(self):
        return self.inputNum

class Event:
    def __init__(self, t, p, r,Etype):
        self.time = t
        self.packet = p
        self.toNext = r
        self.EventType = Etype

    def getTime(self):
        return self.time

    def MovePacket(self):
        if (self.EventType == EventType.PacketSpawn):
            return self.toNext.SpawnPacket(self.packet, self.time)
        elif (self.EventType == EventType.RouterToLink):
            return self.toNext.GoThroughLink(self.packet, self.time, False)
        elif (self.EventType == EventType.ProcDone ):
            return self.toNext.ProcIsDone(self.packet, self.time)
        elif (self.EventType == EventType.TransmissionDone):
            return self.toNext.GoThroughLink(self.packet, self.time, True)
        elif (self.EventType == EventType.LinkToRouter ):
            return self.toNext.AddPacketToQueue(self.packet, self.time)

        return Event(-1, 0, 0, EventType.RouterToLink)
        

    # overload < operator
    def __lt__(self, other):
        return self.time < other.time
    # overload <= operator
    def __le__(self, other):
        return self.time <= other.time
    # overload == operator
    def __eq__(self, other):
        return self.time == other.time
    # overload != operator
    def __ne__(self, other):
        return self.time != other.time
    # overload > operator
    def __gt__(self, other):
        return self.time > other.time
    # overload >= operator
    def __ge__(self, other):
        return self.time >= other.time

#Router/EndDevice
class Router:
    def __init__(self, ed, a, pos):
        self.IsEndDevice = ed
        self.address = a #it's address
        self.PacketQ= queue.Queue(maxQSize)
        self.ProcPacketQ= queue.Queue(maxQSize)
        self.Links = []
        self.pos = pos
        self.proccessingDelay = phpd
        self.visual = sphere(pos=pos, radius=0.5, color=color.purple)
        self.PacketsReachedDest = []
        self.isProccessing = False

        self.arrived = 0
        self.serviced = 0
        self.arrivalRate = 0
        self.serviceRate = 0

    def PacketsReachedDestLen(self):
        return len(self.PacketsReachedDest)

    def GetQDelay(self):
        return 1/(self.serviceRate - self.arrivalRate)
    def GetPos(self):
        return self.pos
    def getAddress(self):
        return self.address
    def popOffFromQ(self):
        if not self.PacketQ.empty():
            return self.PacketQ.get()
        else:
            #print("nothing in q")
            return 0
    def popOffFromProcQ(self):
        if not self.ProcPacketQ.empty():
            return self.ProcPacketQ.get()
        else:
            #print("nothing in proc q")
            return 0

    def isQEmpty(self):
        return self.PacketQ.empty()
    def isProcQEmpty(self):
        return self.ProcPacketQ.empty()

    def AddLinks(self, l):
        self.Links = l

    def SpawnPacket(self, p, time):
        self.ProcPacketQ.put(p)
        nextL = self.determineNextRouter()
        if(nextL.getIsBusy()==True):
            return Event(-1, 0, 0, EventType.RouterToLink)
        else:
            return Event(time, p, nextL, EventType.RouterToLink)

    def AddPacketToQueue(self, p, time):
        l = self.getPastLink(p)
        l.setIsBusy(False)

        if(p.getDestination()==self.address):
            print(self.address +"  recieved at time  "+ str(time)+ "   says: " + p.getPayload())
            self.PacketsReachedDest.append(time)
            return Event(-1, 0, 0, EventType.RouterToLink)
        else:
            print(self.address +"   "+ str(time)+ "   says: " + p.getPayload())
            if(self.PacketQ.qsize()>=maxQSize):
                print("--Packet Lost: "+p.getPayload())
                return Event(-1, 0, 0, EventType.RouterToLink) #packet loss as queue is overrun
            else:
                self.arrived = self.arrived+1
                self.arrivalRate = self.arrived / time
                self.PacketQ.put(p)
            nextL = self.determineNextRouter()
            if self.isProccessing:
                return Event(-1, 0, 0, EventType.RouterToLink)
            self.isProccessing = True
            return Event(time+self.proccessingDelay, p, self, EventType.ProcDone)

    def ProcIsDone(self, p, time):
        print(self.address +"   "+ str(time)+ "   finished Proc: " + p.getPayload())
        self.isProccessing = False
        nextP = self.popOffFromQ()

        if isinstance(nextP, int):
            return Event(-1, 0, 0, EventType.RouterToLink)

        self.ProcPacketQ.put(nextP)
        nextL = self.determineNextRouter()
        e1 = Event(time, nextP, nextL, EventType.RouterToLink)

        followingP = self.popOffFromQ()

        self.serviced = self.serviced+1
        self.serviceRate = self.serviced / (time+self.proccessingDelay)

        if isinstance(followingP, Packet):
            self.PacketQ.put(followingP)
            self.isProccessing = True
            print("   "+self.address +"   "+ str(time)+ "   up next: " + followingP.getPayload())
            return (e1, Event(time+self.proccessingDelay, followingP, self, EventType.ProcDone))
        else:
            return (e1, Event(time+self.proccessingDelay, p, self, EventType.ProcDone))
        
        return e1

    def getPastLink(self, p):
        l = [x 
             for x in self.Links 
             if x.getToAddr(self.address).getAddress() == p.getFrom()]
        return l[0]

    def determineNextRouter(self):
        if self.IsEndDevice :
            return self.Links[0]
        else:
            return self.Links[1]
            #choice = [x for x in Links if x.getToAddr(self.address) ]

#Link
class Link:
    def __init__(self, l, v, b):
        self.Length = l 
        self.V_Speed = v
        self.BitRate = b
        self.visual = cylinder(pos=vector(0,0,0), axis=vector(0,0,1), radius=.125, color=color.blue)
        self.Routers = [] 
        self.isBusy=False

    def getIsBusy(self):
        return self.isBusy
    def setIsBusy(self, b):
        self.isBusy = b

    def GoThroughLink(self, p, time, busy):
        if busy:
            print("  Transmission Ended")
            self.isBusy = False
        #print("  Line is Busy:" + str(busy))
        if(self.isBusy):
            return Event(-1, 0, 0, EventType.RouterToLink)
        self.isBusy=True
        (nextTime1, transmTime1) = self.calcDelays(p)
        nextTime = time + nextTime1
        transmTime = time + transmTime1

        fromAdd = p.getFrom()
        nextR = self.getToAddr(fromAdd)
        prevR = self.getToAddr(nextR.getAddress())
        pack = prevR.popOffFromProcQ()
        if isinstance(pack, int):
            return Event(-1, 0, 0, EventType.RouterToLink)
        pack.setTo(nextR.getAddress())
        pack.setFrom(prevR.getAddress())
        e1 =  Event(nextTime, pack, nextR, EventType.LinkToRouter)
        #if not nextR.isQEmpty():
        e2 =  Event(transmTime, pack, self, EventType.TransmissionDone)
        print(prevR.address +"  sending at time  "+ str(time)+ "   says: " + pack.getPayload())
        return (e1, e2)
        return e1

    def calcDelays(self, p):
        time = (self.Length/self.V_Speed) + (p.getNumberOfBits()/self.BitRate)
        transmTime = (p.getNumberOfBits()/self.BitRate)
        return (time,transmTime)

    def getToAddr(self, fromA):
        if(fromA==self.Routers[0].getAddress() ):
            return self.Routers[1]
        else:
            return self.Routers[0]

    def AddRouters(self, r, l):
        self.Routers = [r,l]
        self.visual.pos = r.GetPos()
        self.visual.axis = l.GetPos()-r.GetPos()


#----constants----
L = 50000     #meters (half of max)
V = 204190477 #m/s   - 5 microsec per kilometer
B = 15000000  #bits per sec (15 Mb)
bits = 1024

#-----Network Setup------
Routers={"4004":Router(True, "4004", vector(-4,-1,0)),  "f11c":Router(False, "f11c", vector(-2,0,0)),
         "cead":Router(False, "cead", vector(0,0,0)), "1297":Router(False, "1297", vector(2,0,0)),
         "e17d":Router(True, "e17d", vector(4,0,0)), "f88f":Router(True, "f88f", vector(-4,1,0))}

Links=[Link(L,V,B), Link(L,V,B), Link(L,V,B), Link(L,V,B), Link(L,V,B)]

Routers["4004"].AddLinks([Links[0]])
Routers["f11c"].AddLinks([Links[0], Links[1], Links[4]])
Routers["cead"].AddLinks([Links[1], Links[2]])
Routers["1297"].AddLinks([Links[2], Links[3]])
Routers["e17d"].AddLinks([Links[3]])
Routers["f88f"].AddLinks([Links[4]])

Links[0].AddRouters(Routers["4004"], Routers["f11c"])
Links[1].AddRouters(Routers["cead"], Routers["f11c"])
Links[2].AddRouters(Routers["1297"], Routers["cead"])
Links[3].AddRouters(Routers["e17d"], Routers["1297"])
Links[4].AddRouters(Routers["f88f"], Routers["f11c"])

#------Event Setup---------
EventQ = queue.PriorityQueue()

e = Event(0, Packet(0, "4004", "e17d", "1: Hello There", bits), Routers["4004"], EventType.PacketSpawn)
EventQ.put(e)
e = Event(0.0000000001, Packet(0, "4004", "e17d", "2: Hello There", bits), Routers["4004"], EventType.PacketSpawn)
EventQ.put(e)
e = Event(0.0000000002, Packet(0, "4004", "e17d", "3: Hello There", bits), Routers["4004"], EventType.PacketSpawn)
EventQ.put(e)
#e = Event(0.0000000003, Packet(0, "4004", "e17d", "4: Hello There", bits), Routers["4004"], EventType.PacketSpawn)
#EventQ.put(e)
#e = Event(0, Packet(0, "f88f", "e17d", "5: Hello There", bits), Routers["f88f"], EventType.PacketSpawn)
#EventQ.put(e)
#e = Event(0.0000000001, Packet(0, "f88f", "e17d", "6: Hello There", bits), Routers["f88f"], EventType.PacketSpawn)
#EventQ.put(e)
#e = Event(0.0000000002, Packet(0, "f88f", "e17d", "7: Hello There", bits), Routers["f88f"], EventType.PacketSpawn)
#EventQ.put(e)
#e = Event(0.0000000003, Packet(0, "f88f", "e17d", "8: Hello There", bits), Routers["f88f"], EventType.PacketSpawn)
#EventQ.put(e)

Nm = EventQ.qsize()

while(not EventQ.empty()):
    rate(90)
    currEvent = EventQ.get()
    nextEvent = currEvent.MovePacket()
    if  isinstance(nextEvent, tuple):
        EventQ.put(nextEvent[0])
        EventQ.put(nextEvent[1])
    elif nextEvent.getTime() != -1:
        EventQ.put(nextEvent)
    print("yo")

nh=len(Links)

TotDelay = (nh*(L/V)) + (Nm*(bits/B))+ (nh-1)*((bits/B)+phpd)

print("Calc Total delay:" + str(TotDelay))
packRec = Routers["e17d"].PacketsReachedDestLen()/Nm
qD = Routers["f11c"].GetQDelay()
print("Queuing delay:" + str(qD))
print("Received Packet:  "+str(packRec)+"%")
print("done")