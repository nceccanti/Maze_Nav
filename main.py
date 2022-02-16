import os.path
import sys

from controller import Robot, Motor
import math

TIME_STEP = 64
robot = Robot()
print("Starting")
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
rightMotor.setVelocity(0.5)
leftMotor.setVelocity(0.5)
compass = robot.getDevice("compass")
compass.enable(TIME_STEP)
leftE = robot.getDevice("left wheel sensor")
leftE.enable(TIME_STEP)
rightE = robot.getDevice("right wheel sensor")
rightE.enable(TIME_STEP)
oneDist = robot.getDevice("ps1")
threeDist = robot.getDevice("ps2")
fiveDist = robot.getDevice("ps3")
oneDist.enable(TIME_STEP)
threeDist.enable(TIME_STEP)
fiveDist.enable(TIME_STEP)
frontLeft = robot.getDevice("ps0")
frontRight = robot.getDevice("ps7")
frontLeft.enable(TIME_STEP)
frontRight.enable(TIME_STEP)
sevenDist = robot.getDevice("ps4")
nineDist = robot.getDevice("ps5")
elevenDist = robot.getDevice("ps6")
sevenDist.enable(TIME_STEP)
nineDist.enable(TIME_STEP)
elevenDist.enable(TIME_STEP)
answer = compass.getValues()
angle = math.degrees(math.atan2(answer[0], answer[1]))
touch = robot.getDevice("touch sensor")
touch.enable(TIME_STEP)

class State(object):
    def __init__(self, FSM):
        self.FSM = FSM

    def Execute(self):
        pass

    def Exit(self):
        pass

class ForwardRight(State):
    def __init__(self, FSM):
        super(ForwardRight, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        one = oneDist.getValue()
        three = threeDist.getValue()
        five = fiveDist.getValue()

        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        if one > 200:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(0.5)
        elif angle % 90 > 42.5 or three > 400:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(0.8)
        elif angle % 90 < 42.5:
            rightMotor.setVelocity(0.8)
            leftMotor.setVelocity(1.0)
        else:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(1.0)
        if left > 200 and right > 200:
            self.FSM.direction = angle
            self.FSM.ToTransition("toDeadEndRight")
        if three < 150 and five < 150 and one < 150:
            self.FSM.ToTransition("toRight")


    def Exit(self):
        #print("out of forward mode")
        pass

class ForwardLeft(State):
    def __init__(self, FSM):
        super(ForwardLeft, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        seven = sevenDist.getValue()
        nine = nineDist.getValue()
        eleven = elevenDist.getValue()

        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        if eleven > 200:
            rightMotor.setVelocity(0.5)
            leftMotor.setVelocity(1.0)
        if angle % 90 > 42.5:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(0.8)
        elif angle % 90 < 42.5 or nine > 400:
            rightMotor.setVelocity(0.8)
            leftMotor.setVelocity(1.0)
        else:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(1.0)
        if left > 200 and right > 200:
            self.FSM.direction = angle
            self.FSM.ToTransition("toDeadEndLeft")
        if seven < 150 and nine < 150 and eleven < 150:
            self.FSM.ToTransition("toLeft")


    def Exit(self):
        #print("out of forward mode")
        pass

class LeftTurn(State):
    def __init__(self, FSM):
        super(LeftTurn, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        seven = sevenDist.getValue()
        nine = nineDist.getValue()
        eleven = elevenDist.getValue()

        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        rightMotor.setVelocity(1.0)
        leftMotor.setVelocity(0.0)
        setDir = self.FSM.direction + 90
        if setDir > 180:
            setDir -= 360
        if abs(setDir - angle) < 20 and abs(self.FSM.direction - angle) > 10:
            self.FSM.direction = setDir
            self.FSM.isTurn(self.FSM.getCard(angle))
            self.FSM.ToTransition("toForwardLeft")
        if seven > 200 or nine > 200 or eleven > 200:
            self.FSM.ToTransition("toForwardLeft")
        if left > 200 and right > 200:
            self.FSM.direction = setDir
            self.FSM.ToTransition("toDeadEndLeft")

    def Exit(self):
        #print("Out of left mode")
        pass

class RightTurn(State):
    def __init__(self, FSM):
        super(RightTurn, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        one = oneDist.getValue()
        three = threeDist.getValue()
        five = fiveDist.getValue()

        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        rightMotor.setVelocity(0.0)
        leftMotor.setVelocity(1.0)
        setDir = self.FSM.direction - 90
        if setDir < -180:
            setDir += 360
        #print(setDir, angle, left, right)
        if abs(setDir - angle) < 20 and abs(self.FSM.direction - angle) > 10:
            self.FSM.direction = setDir
            self.FSM.isTurn(self.FSM.getCard(angle))
            self.FSM.ToTransition("toForwardRight")
        if one > 200 or three > 200 or five > 200:
            self.FSM.ToTransition("toForwardRight")
        if left > 300 or right > 300:
            self.FSM.direction = setDir
            self.FSM.ToTransition("toDeadEndRight")

    def Exit(self):
        #print("Out of right mode")
        pass

class DeadEndRight(State):
    def __int__(self, FSM):
        super(DeadEndRight, self).__init__(FSM)

    def Execute(self):
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        rightMotor.setVelocity(1.0)
        leftMotor.setVelocity(-1.0)
        setDir = self.FSM.direction + 90
        if setDir > 180:
            setDir -= 360
        if abs(setDir - angle) < 5 and abs(self.FSM.direction - angle) > 10:
            self.FSM.direction = setDir
            self.FSM.isTurn(self.FSM.getCard(angle))
            self.FSM.ToTransition("toForwardRight")

    def Exit(self):
        #print("out of deadend mode")
        pass

class DeadEndLeft(State):
    def __int__(self, FSM):
        super(DeadEndLeft, self).__init__(FSM)

    def Execute(self):
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        rightMotor.setVelocity(-1.0)
        leftMotor.setVelocity(1.0)
        setDir = self.FSM.direction - 90
        #print(setDir, angle, self.FSM.direction)
        if setDir < -180:
            setDir += 360
        if abs(setDir - angle) < 5 and abs(self.FSM.direction - angle) > 10:
            self.FSM.direction = setDir
            self.FSM.isTurn(self.FSM.getCard(angle))
            self.FSM.ToTransition("toForwardLeft")

    def Exit(self):
        #print("out of deadend mode")
        pass

class Orient(State):
    def __init__(self, FSM):
        super(Orient, self).__init__(FSM)
        self.direction = None
        self.prevDist = None

    def Execute(self):
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        if self.FSM.toNode is None or self.FSM.change == True:
            self.FSM.toNode = self.FSM.map.path[1]
            self.FSM.prevNode = self.FSM.map.path[0]
            self.FSM.map.path.pop(0)
            self.FSM.change = False
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        print(self.FSM.toNode)
        print(self.FSM.prevNode)
        newDir = self.FSM.map.edges[self.FSM.toNode][self.FSM.prevNode][0]

        if newDir == 'N':
            newAngle = 0
        elif newDir == 'W':
            newAngle = -90
        elif newDir == 'E':
            newAngle = 90
        elif newDir == 'S':
            newAngle = 179.99
        else:
            newAngle = "error"
        setDir = angle - newAngle
        if setDir > 180:
            setDir -= 360
        if setDir < -180:
            setDir += 360
        if setDir < 5:
            self.FSM.navDistance = currentDist
            self.FSM.ToTransition("toNav")
        leftMotor.setVelocity(1.0)
        rightMotor.setVelocity(-1.0)

    def Exit(self):
        #print("out of orient mode")
        pass

class Nav(State):
    def __init__(self, FSM):
        super(Nav, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        one = oneDist.getValue()
        three = threeDist.getValue()
        five = fiveDist.getValue()
        seven = sevenDist.getValue()
        nine = nineDist.getValue()
        eleven = elevenDist.getValue()
        current = (leftE.getValue() + rightE.getValue()) / 2
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        if angle % 90 > 42.5 or three > 400:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(0.8)
        elif angle % 90 < 42.5 or nine > 400:
            rightMotor.setVelocity(0.8)
            leftMotor.setVelocity(1.0)
        else:
            rightMotor.setVelocity(1.0)
            leftMotor.setVelocity(1.0)
        if current - self.FSM.navDistance > self.FSM.map.edges[self.FSM.toNode][self.FSM.prevNode][1] or (left > 400 and right > 400):
            self.FSM.change = True
            self.FSM.prevNode = self.FSM.toNode
            self.FSM.ToTransition("toOrient")
        #print(nine, eleven, left, right, one, three)

    def Exit(self):
        #print("out of nav mode")
        pass

class Transistion(object):
    def __init__(self, toState):
        self.toState = toState

    def Execute(self):
        #print("Transition")
        pass

class FSM(object):
    def __init__(self, char, map):
        self.map = map
        self.char = char
        self.states = {}
        self.transitions = {}
        self.currentState = None
        self.trans = None
        self.direction = None
        self.file = None
        self.id = 0
        self.prevD = None
        self.posx = 0
        self.posy = 0
        self.prevDistance = 0
        self.mode = 0
        self.trophy = False
        self.toNode = None
        self.prevNode = None
        self.navDistance = 0
        self.navLeg = 0
        self.change = True

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state

    def setState(self, stateName):
        self.currentState = self.states[stateName]

    def ToTransition(self, transName):
        self.trans = self.transitions[transName]

    def isTurn(self, direction):
        #print("last", self.prevD, direction)
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        # print(currentDist, self.prevDistance)
        leg = currentDist - self.prevDistance
        if self.prevD != direction or self.trophy:
            opposite = None
            if(self.prevD == "N"):
                self.posy += leg
                opposite = "S"
            elif(self.prevD == "W"):
                self.posx += leg
                opposite = "E"
            elif(self.prevD == "S"):
                self.posy -= leg
                opposite = "N"
            elif(self.prevD == "E"):
                self.posx -= leg
                opposite = "W"
            else:
                print("Error in direction positioning")
            currentID = self.map.id
            prevID = self.map.prev
            self.map.addNode(self.posx, self.posy, self.trophy)
            if(leg < 0.1):
                leg = 0
            #print(currentID, prevID, self.prevD, leg)
            self.map.addEdge(currentID, prevID, self.prevD, leg)
            self.map.addEdge(prevID, currentID, opposite, leg)
            self.prevD = direction
            self.prevDistance = currentDist


    def Execute(self):
        if (self.trans):
            self.currentState.Exit()
            self.trans.Execute()
            self.setState(self.trans.toState)
            self.trans = None
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        if self.prevD is None:
            if self.getCard(angle) != "null":
                self.prevD = self.getCard(angle)
        if self.direction is None:
            self.direction = angle
        if frontLeft.getValue() < 100 and frontRight.getValue() < 100 and touch.getValue() > 0 and leftE.getValue() > 1:
            answer = compass.getValues()
            angle = math.degrees(math.atan2(answer[0], answer[1]))
            if self.map.trophyNode is None:
                self.trophy = True
                self.isTurn(self.getCard(angle))
            else:
                currentDist = (leftE.getValue() + rightE.getValue()) / 2
                self.map.addEdge(self.map.prev, self.map.trophyNode, self.getCard(angle), currentDist - self.prevDistance)
            #self.map.postProcess()
            self.map.writeFile()
            sys.exit(0)
        self.currentState.Execute()

    def getCard(self, bearing):
        if bearing >= -45 and bearing <= 45:
            return "N"
        elif bearing > -135 and bearing < -45:
            return "W"
        elif bearing > 45 and bearing < 135:
            return "E"
        elif bearing <= -135 or bearing >= 135:
            return "S"
        else:
            return "null"

Puck = type("Puck", (object,), {})

class PuckRobot(Puck):
    def __init__(self, map):
        self.FSM = FSM(self, map)
        self.map = map
        self.counter = 0
        self.FSM.AddState("ForwardRight", ForwardRight(self.FSM))
        self.FSM.AddState("Right", RightTurn(self.FSM))
        self.FSM.AddState("DeadEndRight", DeadEndRight(self.FSM))
        self.FSM.AddState("ForwardLeft", ForwardLeft(self.FSM))
        self.FSM.AddState("Left", LeftTurn(self.FSM))
        self.FSM.AddState("DeadEndLeft", DeadEndLeft(self.FSM))
        self.FSM.AddState("Orient", Orient(self.FSM))
        self.FSM.AddState("Nav", Nav(self.FSM))

        self.FSM.AddTransition("toForwardRight", Transistion("ForwardRight"))
        self.FSM.AddTransition("toRight", Transistion("Right"))
        self.FSM.AddTransition("toDeadEndRight", Transistion("DeadEndRight"))
        self.FSM.AddTransition("toForwardLeft", Transistion("ForwardLeft"))
        self.FSM.AddTransition("toLeft", Transistion("Left"))
        self.FSM.AddTransition("toDeadEndLeft", Transistion("DeadEndLeft"))
        self.FSM.AddTransition("toOrient", Transistion("Orient"))
        self.FSM.AddTransition("toNav", Transistion("Nav"))

        self.count = 0
        if map.state == 0:
            self.FSM.setState("ForwardRight")
        elif map.state == 1:
            self.FSM.setState("ForwardLeft")
        elif map.state == 2:
            self.FSM.map.shortestPath()
            self.FSM.setState("Orient")

    def Execute(self):
        self.FSM.Execute()

class Map:
    def __init__(self):
        self.id = 0
        self.nodes = dict({})
        self.edges = dict({})
        self.prev = 0
        self.addNode(0, 0, False)
        self.state = 0
        self.trophyNode = None
        self.path = None

    def loadFile(self):
        if os.path.isfile("data.txt"):
            with open("data.txt", 'r') as file:
                f = file.read()
                if len(f) > 0:
                    lines = f.split("\n")
                    headers = lines[0].split(",")
                    self.state = int(headers[0])
                    self.trophyNode = int(headers[1])
                    nodes = lines[1].split(",")
                    for i in nodes:
                        coord = i.split(" ")
                        if len(coord) == 3:
                            self.addNode(float(coord[1]), float(coord[2]), False)

                    for i in range(2, len(lines)):
                        edges = lines[i].split(",")
                        n = edges[0]
                        for j in range(1, len(edges)):
                            l = edges[j].split(" ")
                            if len(l) == 3 and len(n) > 0:
                                self.addEdge(int(n), int(l[0]), l[1], float(l[2]))
            file.close()

    def addNode(self, x, y, trophyStatus):
        print("node added", self.id)
        self.nodes.update({self.id: [x, y]})
        self.edges[self.id] = dict({})
        if trophyStatus and self.trophyNode is None:
            self.trophyNode = self.id
        self.prev = self.id
        self.id += 1


    def addEdge(self, start, end, direction, weight):
        temp = self.edges[start]
        temp[end] = [direction, weight]
        self.edges[start].update(temp)

    def removeEdge(self, start, end):
        self.edges[start].pop(end)
        self.edges[end].pop(start)

    def printMap(self):
        print(self.trophyNode)
        print(self.nodes)
        print(self.edges)

    def writeFile(self):
        self.state += 1
        with open("test.txt", "w") as file:
            line = ""
            file.write(str(self.state) + ",")
            file.write(str(self.trophyNode))
            file.write("\n")
            for i, posi in self.nodes.items():
                if i != 0:
                    line += str(i) + " " + str(round(posi[0], 4)) + " " + str(round(posi[1], 4)) + ","
            file.write(line)
            file.write("\n")
            for i, posi in self.edges.items():
                edge = str(i) + ","
                for j, posj in posi.items():
                    edge += str(j) + " " + posj[0] + " " + str(round(posj[1], 4)) + ","
                file.write(edge)
                file.write("\n")
        file.close()

    def xDirect(self, num):
        if num > 0:
            dir = ['W', 'E']
        else:
            dir = ['E', 'W']
        return dir

    def yDirect(self, num):
        if num > 0:
            dir = ['N', 'S']
        else:
            dir = ['S', 'N']
        return dir

    def postProcess(self):
        nodes = []
        edges = []
        for i, posi in self.nodes.items():
            for j, posj in self.nodes.items():
                if i != j:
                    xDiff = posi[0] - posj[0]
                    yDiff = posi[1] - posj[1]
                    dist = math.sqrt(pow(xDiff, 2) + pow(yDiff, 2))
                    for l, posl in self.nodes.items():
                        if i == 30 and j == 31 and l == 33:
                            print(self.edges[i], i)
                        if abs(posl[1] - posi[1]) < 3 and abs(posl[1] - posj[1]) < 3 and j in self.edges[i] and yDiff < 3 and abs(posl[0] - posi[0]) < abs(xDiff) and abs(posl[0] - posj[0]) < abs(xDiff):
                            y = (posi[1] + posj[1]) / 2
                            x = posl[0]
                            iTo = x - posi[0]
                            jTo = x - posj[0]
                            lTo = y - posl[1]
                            iDir = self.xDirect(iTo)
                            jDir = self.xDirect(jTo)
                            lDir = self.yDirect(lTo)
                            nodes.append([x, y])
                            edges.append([i, iDir[0], abs(iTo)])
                            edges.append([i, iDir[1], abs(iTo)])
                            edges.append([j, jDir[0], abs(jTo)])
                            edges.append([j, jDir[1], abs(jTo)])
                            edges.append([l, lDir[0], abs(lTo)])
                            edges.append([l, lDir[1], abs(lTo)])
                        if abs(posl[0] - posi[0]) < 3 and abs(posl[0] - posj[0]) < 3 and j in self.edges[i] and xDiff < 3 and abs(posl[1] - posi[1]) < abs(yDiff) and abs(posl[1] - posj[1]) < abs(yDiff):
                            x = (posi[1] + posj[1]) / 2
                            y = posl[0]
                            iTo = y - posi[0]
                            jTo = y - posj[0]
                            lTo = x - posl[1]
                            iDir = self.yDirect(iTo)
                            jDir = self.yDirect(jTo)
                            lDir = self.xDirect(lTo)
                            nodes.append([x, y])
                            edges.append([i, iDir[0], abs(iTo)])
                            edges.append([i, iDir[1], abs(iTo)])
                            edges.append([j, jDir[0], abs(jTo)])
                            edges.append([j, jDir[1], abs(jTo)])
                            edges.append([l, lDir[0], abs(lTo)])
                            edges.append([l, lDir[1], abs(lTo)])

                    if dist < 3:
                        if j not in self.edges[i] or i not in self.edges[j]:
                            direction = ""
                            opposite = ""
                            decideX = 0
                            decideY = 0
                            if abs(xDiff) > abs(yDiff):
                                decideX = xDiff
                            else:
                                decideY = yDiff
                            if decideY > 0 and decideX == 0:
                                direction = "N"
                                opposite = "S"
                            elif decideY < 0 and decideX == 0:
                                direction = "S"
                                opposite = "N"
                            elif decideX > 0 and decideY == 0:
                                direction = "W"
                                opposite = "E"
                            elif decideX < 0 and decideY == 0:
                                direction = "E"
                                opposite = "W"
                            else:
                                print("Post processing error")
                            self.addEdge(i, j, direction, dist)
                            self.addEdge(j, i, opposite, dist)
        for i in range(len(edges)):
            if i % 6 == 0:
                print(i)
                n = nodes[int(i/6)]
                self.addNode(n[0], n[1], False)
                self.removeEdge(edges[i][0], edges[i+2][0])
            e = edges[i]
            if i % 2 == 0:
                self.addEdge(self.prev, e[0], e[1], e[2])
            else:
                self.addEdge(e[0], self.prev, e[1], e[2])

    def minDistance(self, dist, sptSet):
        min = float('inf')
        for i in self.edges.keys():
            for j in self.edges[i].keys():
                if dist[j] < min and sptSet[j] == False:
                    min = dist[j]
                    min_index = j
        return min_index

    def shortestPath(self):
        dist = []
        sptSet = []
        path = []
        for i in self.edges.keys():
            dist.append(float('inf'))
            sptSet.append(False)
            path.append([i])

        dist[0] = 0
        for cout in range(len(self.nodes)):
            x = self.minDistance(dist, sptSet)
            sptSet[x] = True
            for y in self.edges[x].keys():
                if self.edges[x][y][1] > 0 and sptSet[y] == False and dist[y] > dist[x] + self.edges[x][y][1]:
                    dist[y] = dist[x] + self.edges[x][y][1]
                    for j in path[x]:
                        path[y].append(j)
        temp = path[self.trophyNode]
        temp.reverse()
        print(temp)
        for i in range(len(temp) - 1):
            print(self.edges[temp[i]][temp[i+1]])
        self.path = temp


if __name__ == '__main__':
    m = Map()
    m.loadFile()
    m.postProcess()
    m.shortestPath()
    m.writeFile()
    # r = PuckRobot(m)
    # while robot.step(TIME_STEP) != -1:
    #     r.Execute()