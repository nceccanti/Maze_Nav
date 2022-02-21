import os.path
import sys

from controller import Robot, Motor
import math

TIME_STEP = 64
robot = Robot()
#print("Starting")
leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')
leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
rightMotor.setVelocity(6)
leftMotor.setVelocity(6)
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
touchRight = robot.getDevice("touch sensor")
touchRight.enable(TIME_STEP)
touchLeft = robot.getDevice("touch sensor(1)")
touchLeft.enable(TIME_STEP)


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
        #print("forward")
        left = frontLeft.getValue()
        right = frontRight.getValue()
        one = oneDist.getValue()
        three = threeDist.getValue()
        five = fiveDist.getValue()

        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        if one > 200:
            rightMotor.setVelocity(4.5)
            leftMotor.setVelocity(4.0)
        elif angle % 90 > 42.5 or three > 500:
            rightMotor.setVelocity(4.5)
            leftMotor.setVelocity(4.2)
        elif angle % 90 < 42.5:
            rightMotor.setVelocity(4.2)
            leftMotor.setVelocity(4.5)
        else:
            rightMotor.setVelocity(4.5)
            leftMotor.setVelocity(4.5)
        if left > 200 and right > 200:
            self.FSM.direction = angle
            self.FSM.ToTransition("toDeadEndRight")
        if three < 80 and five < 80 and one < 80:
            print(three, five, one)
            self.FSM.ToTransition("toRight")


    def Exit(self):
        print("out of forward mode")
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
            rightMotor.setVelocity(4.0)
            leftMotor.setVelocity(4.6)
        if angle % 90 < 42.5:
            rightMotor.setVelocity(4.2)
            leftMotor.setVelocity(4.5)
        elif angle % 90 > 42.5 or nine > 500:
            rightMotor.setVelocity(4.5)
            leftMotor.setVelocity(4.2)
        else:
            rightMotor.setVelocity(4.5)
            leftMotor.setVelocity(4.5)
        if left > 200 and right > 200:
            self.FSM.direction = angle
            self.FSM.ToTransition("toDeadEndLeft")
        if seven < 80 and nine < 80 and eleven < 80:
            self.FSM.ToTransition("toLeft")


    def Exit(self):
        print("out of forward mode")
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
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        if self.FSM.coast is None:
            self.FSM.coast = currentDist + 1.4
        setDir = self.FSM.direction + 95
        if setDir > 180:
            setDir -= 360
        if abs(setDir - angle) < 10 or abs(setDir - angle) > 355:
            self.FSM.isTurn(self.FSM.getCard(angle), 'N')
            rightMotor.setVelocity(3.0)
            leftMotor.setVelocity(2.8)
        elif self.FSM.coast < currentDist:
            self.FSM.rotationDist = currentDist - self.FSM.coast
            rightMotor.setVelocity(3.0)
            leftMotor.setVelocity(-3.0)

        if left > 200 and right > 200:
            self.FSM.direction = setDir - 5
            self.FSM.isTurn(self.FSM.getCard(angle), 'D')
            self.FSM.coast = None
            self.FSM.ToTransition("toDeadEndLeft")
        if abs(setDir - angle) < 20 and abs(self.FSM.direction - angle) > 10  and abs(self.FSM.direction - angle) > 10 and nine > 300:
            self.FSM.direction = setDir - 5
            self.FSM.coast = None
            self.FSM.prevDistance += self.FSM.rotationDist
            self.FSM.isTurn(self.FSM.getCard(angle), 'N')
            self.FSM.ToTransition("toForwardLeft")
        # if seven > 200 or nine > 200 or eleven > 200:
        #     self.FSM.direction = setDir
        #     self.FSM.isTurn(self.FSM.getCard(angle))
        #     self.FSM.ToTransition("toForwardLeft")

    def Exit(self):
        print("Out of left mode")
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
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        if self.FSM.coast is None:
            self.FSM.coast = currentDist + 1.4

        setDir = self.FSM.direction - 95
        if setDir < -180:
            setDir += 360
        if abs(setDir - angle) < 10 or abs(setDir - angle) > 355:
            self.FSM.isTurn(self.FSM.getCard(angle), 'N')
            rightMotor.setVelocity(2.8)
            leftMotor.setVelocity(3.0)
        elif self.FSM.coast < currentDist:
            self.FSM.rotationDist = currentDist - self.FSM.coast
            rightMotor.setVelocity(-3.0)
            leftMotor.setVelocity(3.0)
            print(self.FSM.rotationDist)

        if left > 200 or right > 200:
            self.FSM.direction = setDir + 5
            self.FSM.isTurn(self.FSM.getCard(angle), 'D')
            self.FSM.coast = None
            self.FSM.ToTransition("toDeadEndRight")
        if (abs(setDir - angle) < 10 or abs(setDir - angle) > 355) and abs(self.FSM.direction - angle) > 10 and three > 300:
            self.FSM.direction = setDir + 5
            self.FSM.coast = None
            self.FSM.prevDistance += self.FSM.rotationDist
            self.FSM.isTurn(self.FSM.getCard(angle), 'N')
            self.FSM.ToTransition("toForwardRight")
        # if one > 200 or three > 200 or five > 200:
        #     self.FSM.coast = None
        #     self.FSM.ToTransition("toForwardRight")

    def Exit(self):
        print("Out of right mode")
        pass

class DeadEndRight(State):
    def __int__(self, FSM):
        super(DeadEndRight, self).__init__(FSM)

    def Execute(self):
        #print("deadend")
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        if self.FSM.coast is None:
            self.FSM.coast = currentDist - 0.3

        if currentDist > self.FSM.coast:
            rightMotor.setVelocity(-3.0)
            leftMotor.setVelocity(-3.0)
            self.FSM.rotationDist = currentDist - self.FSM.coast
        else:
            rightMotor.setVelocity(3.0)
            leftMotor.setVelocity(-3.0)
        setDir = self.FSM.direction + 85
        if setDir > 180:
            setDir -= 360
        if abs(setDir - angle) < 10 and abs(self.FSM.direction - angle) > 10:
            self.FSM.coast = None
            self.FSM.direction = setDir + 5
            self.FSM.prevDistance += self.FSM.rotationDist
            self.FSM.isTurn(self.FSM.getCard(angle), 'D')
            self.FSM.ToTransition("toForwardRight")

    def Exit(self):
        print("out of deadend mode")
        pass

class DeadEndLeft(State):
    def __int__(self, FSM):
        super(DeadEndLeft, self).__init__(FSM)

    def Execute(self):
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        if self.FSM.coast is None:
            self.FSM.coast = currentDist - 0.3

        if currentDist > self.FSM.coast:
            rightMotor.setVelocity(-3.0)
            leftMotor.setVelocity(-3.0)
        else:
            self.FSM.rotationDist = currentDist - self.FSM.coast
            rightMotor.setVelocity(-3.0)
            leftMotor.setVelocity(3.0)
        setDir = self.FSM.direction - 85
        if setDir < -180:
            setDir += 360
        if abs(setDir - angle) < 5 and abs(self.FSM.direction - angle) > 10:
            self.FSM.coast = None
            self.FSM.direction = setDir - 5
            self.FSM.prevDistance += self.FSM.rotationDist
            self.FSM.isTurn(self.FSM.getCard(angle), 'D')
            self.FSM.ToTransition("toForwardLeft")

    def Exit(self):
        #print("out of deadend mode")
        pass

class Orient(State):
    def __init__(self, FSM):
        super(Orient, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
        if self.FSM.toNode is None or self.FSM.change == True:
            if len(self.FSM.map.path) > 1:
                self.FSM.toNode = self.FSM.map.path[1]
                self.FSM.prevNode = self.FSM.map.path[0]
                self.FSM.map.path.pop(0)
            self.FSM.change = False
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        freeze = False
        if self.FSM.toNode != self.FSM.prevNode:
            print(self.FSM.toNode, self.FSM.prevNode)
            newDir = self.FSM.map.edges[self.FSM.toNode][self.FSM.prevNode][0]
        else:
            newDir = self.FSM.oldDir
            freeze = True
        self.FSM.oldDir = newDir
        if newDir == 'N':
            newAngle = 0
        elif newDir == 'W':
            newAngle = -90
        elif newDir == 'E':
            newAngle = 90
        elif newDir == 'S':
            if angle < 0:
                newAngle = -180
            else:
                newAngle = 180
        else:
            newAngle = "error"
            print("error")
        if (newAngle < 0 and angle < 0) or (newAngle >= 0 and angle >= 0):
            setDir = newAngle - angle
        else:
            setDir = newAngle + angle
        if abs(setDir) < 5 and not freeze:
            self.FSM.navDistance = currentDist
            self.FSM.ToTransition("toNav")
        if not freeze:
            if setDir < 0:
                leftMotor.setVelocity(3.0)
                rightMotor.setVelocity(-3.0)
            else:
                leftMotor.setVelocity(-3.0)
                rightMotor.setVelocity(3.0)
        else:
            leftMotor.setVelocity(3.0)
            rightMotor.setVelocity(3.0)
        if left > 400 and right > 400:
            print("yes")

    def Exit(self):
        print("out of orient mode")
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
            rightMotor.setVelocity(5.0)
            leftMotor.setVelocity(4.5)
        elif angle % 90 < 42.5 or nine > 400:
            rightMotor.setVelocity(4.5)
            leftMotor.setVelocity(5.0)
        else:
            rightMotor.setVelocity(6.0)
            leftMotor.setVelocity(6.0)
            # -0.4
        if current - self.FSM.navDistance > self.FSM.map.edges[self.FSM.toNode][self.FSM.prevNode][1] or (left > 400 and right > 400 and current - self.FSM.navDistance > self.FSM.map.edges[self.FSM.toNode][self.FSM.prevNode][1] / 3):
            self.FSM.change = True
            self.FSM.prevNode = self.FSM.toNode
            self.FSM.ToTransition("toOrient")
        elif left > 400 and right > 400 and current - self.FSM.navDistance < self.FSM.map.edges[self.FSM.toNode][self.FSM.prevNode][1] / 3:
            self.FSM.ToTransition("toAvoid")

    def Exit(self):
        print("out of nav mode")
        pass

class Avoiddance(State):
    def __init__(self, FSM):
        super(Avoiddance, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        one = oneDist.getValue()
        eleven = elevenDist.getValue()
        dir = one - eleven
        if dir > 0:
            leftMotor.setVelocity(-1.0)
            rightMotor.setVelocity(1.0)
        else:
            leftMotor.setVelocity(1.0)
            rightMotor.setVelocity(-1.0)

        if left < 200 and right < 200:
            self.FSM.navDistance +=  ((leftE.getValue() + rightE.getValue()) / 2) + self.FSM.navDistance
            self.FSM.ToTransition("toNav")

    def Exit(self):
        print("out of avoidance mode")


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
        self.erase = False
        self.oldDir = None
        self.coast = None
        self.turnMod = 0
        self.endMod = 0
        self.rotationDist = 0

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state

    def setState(self, stateName):
        self.currentState = self.states[stateName]

    def ToTransition(self, transName):
        self.trans = self.transitions[transName]

    def isTurn(self, direction, sign):
        currentDist = (leftE.getValue() + rightE.getValue()) / 2
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
                #print("Error in direction positioning")
                pass
            currentID = self.map.id
            prevID = self.map.prev
            if self.trophy:
                sign = 'D'
            self.map.addNode(self.posx, self.posy, self.trophy, sign)
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
        l1 = (elevenDist.getValue() < 150 and touchLeft.getValue() > 0) or (oneDist.getValue() < 150 and touchRight.getValue() > 0)
        l2 = (touchRight.getValue() > 0 and touchLeft.getValue() > 0)
        l3 = frontLeft.getValue() < 100 and frontRight.getValue() < 100 and leftE.getValue() > 1
        #print(l1, l2, l3)
        if (l1 or l2) and l3:
            print(l1, l2, l3)
            answer = compass.getValues()
            angle = math.degrees(math.atan2(answer[0], answer[1]))
            if self.map.trophyNode is None:
                self.trophy = True
                self.isTurn(self.getCard(angle), 'N')
            else:
                currentDist = (leftE.getValue() + rightE.getValue()) / 2
                newAng = angle - 180
                if newAng < -180:
                    newAng += 360
                self.map.addEdge(self.map.prev, self.map.trophyNode, self.getCard(newAng), currentDist - self.prevDistance)
                self.map.addEdge(self.map.trophyNode, self.map.prev, self.getCard(angle), currentDist - self.prevDistance)
            if self.map.state == 1:
                self.map.postProcess()
            if self.map.state == 2:
                self.map.endGame()
                pass
            if self.erase:
                #self.map.eraseFile()
                pass
            else:
                self.map.writeFile()
                pass
            print("Win")
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
        self.FSM.AddState("Avoid", Avoiddance(self.FSM))

        self.FSM.AddTransition("toForwardRight", Transistion("ForwardRight"))
        self.FSM.AddTransition("toRight", Transistion("Right"))
        self.FSM.AddTransition("toDeadEndRight", Transistion("DeadEndRight"))
        self.FSM.AddTransition("toForwardLeft", Transistion("ForwardLeft"))
        self.FSM.AddTransition("toLeft", Transistion("Left"))
        self.FSM.AddTransition("toDeadEndLeft", Transistion("DeadEndLeft"))
        self.FSM.AddTransition("toOrient", Transistion("Orient"))
        self.FSM.AddTransition("toNav", Transistion("Nav"))
        self.FSM.AddTransition("toAvoid", Transistion("Avoid"))

        self.count = 0
        if map.state == 0:
            self.FSM.setState("ForwardRight")
        elif map.state == 1:
            self.FSM.setState("ForwardLeft")
        elif map.state == 2:
            self.FSM.map.shortestPath()
            self.FSM.setState("Orient")
            self.FSM.erase = True
        elif map.state > 3:
            self.FSM.map.eraseFile()
            print("Previous map file erased. Restart program.")
            sys.exit(0)

    def Execute(self):
        self.FSM.Execute()

class Map:
    def __init__(self):
        self.id = 0
        self.nodes = dict({})
        self.edges = dict({})
        self.prev = 0
        self.addNode(0, 0, False, 'D')
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
                        if len(coord) == 4:
                            self.addNode(float(coord[1]), float(coord[2]), False, coord[3])

                    for i in range(2, len(lines)):
                        edges = lines[i].split(",")
                        n = edges[0]
                        for j in range(1, len(edges)):
                            l = edges[j].split(" ")
                            if len(l) == 3 and len(n) > 0:
                                self.addEdge(int(n), int(l[0]), l[1], float(l[2]))
            file.close()
            self.prev = 0

    def addNode(self, x, y, trophyStatus, isDeadEnd):
        print("node added", self.id, isDeadEnd)
        self.nodes.update({self.id: [x, y, isDeadEnd]})
        self.edges[self.id] = dict({})
        if trophyStatus:
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
        with open("data.txt", "w") as file:
            line = ""
            file.write(str(self.state) + ",")
            file.write(str(self.trophyNode))
            file.write("\n")
            for i, posi in self.nodes.items():
                if i != 0:
                    line += str(i) + " " + str(round(posi[0], 4)) + " " + str(round(posi[1], 4)) + " " + posi[2] + ","
            file.write(line)
            file.write("\n")
            for i, posi in self.edges.items():
                edge = str(i) + ","
                for j, posj in posi.items():
                    edge += str(j) + " " + posj[0] + " " + str(round(posj[1], 4)) + ","
                file.write(edge)
                file.write("\n")
        file.close()

    def eraseFile(self):
        with open("data.txt", "w") as file:
            file.write("")
        file.close()

    def vector(self, one, two):
        x = two[0] - one[0]
        y = two[1] - one[1]
        card = ''
        opp = ''
        if abs(x) < abs(y):
            if y > 0:
                card = 'S'
                opp = 'N'
            else:
                card = 'N'
                opp = 'S'
        else:
            if x > 0:
                card = 'W'
                opp = 'E'
            else:
                card = 'E'
                opp = 'W'
        dist = math.sqrt(pow(x, 2) + pow(y, 2))
        return [card, opp, dist]

    def endGame(self):
        ext = 5
        cardinal = self.getCard()
        while cardinal == "null":
            cardinal = self.getCard()
            print(cardinal)
        isDir = True
        for i in self.edges[self.trophyNode].keys():
            print(self.edges[self.trophyNode][i][0], cardinal, "trophy")
            if self.edges[self.trophyNode][i][0] == cardinal:
                isDir = False
        if isDir:
            modX = 0
            modY = 0
            opp = ''
            if cardinal == 'N':
                modY = -ext
                opp = 'S'
            elif cardinal == 'E':
                modX = -ext
                opp = 'W'
            elif cardinal == 'W':
                modX = ext
                opp = 'E'
            elif cardinal == 'S':
                modY = ext
                opp = 'N'
            temp = self.nodes[self.trophyNode]
            tempNode = self.trophyNode
            temp[0] += modX
            temp[1] += modY
            print("bruh")
            self.addNode(temp[0], temp[1], True, 'N')
            self.addEdge(self.prev, tempNode, opp, ext)
            self.addEdge(tempNode, self.prev, cardinal, ext)

    def postProcess(self):
        print("processing data")
        for i, posi in self.nodes.items():
            for j, posj in self.nodes.items():
                if i != j and posi[2] != 'D' or posj[2] != 'D':
                    xDiff = posi[0] - posj[0]
                    yDiff = posi[1] - posj[1]
                    dist = math.sqrt(pow(xDiff, 2) + pow(yDiff, 2))
                    if dist < 3:
                        if j not in self.edges[i] or i not in self.edges[j]:
                            isConnected = False
                            for l in self.edges[i].keys():
                                for k in self.edges[j].keys():
                                    if k == l:
                                        isConnected = True
                            if not isConnected:
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
                                    #print("Post processing error")
                                    pass
                                print(i,j)
                                self.addEdge(i, j, direction, dist)
                                self.addEdge(j, i, opposite, dist)

        nodesX = []
        nodesY = []
        print(nodesX)
        print(nodesY)
        for i, posi in self.nodes.items():
            for j, posj in self.nodes.items():
                if i != j and posi[2] != 'D' or posj[2] != 'D':
                    xDiff = posi[0] - posj[0]
                    yDiff = posi[1] - posj[1]
                    for l, posl in self.nodes.items():
                        if posl[2] != 'D':
                            if l not in self.edges[i] and l not in self.edges[j] and abs(posl[0] - posi[0]) < abs(xDiff) and abs(posl[0] - posj[0]) < abs(xDiff) and j in self.edges[i] and abs(posl[1] - posi[1]) < 3 and abs(posl[1] - posj[1]) < 3 and posi[0] > posj[0]:
                                x = posl[0]
                                y = (posi[1] + posj[1]) / 2
                                nodesX.append([x,y,i,j,l])
                                #print(i,j,l)
                            if l not in self.edges[i] and l not in self.edges[j] and abs(posl[1] - posi[1]) < abs(yDiff) and abs(posl[1] - posj[1]) < abs(yDiff) and j in self.edges[i] and abs(posl[0] - posi[0]) < 3 and abs(posl[0] - posj[0]) < 3 and posi[1] > posj[1]:
                                x = (posi[0] + posj[0]) / 2
                                y = posl[1]
                                nodesY.append([x,y,i,j,l])
                                #print(x,y,i,j,l)

        deleteX = []
        dupsX = []
        deleteY = []
        dupsY = []
        for i in range(len(nodesX)):
            for j in range(i, len(nodesX)):
                if i != j and nodesX[i][2] == nodesX[j][2] and nodesX[i][3] == nodesX[j][3]:
                    dupsX.append([nodesX[i], nodesX[j]])
                    deleteX.append(j)
        for i in range(len(nodesY)):
            for j in range(i, len(nodesY)):
                if i != j and nodesY[i][2] == nodesY[j][2] and nodesY[i][3] == nodesY[j][3]:
                    dupsY.append([nodesY[i], nodesY[j]])
                    deleteY.append(j)
        print(nodesX)
        print(deleteX)
        for i in deleteX:
            if i in nodesX:
                nodesX.pop(i)
        for i in deleteY:
            if i in nodesY:
                nodesY.pop(i)

        for i in nodesX:
            print("821")
            if i[2] in self.edges[i[3]]:
                self.addNode(i[0],i[1], False, 'N')
                self.addEdgePostX(i[2], i[3], i[4], self.prev)
        for i in nodesY:
            print("825")
            if i[2] in self.edges[i[3]]:
                self.addNode(i[0],i[1], False, 'N')
                self.addEdgePostY(i[2], i[3], i[4], self.prev)

        for i in dupsX:
            re = -1
            for j in self.nodes.keys():
                isEdge = True
                for l in range(2, len(i[0])):
                    if i[0][l] not in self.edges[j]:
                        isEdge = False
                if isEdge:
                    re = j
                    break
            if re != -1:
                if self.nodes[re][0] < i[1][0] and self.nodes[i[1][2]][0] > i[1][0]:
                    self.addNode(i[1][0], i[1][1], False, 'N')
                    self.addEdgePostX(i[1][2], re, i[1][4], self.prev)
                elif self.nodes[re][0] < i[1][0] and self.nodes[i[1][3]][0] < i[1][0]:
                    self.addNode(i[1][0], i[1][1], False, 'N')
                    self.addEdgePostX(re, i[1][3], i[1][4], self.prev)

        for i in dupsY:
            re = -1
            for j in self.nodes.keys():
                isEdge = True
                for l in range(2, len(i[0])):
                    if i[0][l] not in self.edges[j]:
                        isEdge = False
                if isEdge:
                    re = j
                    break
            if re != -1:
                print("858")
                print("post process")
                if self.nodes[re][1] < i[1][1] and self.nodes[i[1][2]][1] > i[1][1]:
                    self.addNode(i[1][0], i[1][1], False, 'N')
                    self.addEdgePostY(i[1][2], re, i[1][4], self.prev)
                elif self.nodes[re][1] < i[1][1] and self.nodes[i[1][3]][1] < i[1][1]:
                    self.addNode(i[1][0], i[1][1], False, 'N')
                    self.addEdgePostY(re, i[1][3], i[1][4], self.prev)

    def addEdgePostX(self, node1, node2, node3, newNode):
        self.removeEdge(node1, node2)
        coord1 = self.nodes[node1]
        coord2 = self.nodes[node2]
        coord3 = self.nodes[node3]
        coordNew = self.nodes[newNode]
        oneEdge = self.vector(coordNew, coord1)
        twoEdge = self.vector(coordNew, coord2)
        threeEdge = self.vector(coord3, coordNew)
        self.addEdge(node1, newNode, oneEdge[0], oneEdge[2])
        self.addEdge(newNode, node1, oneEdge[1], oneEdge[2])
        self.addEdge(node2, newNode, twoEdge[0], twoEdge[2])
        self.addEdge(newNode, node2, twoEdge[1], twoEdge[2])
        self.addEdge(node3, newNode, threeEdge[0], threeEdge[2])
        self.addEdge(newNode, node3, threeEdge[1], threeEdge[2])

    def addEdgePostY(self, node1, node2, node3, newNode):
        self.removeEdge(node1, node2)
        coord1 = self.nodes[node1]
        coord2 = self.nodes[node2]
        coord3 = self.nodes[node3]
        coordNew = self.nodes[newNode]
        oneEdge = self.vector(coord1, coordNew)
        twoEdge = self.vector(coord2, coordNew)
        threeEdge = self.vector(coordNew, coord3)
        self.addEdge(node1, newNode, oneEdge[0], oneEdge[2])
        self.addEdge(newNode, node1, oneEdge[1], oneEdge[2])
        self.addEdge(node2, newNode, twoEdge[0], twoEdge[2])
        self.addEdge(newNode, node2, twoEdge[1], twoEdge[2])
        self.addEdge(node3, newNode, threeEdge[0], threeEdge[2])
        self.addEdge(newNode, node3, threeEdge[1], threeEdge[2])

    def shortestPath(self):
        unvisited = {node: None for node in self.nodes}
        path = {node: [] for node in self.nodes}
        visited = {}
        current = 0
        currentDistance = 0
        unvisited[current] = currentDistance
        while True:
            for neighbour, distance in self.edges[current].items():
                if neighbour not in unvisited: continue
                newDistance = currentDistance + distance[1]
                if unvisited[neighbour] is None or unvisited[neighbour] > newDistance:
                    unvisited[neighbour] = newDistance
                    path[neighbour].append(unvisited[neighbour])
            visited[current] = currentDistance
            del unvisited[current]
            if not unvisited: break
            candidates = [node for node in unvisited.items() if node[1]]
            current, currentDistance = sorted(candidates, key = lambda x: x[1])[0]
        print(path)

    def getCard(self):
        answer = compass.getValues()
        bearing = math.degrees(math.atan2(answer[0], answer[1]))
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


if __name__ == '__main__':
    m = Map()
    m.loadFile()
    m.shortestPath()
    # r = PuckRobot(m)
    # while robot.step(TIME_STEP) != -1:
    #     r.Execute()