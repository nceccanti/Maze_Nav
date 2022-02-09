import os.path
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
answer = compass.getValues()
angle = math.degrees(math.atan2(answer[0], answer[1]))

class State(object):
    def __init__(self, FSM):
        self.FSM = FSM

    def Execute(self):
        pass

    def Exit(self):
        pass


class Forward(State):
    def __init__(self, FSM):
        super(Forward, self).__init__(FSM)

    def Execute(self):
        left = frontLeft.getValue()
        right = frontRight.getValue()
        one = oneDist.getValue()
        three = threeDist.getValue()
        five = fiveDist.getValue()

        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        if angle % 90 > 45:
            rightMotor.setVelocity(0.6)
            leftMotor.setVelocity(0.4)
        elif angle % 90 < 45:
            rightMotor.setVelocity(0.4)
            leftMotor.setVelocity(0.6)
        else:
            rightMotor.setVelocity(0.5)
            leftMotor.setVelocity(0.5)
        # print(left, right)

        if left > 200 and right > 200:
            # self.FSM.println(self.FSM.getCard(angle))
            self.FSM.ToTransition("toDeadEnd", angle)
        if three < 150 and five < 150 and one < 150:
            # self.FSM.println(self.FSM.getCard(angle))
            self.FSM.ToTransition("toRight", angle)

    def Exit(self):
        print("out of forward mode")


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
        rightMotor.setVelocity(0)
        leftMotor.setVelocity(0.5)
        setDir = self.FSM.getDirection() - 90
        if setDir < 0:
            setDir += 360

        if abs(setDir - angle) < 5 and abs(self.FSM.getDirection() - angle) > 10:
            print("Node added")
            self.map.addNode(FSM.getCard(angle))
            self.FSM.ToTransition("toForward", angle)
        if one > 200 or three > 200 or five > 200:
            self.FSM.ToTransition("toForward", setDir + 90)
        if left > 200 and right > 200:
            self.FSM.ToTransition("toDeadEnd", angle)

    def Exit(self):
        print("Out of right mode")


class DeadEnd(State):
    def __int__(self, FSM):
        super(DeadEnd, self).__init__(FSM)

    def Execute(self):
        # left = frontLeft.getValue()
        # right = frontRight.getValue()
        # one = oneDist.getValue()
        # three = threeDist.getValue()
        # five = fiveDist.getValue()
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        rightMotor.setVelocity(0.5)
        leftMotor.setVelocity(-0.5)
        setDir = self.FSM.getDirection() + 90
        if setDir > 360:
            setDir -= 360
        if abs(setDir - angle) < 5 and abs(self.FSM.getDirection() - angle) > 10:
            self.FSM.ToTransition("toForward", angle)

    def Exit(self):
        print("out of deadend mode")


class Transistion(object):
    def __init__(self, toState):
        self.toState = toState

    def Execute(self):
        print("Transition")


class FSM(object):
    def __init__(self, char):
        answer = compass.getValues()
        angle = math.degrees(math.atan2(answer[0], answer[1]))
        self.char = char
        self.states = {}
        self.transitions = {}
        self.currentState = None
        self.trans = None
        self.direction = None
        self.file = None
        self.id = 0
        self.prevE = 1
        self.prevD = self.getCard(angle)

    def AddTransition(self, transName, transition):
        self.transitions[transName] = transition

    def AddState(self, stateName, state):
        self.states[stateName] = state

    def setState(self, stateName):
        self.currentState = self.states[stateName]

    def ToTransition(self, transName, direction):
        self.trans = self.transitions[transName]
        self.direction = direction

    def getDirection(self):
        return self.direction

    def Execute(self):
        if (self.trans):
            self.currentState.Exit()
            self.trans.Execute()
            self.setState(self.trans.toState)
            self.trans = None
        self.currentState.Execute()

    # def println(self, directions):
    #     if self.prevD != directions:
    #         line = str(self.id) + " " + str(self.prevE) + " " + self.prevD
    #         self.prevD = directions
    #         self.prevE = self.id
    #         self.id += 1

    def getCard(self, bearing):
        if bearing >= -45 and bearing <= 45:
            return "N"
        elif bearing > -135 and bearing < -45:
            return "W"
        elif bearing > 45 and bearing < 135:
            return "E"
        elif bearing <= -135 and bearing >= 135:
            return "S"
        else:
            return "null"


Puck = type("Puck", (object,), {})


class PuckRobot(Puck):
    def __init__(self, map):
        self.FSM = FSM(self)
        self.map = map
        self.FSM.AddState("Forward", Forward(self.FSM))
        self.FSM.AddState("Right", RightTurn(self.FSM))
        self.FSM.AddState("DeadEnd", DeadEnd(self.FSM))

        self.FSM.AddTransition("toForward", Transistion("Forward"))
        self.FSM.AddTransition("toRight", Transistion("Right"))
        self.FSM.AddTransition("toDeadEnd", Transistion("DeadEnd"))

        self.FSM.setState("Forward")

    def Execute(self):
        self.FSM.Execute()


class Map:
    def __init__(self):
        self.id = 1
        self.map = dict({})
        self.prev = 0
        self.addNode("null")

    def loadFile(self):
        if os.path.isfile("data.txt"):
            with open("data.txt", 'r') as file:
                f = file.read()
                lines = f.split("\n")
                for i in range(1, len(lines)):
                    temp = lines[i].split(" ")
                    temp[len(temp) - 1].strip("\n")
                    self.map.update({temp[0]: {}})
                    for j in range(1, len(temp), 2):
                        self.map[temp[0]].update({temp[j]: temp[j + 1]})

    def addNode(self, direction):
        self.map.update({self.id: {self.prev: direction}})
        self.prev = self.id
        self.id += 1

    def addEdge(self, key, direction, current):
        self.map[key].update({current: direction})

    def printMap(self):
        print(self.map)


if __name__ == '__main__':
    m = Map()
    r = PuckRobot(m)
    while robot.step(TIME_STEP) != -1:
        r.Execute()

