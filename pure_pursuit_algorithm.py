import matplotlib.pyplot as plt
import math

# select lookahead based on speed
# find goal point
# generate control command 
# update state
# animation

class State:
    """
    vehicle state class
    """

    def __init__(self):
        self.x = 0
        self.y = 0.1
        self.yaw = math.pi/6
        self.v = 0
        
    def updateState(self, x, y, yaw, v):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        
class Params:
    """
    vehicle params
    """
    def __init__(self, minLA=0.1, maxLA=1, maxV=1):
        self.minLA = minLA
        self.maxLA = maxLA
        self.maxV = maxV
        self.lookAhead = 0
        
class Path:
    
    def __init__(self, x = 0.0 , y = 0.0, th = 0.0):
        self.x = [0,]
        self.y = [0,]
        self.th = [0,]
        
    def appendCircularPath(self, r, arcL, arcAoff, dir):
        
        if dir == 'cw':
            cx = self.x[-1] + r*math.sin(self.th[-1] + arcAoff)
            cy = self.y[-1] - r*math.cos(self.th[-1] + arcAoff)  
            thOff = self.th[-1] + arcAoff + math.pi/2
            mov = -1
        else:
            cx = self.x[-1] - r*math.sin(self.th[-1] + arcAoff)
            cy = self.y[-1] + r*math.cos(self.th[-1] + arcAoff)  
            thOff = self.th[-1] + arcAoff - math.pi/2 
            mov = 1
            
        numPoints = 100
        for i in range(1, numPoints+1):
            th = thOff + mov*i*(arcL)/numPoints
            self.x.append(cx + r*math.cos(th))
            self.y.append(cy + r*math.sin(th))
            self.th.append(th + mov*math.pi/2)
            
    def appendStraightPath(self, pL, lA):
               
        numPoints = 100
        dl = pL / 100
        th = lA + self.th[-1]
        for i in range(1, numPoints+1):
            self.x.append(self.x[-1] + dl*math.cos(th))
            self.y.append(self.y[-1] + dl*math.sin(th))
            self.th.append(th)
            
    def getCourse0(self):
        self.appendStraightPath(1, math.pi/6)
        self.appendCircularPath(1, math.pi, math.pi/3, 'cw')
        self.appendCircularPath(1, math.pi, -math.pi/3, 'ccw')
        self.appendStraightPath(1, math.pi)
        
    def getCourse1(self):
        self.appendStraightPath(1, math.pi/2)
        self.appendCircularPath(1, math.pi, math.pi, 'cw')
        self.appendCircularPath(1, math.pi, math.pi, 'cw')
        self.appendStraightPath(1, math.pi)
        
    def getCourse2(self):
        self.appendStraightPath(1, math.pi/6)
        self.appendCircularPath(1, math.pi, 0, 'cw')
        self.appendCircularPath(1, math.pi, 0, 'ccw')
        self.appendStraightPath(1, 0)
        
    def getCourse3(self):
        self.appendStraightPath(1, math.pi/2)
        self.appendCircularPath(1, math.pi, math.pi, 'cw')
        self.appendStraightPath(1, math.pi)
        self.appendStraightPath(1, -math.pi/3)
        
        
class AlgoritmUtility:
    
    def __init__(self):
        self.pI = 0
        self.cI = 0
        self.cte = [0,]
        self.oe = [0,]
        self.mind = 10000000
    
    def computeDist(self, x1, y1, x2, y2):
        return math.sqrt((y2-y1)*(y2-y1) + (x2-x1)*(x2-x1))
    
    def crossProduct(self, x1, y1, x2, y2):
        return x1*y2 - y1*x2
    
    def computeNearestPoint(self, state, path):
        
        i = self.pI
        self.mind = self.computeDist(path.x[self.pI], path.y[self.pI], state.x, state.y)
        for x, y in zip(path.x[self.pI:], path.y[self.pI:]):
            d = self.computeDist(x, y, state.x, state.y)
            if d < self.mind:
                self.mind = d
                self.pI = i    
            i += 1
        return 
        
        # if self.pI < len(path.x) - 1:
        #     d1 = self.computeDist(state.x, state.y, path.x[self.pI], path.y[self.pI])
        #     d2 = self.computeDist(state.x, state.y, path.x[self.pI+1], path.y[self.pI+1])
        #     self.mind = min(d1,d2)
        #     if d2 < d1:
        #         self.pI += 1
        # return
        
    def computeError(self, state, path):
        cp = self.crossProduct(state.x - path.x[self.pI], state.y - path.y[self.pI], 
                               path.x[self.pI] - path.x[self.pI-1], path.y[self.pI] - path.y[self.pI-1])
        d = self.computeDist(path.x[self.pI], path.y[self.pI], path.x[self.pI-1], path.y[self.pI-1])
        self.cte.append(cp/d)
        self.oe.append(state.yaw - path.th[self.pI])
        return
        
    def computeIntersection(self, state, path, param):
        i = 0
        for x, y in zip(path.x[self.pI:], path.y[self.pI:]):
            if self.computeDist(x,y,state.x,state.y) < param.lookAhead:
                self.cI = self.pI + i
            i += 1
        return path.x[self.cI] , path.y[self.cI]
    
    def tranformToRobotFrame(self, state, xw, yw):
        xr =  xw*math.cos(state.yaw) + yw*math.sin(state.yaw)
        yr = -xw*math.sin(state.yaw) + yw*math.cos(state.yaw)
        return xr, yr
        
    def computeOmega(self, state, k):
        return k*state.v
            
            
            
def main():
    
    param = Params()
    state = State()
    path = Path()
    alg = AlgoritmUtility()
    
    x = [state.x,]
    y = [state.y,]
    th = [state.yaw,]
    dt = 0.01
    
    path.getCourse3()
    path.th[0] = path.th[1]
    t = [0,]
    state.v = 1
    while(alg.computeDist(state.x, state.y, path.x[-1], path.y[-1]) > 0.1):
        alg.computeNearestPoint(state, path)
        alg.computeError(state, path)
        t.append(t[-1]+dt)
        param.lookAhead = 0.05 #param.minLA + (param.maxLA - param.minLA)*state.v/param.maxV
        gxw, gyw = alg.computeIntersection(state, path, param)
        print(f'Nearest INDEX : {alg.pI}, Path Point : ({path.x[alg.pI]}, {path.y[alg.pI]}), Target Point : ({gxw}, {gyw})')
        gxr, gyr = alg.tranformToRobotFrame(state, gxw - state.x, gyw - state.y)
        k = 2*gyr/(param.lookAhead**2)
        w = alg.computeOmega(state, k)
        x.append(state.x + state.v*math.cos(th[-1])*dt)
        y.append(state.y + state.v*math.sin(th[-1])*dt)
        th.append(state.yaw + w*dt)
        state.updateState(x[-1], y[-1], th[-1], state.v)
        plt.cla()
        # for stopping simulation with the esc key.
        plt.gcf().canvas.mpl_connect('key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
        plt.plot(path.x, path.y,x, y)
        plt.plot(gxw, gyw, 'xg')
        plt.plot(x[-1], y[-1], 'or')
        plt.axis('equal')
        plt.pause(0.000001)
        
    cteRMS = [x**2 for x in alg.cte]
    oeRMS = [x**2 for x in alg.oe]
    
    print(f'CROSSTRACK ERROR : {math.sqrt(sum(cteRMS)/len(cteRMS))}')
    print(f'ORIENTATION ERROR : {math.sqrt(sum(oeRMS)/len(oeRMS))}')
    plt.cla()
    plt.plot(t, alg.cte, '-r', t, alg.oe, '-b')
    plt.show()
    
    # plt.plot(path.x, path.y)
    # plt.show()
    
    
    

if __name__ == '__main__':
    main()