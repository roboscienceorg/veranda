import rclpy
import heapq
import time

from std_msgs.msg import Float64MultiArray

# Eventually rclpy.Timer will allow
# for custom time sources; when that happens
# this can probably be rewritten a bit
class SimTimer:
    def __init__(self, useSimulatedTime, simulatedTimeChannel, rosNode):
        self.__usingSimulatedTime = useSimulatedTime
        self.__simulatedTimeChannel = simulatedTimeChannel
        self.__rosNode = rosNode

        # Priority queue of timers
        self.__timers = []

        # Keep track of global time passed in seconds
        # from simulator
        self.__globalTime = 0

        # Keep track of instantiation time to return
        # global time when not simulated
        self.__instantiation = time.time()

        if self.__usingSimulatedTime:
            def timerCb(msg):
                self.__globalTime = msg.data[0]
                while len(self.__timers) > 0 and self.__timers[0][0] <= self.__globalTime:
                    tData = heapq.heappop(self.__timers)
                    tData[2]()
                    tData[0] = tData[0] + tData[1]
                    heapq.heappush(self.__timers, tData)

            rosNode.create_subscription(Float64MultiArray, self.__simulatedTimeChannel, timerCb)

    def global_time(self):
        if self.__usingSimulatedTime:
            return self.__globalTime
        else:
            return time.time() - self.__instantiation

    def create_timer(self, dt, cb):
        if self.__usingSimulatedTime:
            heapq.heappush(self.__timers, [self.__globalTime + dt, dt, cb])
        else:
            self.__rosNode.create_timer(dt, cb)

    