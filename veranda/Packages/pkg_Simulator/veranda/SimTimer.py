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
        self.__nextTimerHandle = 0

        # Priority queue of timers
        self.__timers = []
        self.__uninittedTimers = []

        # Keep track of global time passed in seconds
        # from simulator
        self.__globalTime = None

        # Keep track of instantiation time to return
        # global time when not simulated
        self.__instantiation = time.time()

        if self.__usingSimulatedTime:
            def timerCb(msg):
                if self.__globalTime is None:
                    for newTimer in self.__uninittedTimers:
                        if not newTimer[4]:
                            newTimer[0] = newTimer[0] + msg.data[0]
                            heapq.heappush(self.__timers, newTimer)

                    self.__uninittedTimers = []

                self.__globalTime = msg.data[0]
                while len(self.__timers) > 0 and self.__timers[0][0] <= self.__globalTime:

                    # Don't do callback if timer was previously canceled
                    if not self.__timers[0][4]:
                        self.__timers[0][2]()
                    
                    tData = heapq.heappop(self.__timers)

                    # Check canceled state again because it could have been
                    # Canceled during callback
                    if not tData[4]:
                        tData[0] = tData[0] + tData[1]

                        heapq.heappush(self.__timers, tData)

            rosNode.create_subscription(Float64MultiArray, self.__simulatedTimeChannel, timerCb)

    def global_time(self):
        if self.__usingSimulatedTime:
            if self.__globalTime is None:
                return 0
            return self.__globalTime
        else:
            return time.time() - self.__instantiation

    def create_timer(self, dt_seconds, cb_function):
        if self.__usingSimulatedTime:
            timerHandle = self.__nextTimerHandle
            self.__nextTimerHandle = self.__nextTimerHandle+1

            timerDat = [dt_seconds, dt_seconds, cb_function, timerHandle, False]
            if self.__globalTime is None:
                self.__uninittedTimers.append(timerDat)
            else:
                timerDat[0] = timerDat[0] + self.__globalTime
                heapq.heappush(self.__timers, timerDat)

            return timerHandle
        else:
            return self.__rosNode.create_timer(dt_seconds, cb_function)

    def destroy_timer(self, timer_handle):
        if self.__usingSimulatedTime:
            for tmr in self.__timers:
                if tmr[3] == timer_handle:
                    tmr[4] = True
                    return True

            for tmr in self.__uninittedTimers:
                if tmr[3] == timer_handle:
                    tmr[4] = True
                    return True

            return False
        else:
            return self.__rosNode.destroy_timer(timer_handle)