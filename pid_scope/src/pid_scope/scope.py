import time
import random as rng
import matplotlib.pyplot as plt
from typing import Dict, Any, List


class Scope:
    def __init__(self, control_data: Dict[str, Any]):
        self.__unbuild = True
        self.__name = tuple(control_data.keys())
        self.__t0 = time.time()
        self.__timer_start = 0
        self.__timer = 0
        self.__timeline = []
        self.__data: Dict[str, List[List[float]]] = {}  # target, sensor
        self.__color: Dict[str, List[List[float]]] = {}  # target, sensor
        self.__lower = []
        self.__upper = []
        for n in self.__name:
            self.__data[n] = [[], []]
            self.__color[n] = [[rng.random(), rng.random(), rng.random()], []]
            self.__lower.append(control_data[n]["target"]["range"][0])
            self.__upper.append(control_data[n]["target"]["range"][1])
            for i in range(3):
                self.__color[n][1].append(self.__color[n][0][i]*0.5)
        
        self.__lower = max(self.__lower)
        self.__upper = max(self.__upper)

    def __build(self):
        plt.xlim(0, 15)
        plt.ylim(self.__lower, self.__upper)
        for n in self.__name:
            plt.plot(0, self.__lower,
                color=self.__color[n][0], zorder=0, label=f"set_{n}")
            plt.plot(0, self.__lower,
                color=self.__color[n][1], zorder=1, label=n)
        plt.legend()

    def plot(self, data: Dict[str, List[float]]):
        if self.__unbuild:
            self.__build()
            self.__unbuild = False

        t = time.time() - self.__t0
        self.__timer = t - self.__timer_start
        self.__timeline.append(t)
        for n in self.__name:
            self.__data[n][0].append(data[n][0])
            self.__data[n][1].append(data[n][1])
            plt.plot(self.__timeline, self.__data[n][0],
                color=self.__color[n][0], zorder=0, label=f"set_{n}")
            plt.plot(self.__timeline, self.__data[n][1],
                color=self.__color[n][1], zorder=1, label=n)

        if self.__timer > 15:
            plt.cla()
            plt.xlim(self.__timer_start+10, self.__timer_start+25)
            plt.ylim(self.__lower , self.__upper)
            
            for n in self.__name:
                plt.plot(self.__timeline, self.__data[n][0],
                    color=self.__color[n][0], zorder=0, label=f"set_{n}")
                plt.plot(self.__timeline, self.__data[n][1],
                    color=self.__color[n][1], zorder=1, label=n)
                
                self.__data[n][0] = [self.__data[n][0][-1]]
                self.__data[n][1] = [self.__data[n][1][-1]]
            
            self.__timeline = [self.__timeline[-1]]

            plt.legend()
            self.__timer_start = t - 5
        

        

