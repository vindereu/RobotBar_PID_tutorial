import time
import numpy as np
from typing import Sequence, Union

class Single_PID:
    def __init__(self, target, Kp, Ki, Kd, tolerance, useCustomTime=False):
        # Check param type
        names = "target", "Kp", "Ki", "Kd", "tolerance"
        for n, x in zip(names, (target, Kp, Ki, Kd, tolerance)):
            if not isinstance(x, (int, float)):
                raise TypeError(f"Type of the param {n} should be real number, "
                                f"not {type(x)}.")

        # PID coefficient
        self.__Kp = Kp
        self.__Ki = Ki
        self.__Kd = Kd

        # Time
        self.__lastTime = None
        self.__useCustomTime = useCustomTime

        # Target
        self.__target = target

        # Error related
        self.__tolerance = tolerance
        self.__lastData = 0
        self.__dData = 0
        self.__eSum = 0

        # Limit
        self.__I_lower = None
        self.__I_upper = None
        self.__out_lower = None
        self.__out_upper = None

    def __get_delta_time(self, customTime=None) -> Union[float, None]:
        if self.__useCustomTime:
            if customTime is None:
                raise ValueError("Custom time is required")

            if self.__lastTime is None:
                self.__lastTime = customTime
                return None

            dt = customTime - self.__lastTime
            self.__lastTime = customTime
            return dt

        else:
            current = time.time()
            if self.__lastTime is None:
                self.__lastTime = current
                return None

            dt = current - self.__lastTime
            self.__lastTime = current
            return dt

    def reset_integrator(self):
        self.__eSum = 0

    def set_I_limit(self, lower=None, upper=None):
        if not (lower is None or upper is None):
            if upper <= lower:
                raise ValueError("Value of upper must greater than lower.")

        if lower is not None:
            if self.__I_upper is not None and lower >= self.__I_upper:
                raise ValueError("Value of lower must less than upper")
            self.__I_lower = lower

        if upper is not None:
            if self.__I_lower is not None and upper <= self.__I_lower:
                raise ValueError("Value of upper must greater than lower")
            self.__I_upper = upper

    def set_output_limit(self, lower=None, upper=None):
        if not (lower is None or upper is None):
            if upper <= lower:
                raise ValueError("Value of upper must greater than lower.")

        if lower is not None:
            if self.__out_upper is not None and lower >= self.__out_upper:
                raise ValueError("Value of lower must less than upper")
            self.__out_lower = lower

        if upper is not None:
            if self.__out_lower is not None and upper <= self.__out_lower:
                raise ValueError("Value of upper must greater than lower")
            self.__out_upper = upper

    def set_PID(self, Kp=None, Ki=None, Kd=None):
        names = ("Kp", "Ki", "Kd")
        for n, x in zip(names, (Kp, Ki, Kd)):
            if not isinstance(x, [int, float, type(None)]):
                raise TypeError(f"Type of the param {n} should be float, not {type(x)}.")

        if Kp is not None:
            self.__Kp = Kp

        if Ki is not None:
            self.__Ki = Ki

        if Kd is not None:
            self.__Kd = Kd

    def set_target(self, value):
        if not isinstance(value, (int, float)):
            raise TypeError(f"Type of the param value should be float, not {type(value)}.")

        self.__target = value

    def update(self, data, external_time=None) -> float:
        # Time
        dt = self.__get_delta_time(external_time)
        if dt is None:
            return 0

        # Error
        error = self.__target - data
        if abs(error) <= self.__tolerance:
            error = 0

        # Integrator
        self.__eSum += error * dt
        if self.__I_lower is not None and self.__eSum < self.__I_lower:
            self.__eSum = self.__I_lower

        elif self.__I_upper is not None and self.__eSum > self.__I_upper:
            self.__eSum = self.__I_upper
        
        # Derivative
        try:
            self.__dData = (data - self.__lastData) / dt

        except ZeroDivisionError:
            pass

        self.__lastData = data
        

        # PID Output
        output = (self.__Kp * error
                  + self.__Ki * self.__eSum
                  - self.__Kd * self.__dData)

        if self.__out_lower is not None and output < self.__out_lower:
            output = self.__out_lower

        elif self.__out_upper is not None and output > self.__out_upper:
            output = self.__out_upper

        return output


class Multi_PID:
    def __init__(self, names: Sequence[str], targets: Sequence[float],
                       Kp: Sequence[float], Ki: Sequence[float],
                       Kd: Sequence[float], tolerances: Sequence[float]):

        self.__index = dict(zip(names, range(len(names))))
        self.__target = np.array(targets, dtype=np.float64)
        self.__tolerance = tuple(tolerances)
        self.__pid = np.array([*zip(Kp, Ki, Kd)], dtype=np.float64)
        self.__eSum = np.array((0,) * len(names), dtype=np.float64)
        self.__last_data = np.array((0,) * len(names), dtype=np.float64)
        self.__last_dData = np.array((0,) * len(names), dtype=np.float64)
        self.__I_limit = np.array(((np.nan, np.nan),) * len(names), dtype=np.float64)
        self.__out_limit = np.array(((np.nan, np.nan),) * len(names), dtype=np.float64)
        self.__last_time = None

    def __get_delta_time(self, ex_time=None) -> float:
        """????????????????????????.

        ??????:
            ex_time: ????????????, ?????????None, ????????????????????????????????????.
        
        ??????:
            ?????????.
        """
        if ex_time is not None:
            if self.__last_time is None:
                self.__last_time = ex_time
                return

            dt = ex_time - self.__last_time
            self.__last_time = ex_time
            return dt

        current = time.time()
        if self.__last_time is None:
            self.__last_time = current
            return

        dt = current - self.__last_time
        self.__last_time = current
        return dt

    def reset_integral(self, names: Union[str, Sequence[str]]):
        """????????????????????????.

        ??????:
            names: PID????????????.
        """
        if isinstance(names, str):
            self.__eSum[self.__index[names]] = 0
            return

        for name in names:
            self.__eSum[self.__index[name]] = 0

    def set_I_limit(self, name, lower=None, upper=None):
        """????????????????????????.
        ??????:
            name: PID????????????.
            lower: ???????????????, ???????????????.
            upper: ???????????????, ???????????????.
        """
        if lower is not None and upper is not None:
            if upper <= lower:
                raise ValueError("Value of upper must greater than lower.")

            self.__I_limit[self.__index[name]][0] = lower
            self.__I_limit[self.__index[name]][1] = upper

        elif lower is not None:
            self.__I_limit[self.__index[name]][0] = lower

        elif upper is not None:
            self.__I_limit[self.__index[name]][1] = upper

    def set_output_limit(self, name, lower=None, upper=None):
        """??????????????????.

        ??????:
            name: PID????????????.
            lower: ???????????????, ???????????????.
            upper: ???????????????, ???????????????.
        """
        if lower is not None and upper is not None:
            if upper <= lower:
                raise ValueError("Value of upper must greater than lower.")

            self.__out_limit[self.__index[name]][0] = lower
            self.__out_limit[self.__index[name]][1] = upper

        elif lower is not None:
            self.__out_limit[self.__index[name]][0] = lower

        elif upper is not None:
            self.__out_limit[self.__index[name]][1] = upper

    def set_PID(self, name, Kp=None, Ki=None, Kd=None):
        """??????PID??????

        ??????:
            name: PID????????????.
            Kp: ??????????????????, ???????????????.
            Ki: ??????????????????, ???????????????.
            Kd: ??????????????????, ???????????????.
        """
        names = ("Kp", "Ki", "Kd")
        for n, x in zip(names, (Kp, Ki, Kd)):
            if not isinstance(x, (int, float, type(None))):
                raise TypeError(f"Type of the param {n} should be float, not {type(x)}.")

        if Kp is not None:
            self.__pid[self.__index[name]][0] = Kp

        if Ki is not None:
            self.__pid[self.__index[name]][1] = Ki

        if Kd is not None:
            self.__pid[self.__index[name]][2] = Kd
        
    def set_target(self, name, value):
        """???????????????.

        ??????:
            name: PID????????????.
            value: ????????????.
        """
        if not isinstance(value, (int, float)):
            raise TypeError(f"Type of the param value should be float, not {type(value)}.")

        self.__target[self.__index[name]] = value

    def update(self, data: Sequence[float], mode: str="PID") -> 'dict[str, float]':
        """??????PID???????????????????????????????????????

        ??????:
            data: ?????????, ????????????????????????????????????????????????
        
        ??????:
            ?????????, ???????????????
        """
        ### Time ###
        dt = self.__get_delta_time()
        if dt is None:
            return dict(zip(self.__index, (0,) * len(self.__index)))

        ### Error ###
        data = np.array(data)
        error = self.__target - data
        error[np.where(abs(error) <= self.__tolerance)] = 0

        ### Integral ###
        self.__eSum = self.__eSum  + error * dt
        self.__eSum = np.nanmax((self.__eSum, self.__I_limit.T[0]), axis=0)  # ???????????????
        self.__eSum = np.nanmin((self.__eSum, self.__I_limit.T[1]), axis=0)  # ???????????????

        ### Derivative ###
        dData = (data - self.__last_data) / dt
        nan_indexes = np.where(np.isnan(dData))
        if tuple(nan_indexes[0]) != ():
            dData[nan_indexes] = self.__last_dData.copy()
        self.__last_dData = dData
        self.__last_data = data
        ### PID ###
        output = np.sum(self.__pid * np.array((error, self.__eSum, -dData)).T, axis=1)
        output = np.nanmax((output, self.__out_limit.T[0]), axis=0)  # ???????????????
        output = np.nanmin((output, self.__out_limit.T[1]), axis=0)  # ???????????????

        ### Dictionary ###
        return dict(zip(self.__index, output))
