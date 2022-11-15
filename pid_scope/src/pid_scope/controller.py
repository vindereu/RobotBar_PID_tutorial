from typing import Dict, Any

from pid_scope.pid import Multi_PID


class Controller(Multi_PID):
    def __init__(self, control_data: Dict[str, Any]) -> None:
        name = tuple(control_data.keys())
        Kp = []
        Ki = []
        Kd = []
        target = []
        tolerance = []
        for n in name:
            pid = control_data[n]["pid_gains"]
            Kp.append(pid[0])
            Ki.append(pid[1])
            Kd.append(pid[2])
            target.append(control_data[n]["target"]["default"])
            tolerance.append(control_data[n]["tolerance"])
        super().__init__(name, target, Kp, Ki, Kd, tolerance)
        
        for n in name:
            self.set_output_limit(n, *control_data[n]["control"]["range"])
