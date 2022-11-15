#!/usr/bin/env python3
# pip
import os
import re
import tkinter as tk
import matplotlib
import matplotlib.pyplot as plt
from tkinter import ttk
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from typing import List, Dict, Any
import multiprocessing

# ROS
import rospy

# pid_scope
import pid_scope.model_manage as model_manage
import pid_scope.fifo as fifo
from pid_scope.controller import Controller
from pid_scope.scope import Scope
from pid_scope.msg import Sensor, Control


class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.__index = 0  # model selection
        self.__pkg_data = []  # model settings
        self.__pkg_path = []
        self.__images = []  # model snapshot

        # Grab model data
        pkg_path = fifo.get_pkgs_path("robotbar_pid")
        self.__load_setting(*pkg_path)

        # Create GUI and ROS node
        rospy.init_node("PID_scope")
        self.__build_menu()
        self.mainloop()

    def __load_setting(self, *pkg_path):
        for i in range(len(pkg_path)):
            self.__pkg_data.append(fifo.get_model_setting(pkg_path[i], "docs/setting.yaml"))
            self.__pkg_path.append(os.path.basename(pkg_path[i]))
            image_path = os.path.join(pkg_path[i], self.__pkg_data[i]["model"]["snapshot"])
            self.__images.append(tk.PhotoImage(file=image_path))

    def __build_menu(self):
        # Window setting
        icon_path = os.path.join(os.path.dirname(__file__), "img/robotbar_pid.png")
        self.iconphoto(True, tk.PhotoImage(file=icon_path))
        self.title("menu")
        self.wm_protocol("WM_DELETE_WINDOW", self.__close)

        # Widget setting
        frame = tk.Frame(self)

        label_text = tk.Label(frame, font=("Consolas", 20, "bold"))
        label_text.config(text=self.__pkg_data[self.__index]["model"]["description"])
        label_text.grid(row=0, column=0, columnspan=3, pady=5)

        label_image = tk.Label(frame, image=self.__images[self.__index])
        label_image.grid(row=1, column=0, columnspan=3, pady=[0, 5])

        label_index = tk.Label(frame, text=f"{self.__index+1} / {len(self.__images)}")
        label_index.grid(row=3, column=0, columnspan=3, sticky="we")

        button = tk.Button(frame, text="<")
        button.config(command=lambda: self.__switch(label_text, label_image, label_index, -1))
        button.grid(row=2, column=0)
        
        button = tk.Button(frame, text="Select")
        button.config(command=self.__build_scope)
        button.grid(row=2, column=1, padx=5)
        
        button = tk.Button(frame, text=">")
        button.config(command=lambda: self.__switch(label_text, label_image, label_index, 1))
        button.grid(row=2, column=2)

        frame.pack(fill=tk.BOTH)

    def __build_scope(self):
        window = ScopeWindow(self.__pkg_path[self.__index] ,self.__pkg_data[self.__index])
        self.withdraw()
        self.wait_window(window)
        self.deiconify()

    def __close(self):
        os.system("rosnode kill /gazebo_gui")
        self.destroy()
        self.quit()

    def __switch(self, text_label: tk.Label, img_label: tk.Label,
                index_label: tk.Label, direction: int):
        self.__index += direction
        if self.__index == len(self.__images):
            self.__index = 0
        elif self.__index < 0:
            self.__index = len(self.__images) - 1

        # Update display
        text_label.config(text=self.__pkg_data[self.__index]["model"]["description"])
        img_label.config(image=self.__images[self.__index])
        index_label.config(text=f"{self.__index+1} / {len(self.__images)}")


class ScopeWindow(tk.Toplevel):
    def __init__(self, pkg_path: str, data: Dict[str, Any]):
        super().__init__()
        model_manage.spawn_model(os.path.basename(pkg_path), data["model"]["spawn"])
        
        self.__font = ("Consolas", 15)
        controlData: Dict = data["controller_data"]
        self.__data = data
        self.__name = tuple(controlData.keys())
        self.__pid: Dict[str, List[float]] = {}
        self.__target: Dict[str, Any] = {}
        self.__control_msg = Control()
        self.__sensor_msg = Sensor()
        self.__sub = rospy.Subscriber(data["program"]["topic"], Sensor, self.__ros_callback)
        self.__pub = rospy.Publisher("/pid_scope/control_data", Control, queue_size=10)

        for n in self.__name:
            self.__pid[n] = controlData[n]["pid_gains"]
            self.__target[n] = controlData[n]["target"]
            self.__target[n]["value"] = self.__target[n]["default"]  # add value key
            self.__sensor_msg.name.append(n)
            self.__sensor_msg.data.append(self.__target[n]["value"])

        self.__controller = Controller(data["controller_data"])
        self.__scope = Scope(data["controller_data"])
        self.__p = multiprocessing.Process(target=self.__worker, args=(os.path.basename(pkg_path),))
        self.__p.start()
        self.__build()

    def __worker(self, name):
        os.system(f"rosrun {name} {self.__data['program']['file']}")
        pass

    def __build(self):
        # Window setting
        self.title("PID tuning scope")
        self.geometry("950x480")
        self.resizable(False, False)
        self.wm_protocol("WM_DELETE_WINDOW", self.__close)
        
        # Frame setting
        matplotlib.use("TkAgg")
        self.__scope_frame_setting(self)
        self.__set_frame_setting(self)

        # Timer
        #self.__control_thread.start()
        self.after(10, self.__update_control)  # 10ms
        self.after(10, self.__update_plot)

    def __close(self):
        self.__sub.unregister()
        self.__pub.unregister()
        os.system(f"rosnode kill {self.__data['program']['node']}")
        if isinstance(self.__data["model"]["name"], str):
            model_manage.delete_model(self.__data["model"]["name"])
        else:
            for name in self.__data["model"]["name"]:
               model_manage.delete_model(name) 
        self.destroy()

    def __pid_frame_setting(self, master):
        pid_frame = tk.Frame(master)
        # PID entry
        ttk.Style().configure('TNotebook.Tab', font=self.__font)
        pid_page = ttk.Notebook(pid_frame, padding=15)
        pid_entry: Dict[str, List[tk.Entry]] = {}
        vcmd = (self.register(self.__positive_num_validate), "%P")
        for page_name in self.__name:
            entry_frame = tk.Frame(pid_page)

            pid_init = self.__pid[page_name]
            pid_entry[page_name] = []
            for idx, name in enumerate(("Kp", "Ki", "Kd")):
                label = tk.Label(entry_frame, text=name, font=self.__font)
                label.grid(row=idx, column=0, pady=1, padx=(5, 10))
                
                pid_entry[page_name].append(
                    tk.Entry(entry_frame, font=self.__font))
                pid_entry[page_name][idx].config(width=15)
                pid_entry[page_name][idx].config(
                    textvariable=tk.StringVar(value=str(pid_init[idx])), validate="key", vcmd=vcmd)
                pid_entry[page_name][idx].grid(row=idx, column=1, pady=1)

            entry_frame.pack()
            pid_page.add(entry_frame, text=f"{page_name:^3}")
        
        pid_page.grid(row=0, column=0, columnspan=10)

        # PID set button
        set_button = tk.Button(pid_frame, text="Set PID")
        set_button.config(command=lambda: self.__pid_set(pid_entry))
        set_button.grid(row=1, column=4)

        save_button = tk.Button(pid_frame, text="Save")
        save_button.config(command=self.__pid_save)
        save_button.grid(row=1, column=5)

        pid_frame.pack(side=tk.TOP, pady=20)

    def __pid_save(self):
        path = os.path.join(fifo.get_pkgs_path(self._), "docs/setting.yaml")
        fifo.write_setting(path, self.__data)

    def __pid_set(self, object: Dict[str, List[tk.Entry]]):
        for name in object:
            self.__pid[name] = []
            for entry in object[name]:
                if entry.get() == '':
                    entry.insert(0, '0')
                
                self.__pid[name].append(float(entry.get()))
            self.__controller.set_PID(name, *self.__pid[name])
            self.__controller.reset_integral(name)
            self.__data["controller_data"][name]["pid_gains"] = self.__pid[name]

    def __positive_num_validate(self, string: str):
        result = re.match(r"[0-9.]*$", string)
        return result is not None and string.count('.') <= 1

    def __ros_callback(self, msg: Sensor):
        data = []
        # sort
        for name in self.__name:
            data.append(msg.data[msg.name.index(name)])
        self.__sensor_msg.data = data

    def __scope_frame_setting(self, master):
        scope_frame = tk.Frame(master)
        scope_frame.config(borderwidth=3, relief="groove")
        figure = plt.figure()
        self.__canvas = FigureCanvasTkAgg(figure, master=scope_frame)
        self.__canvas.get_tk_widget().pack(padx=5, pady=5)
        scope_frame.pack(side=tk.LEFT, fill=tk.BOTH)
    
    def __set_frame_setting(self, master):
        set_frame = tk.Frame(master)
        self.__pid_frame_setting(set_frame)
        self.__setpoint_frame_setting(set_frame)
        set_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=1)

    def __setpoint_frame_setting(self, master):
        setpoint_frame = tk.Frame(master)

        scale: Dict[str, tk.Scale] = {}
        for idx, name in enumerate(self.__name):
            label = tk.Label(setpoint_frame, text=f"{name}({self.__target[name]['unit']})",
                font=self.__font)

            label.grid(row=idx, column=0, padx=(0, 10))

            scale[name] = tk.Scale(setpoint_frame, orient=tk.HORIZONTAL, length=150)
            scale[name].config(resolution=self.__target[name]["resolution"],
                from_=self.__target[name]["range"][0],
                to=self.__target[name]["range"][1])
            scale[name].set(self.__target[name]["default"])
            scale[name].grid(row=idx, column=1)

        button = tk.Button(setpoint_frame, text="Set Target")
        button.config(command=lambda: self.__target_set(scale))
        button.grid(row=len(self.__name), column=0, columnspan=2, pady=15)

        setpoint_frame.pack(pady=20)

    def __target_set(self, object: Dict[str, tk.Scale]):
        for name in object:
            self.__target[name]["value"] = object[name].get()
            self.__controller.set_target(name, self.__target[name]["value"])

    def __update_control(self):
        output = self.__controller.update(self.__sensor_msg.data)
        self.__control_msg.name = tuple(output.keys())
        for name in output:
            self.__control_msg.value.append(output[name])

        self.__pub.publish(self.__control_msg)
        self.__control_msg.value = []
        self.after(10, self.__update_control)

    def __update_plot(self):
        plot_data = {}
        for idx, n in enumerate(self.__sensor_msg.name):
            plot_data[n] = [self.__target[n]["value"], self.__sensor_msg.data[idx]]
        self.__scope.plot(plot_data)
        self.__canvas.draw()
        self.after(50, self.__update_plot)

if __name__ == "__main__":
    App()