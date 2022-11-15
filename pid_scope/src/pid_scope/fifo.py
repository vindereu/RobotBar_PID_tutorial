import os
import yaml
import rospkg
from typing import List, Dict, Any

def get_pkgs_path(prefix: str="") -> List[str]:
    """獲取多個ROS套件包所在路徑
    
    參數:
        prefix: 套件包前綴名稱, 預設為無前綴.
    
    輸出:
        多個路徑組成之一維串列.
    """
    rospack = rospkg.RosPack()
    pkgs: List[str] = rospack.list()
    pkgs = [name for name in pkgs if name.count(prefix)]
    return [rospack.get_path(pkg) for pkg in pkgs]

def get_pkgs_name(prefix: str="") -> List[str]:
    """獲取多個套件包名稱搭配指定前綴

    參數:
        prefix: 套件包前綴名稱, 預設為無前綴.

    輸出:
        多個套件包名稱組成之一維串列.
    """
    pkgs: List[str] = rospkg.RosPack().list()
    return [name for name in pkgs if name.count(prefix)]

def get_model_setting(pkg_path: str, yaml_path_in_pkg: str) -> Dict[str, Any]:
    """獲取指定套件包的YAML設定檔內容

    參數:
        pkg_path: 套件包路徑
        yaml_path_in_pkg: YAML檔於套件包內的相對路徑
    
    輸出:
        YAML所解析出的字典內容
    """
    file = os.path.join(pkg_path, yaml_path_in_pkg)
    if not os.path.exists(file):
        raise FileExistsError(f"Can't find {yaml_path_in_pkg} in {pkg_path}.")

    with open(file, 'r') as f:
        return yaml.safe_load(f)

def write_setting(pkg_path: str, yaml_path_in_pkg: str, data: Dict[str, Any]):
    """複寫指定YAML檔案的所有內容
    
    參數:
        pkg_path: 套件包路徑
        yaml_path_in_pkg: YAML檔於套件包內的相對路徑
        data: 要寫入的所有資料
    """
    file = os.path.join(pkg_path, yaml_path_in_pkg)
    if not os.path.exists(file):
        raise FileExistsError(f"Can't find {yaml_path_in_pkg} in {pkg_path}.")

    with open(file, "w") as f:
        yaml.dump(data, f)
