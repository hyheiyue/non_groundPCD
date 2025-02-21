# non_groundPCD 分离PCD中地面点云
## 效果展示


* 投影后的非地面点云
  
[![pElEkNV.md.png](https://s21.ax1x.com/2025/02/22/pElEkNV.md.png)](https://imgse.com/i/pElEkNV)
* 地面点云
   
[![pElEAhT.md.png](https://s21.ax1x.com/2025/02/22/pElEAhT.md.png)](https://imgse.com/i/pElEAhT)
* 非地面点云
  
[![pElEV9U.md.png](https://s21.ax1x.com/2025/02/22/pElEV9U.md.png)](https://imgse.com/i/pElEV9U)
* 程序中的可视化
  
[![pElEZ3F.md.png](https://s21.ax1x.com/2025/02/22/pElEZ3F.md.png)](https://imgse.com/i/pElEZ3F)
## 依赖
* Open3D：>= 0.10.0
* NumPy：>= 1.19.0
~~~
pip install open3d>=0.10.0 numpy>=1.19.0
~~~
##  参数
```
{   
    "input_path" :"/home/hy/rostools/output.pcd", #输入点云路径
    "roll": 3.1415926,  # 对输入pcd的rpy旋转，处理后的PCD应为正向
    "pitch": 0.0, 
    "yaw": 0.0 ,   
    "z_min": -1.0, #地面点云z坐标最小值(预处理后)
    "z_max": 0.5, #地面点云z坐标最大值(预处理后)
    "max_slope_deg": 20, #最大倾斜角度(判断斜坡)
    "max_attempts": 5, #RANSAC 平面拟合的最大尝试次数
    "distance_threshold": 0.05,  #RANSAC 平面拟合的距离阈值
    "use_projection": true #是否生成非地面投影点云
     
}
```