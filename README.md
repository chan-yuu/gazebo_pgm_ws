One word introduction:
Here we introduce a quick way to generate raster maps in gazebo. The previous generation generally needed to build a map through slam, which was troublesome and prone to drift. We now implement this process with a few plugins in gazebo, which make it much more logical and the maps are very standardized.

一句话介绍:
这里我们介绍一个在gazebo中快速生成栅格地图的方法。以前的生成一般需要通过slam进行建图，这个过程比较麻烦，而且容易产生漂移。我们现在通过gazebo中的一些插件来实现这个过程，变得更加合理而且地图非常标准化。

<img width="464" height="586" alt="image" src="https://github.com/user-attachments/assets/0b4c5e6c-41ec-41fb-ad97-bbdb0560bff9" />

<img width="464" height="586" alt="image" src="https://github.com/user-attachments/assets/b0d8b4de-739a-4f0d-ad29-8feba9d81a9e" />


GUI操作界面

<img width="920" height="755" alt="image" src="https://github.com/user-attachments/assets/6d7f2ce3-b0eb-44cb-8ae8-97b3163657bc" />

相关说明的博客在：

https://blog.csdn.net/hooksten/article/details/155164882?fromshare=blogdetail&sharetype=blogdetail&sharerId=155164882&sharerefer=PC&sharesource=hooksten&sharefrom=from_link


mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)


python3 gui/map_creator_gui.py


## 参数说明

-c, --corners <coords>      地图边界坐标
                            格式: (x1,y1,z1)(x2,y2,z2)
                            示例: (-10.0,-10.0,0.05)(10.0,10.0,10.0)

-r, --resolution <value>    地图分辨率（米）
                            值越小，分辨率越高，生成时间越长
                            默认: 0.01

-d, --multiplier <value>    碰撞检测距离倍数
                            默认: 0.55

-t, --threshold <value>     2D地图像素阈值 (0-255)
                            0=黑色/占用，255=白色/空闲
                            默认: 255

-f, --filename <path>       输出文件基础名称
                            会生成多个文件: .pgm, .png, .yaml, .pcd, .bt
                            默认: map

--skip-vertical-scan        跳过垂直扫描（更快，适合2D地图）


## 输出文件说明

生成的文件包括：

- **map.pgm** - 2D灰度地图（PGM格式）
- **map.png** - 2D地图PNG图像
- **map.yaml** - ROS/Nav2兼容的地图元数据
- **map.pcd** - 3D点云数据（PCL格式）
- **map.bt** - Octomap二进制文件

## 坐标系统说明

地图生成需要定义一个3D边界框：

- **Lower Right (下右角)**: 扫描区域的底部-前方-右侧角点
- **Upper Left (上左角)**: 扫描区域的顶部-后方-左侧角点

坐标约束：
- `upper_left.x > lower_right.x`
- `upper_left.y < lower_right.y`  
- `upper_left.z > lower_right.z`

示例：
```
Lower Right: (-10.0, -10.0, 0.05)  # 起始点
Upper Left:  (10.0, 10.0, 10.0)    # 结束点
```

这将创建一个 20m x 20m x 10m 的扫描区域。


## 查看生成的地图


# 查看2D地图
xdg-open map.pgm
# 或
eog map.png

# 查看3D点云
pcl_viewer map.pcd

# 在RViz中查看（如果安装了ROS）
rviz
# 然后加载map.yaml文件
