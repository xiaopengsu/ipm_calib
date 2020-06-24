# ipm_calib_s
带有汽车图标的俯视图效果  
![带有汽车图标的俯视图效果](https://github.com/xiaopengsu/ipm_calib/blob/master/theory_image/IPM_logo.png) 
通过PNP算法求解摄像机安装位置和姿态角度_标定摄像机与自车关系  
![通过PNP算法求解摄像机安装位置和姿态角度_标定摄像机与自车关系](https://github.com/xiaopengsu/ipm_calib/blob/master/theory_image/IPM.png)  
地下车库标定效果_满足不出园区标定  
![地下车库标定效果_满足不出园区标定](https://github.com/xiaopengsu/ipm_calib/blob/master/theory_image/2019-11-13%2016-06-53%E5%B1%8F%E5%B9%95%E6%88%AA%E5%9B%BE.png)  

# 说明文档（PPT  ipm_ppt.pdf）
[](https://github.com/xiaopengsu/ipm_calib/blob/master/ipm_ppt.pdf)  

[演示视频](url smb://192.168.144.162/share/suxp/IPM_autoCalib/demo_v3/ipm_demo_v3_logo_car.avi)


# 程序运行  
./ipm_calib_video -v=../data/5.avi  
./ipm_calib_video_un -v=../data/Img297.png  
（5.avi视频没有上传）
[按“S”键完成标定]

# 俯视图测距,在辅助驾驶中主要应用:  
1 车道保持  
横向测距,在车道识别的基础上,可以实现视觉测量车载相机偏离车道的位置,完成车道偏离预警或车道保持等;  
2 前车跟随   
纵向测距,近距离可以通过识别前向车辆,再进行摄像机视野内前向测距,完成碰撞预警或车辆跟随等;  

# 升级系统  
标定时只需要拿一个设计的标定工具和表自动标定程序就能完成任务。该标定装置能折叠收纳到行李箱中，方便出差携带！  
# 摄像机安装位置和姿态角度:  
通过PNP算法求解摄像机安装位置和姿态角度（标定摄像机与自车关系）;过求解出单应性矩阵.完成车载摄像机透视图转俯视图的逆透视变换; 通过图像下侧标识靠近下边缘计算盲区距离;  

# 其他图片  
坐标变换示意图  
![坐标变换示意图](https://github.com/xiaopengsu/ipm_calib/blob/master/theory_image/IPM_s.png)  
![园区内部标定](https://github.com/xiaopengsu/ipm_calib/blob/master/theory_image/bird_view_long.png)


# H矩阵测距测试
![H矩阵测距测试效果](https://github.com/xiaopengsu/ipm_calib/blob/master/bird_view_dist.png)
get_distance_test.m //matlab的测距验证
