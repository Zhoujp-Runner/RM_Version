# Copyright(C),2018-2021,沈阳航空航天大学T-UP战队 All Rights Reserved
  
## Author:  周俊平;刘上;赵梓合;顾昊;刘彪;赵文瑄;李宇航
  
## Version: 4.3.1

### 更新内容：
1.修改了自瞄图像预处理部分，提高后续识别准确度

2.优化了串口部分，离线重连

3.自瞄与大符部分使用同一线程（通过接收下位机命令来进行模式切换）

4.自瞄部分加入数字识别，跳过2号工程机器人

5.参数整定，可通过Variables.xml文件进行参数调整

6.自瞄部分功能添加：小陀螺识别和预测
 *Data:   2022.03-2022.04

更改灯条拟合方式为椭圆拟合,进行相关修改
 *Date:  2021.05.12

卡尔曼参数调节及阈值修改,修复ROI偏移越界的bug
 *Date:  2021.05.05

更新了卡尔曼滤波相关代码；完善了和下位机的通讯机制 *Date: 2021.05.03
 *Date:  2021.05.02

更新了ArmorDetector中的阈值参数；扩大了对装甲板判定的条件范围；将能量机关识别任务独立给长焦镜头；更改编译路径为相对路径  *Date:  2021.04.10

更改了Debug.h的内容；便于调式  *Date:  2021.03.27
  
### Description: 

  沈阳航空航天大学TUP战队Robomaster 2021赛季步兵视觉程序
  
### Features：

  1.自动识别敌方车辆装甲板
  
  2.自动打击能量机关
  
  3.预测敌方车辆移动方向并预留提前量打击（waiting）
  
### Files List:
     
     1.AngelSolver 解算角度(角度解算基本上没有修改，主要是返回了PnP解算后的装甲板的世界坐标，目的主要是为了后续小陀螺识别)
     
     2.Armor 装甲板识别(此处修改了图像预处理，提高了识别效果，加入svm识别2号工程机器人，并保留装甲位置信息，进行小陀螺识别)
     
     3.Camera 驱动大恒139(功能实现：Gamma校正、色彩校正、对比度调整)
     
     4.Energy 能量机关打击
     
     6.ImageProcess 图像处理(合并能量机关和自瞄线程，简化代码)
     
     7.Serial 串口通讯(加入串口接收与发送两个独立线程，防止串口部分干扰主程序，并实现串口掉线重连)
     
     8.CmakeLists (编译配置文件)
     
     9.Debug 调试头文件
     
     10.Readme 说明文件
     
     11.config Ubuntu 16.04/18.04 LTS 快速配置命令合集
     
     12.watchDoge 防崩溃shell

### 流程框图：
总体识别框架：
![L$`YFFX1PG{`9JF(XSYL3MO](https://user-images.githubusercontent.com/92003922/161380466-acb6b8eb-d831-478a-9a37-3a398131b8c9.png)

自瞄：

![21HERRVSBB 6P_U6DB% SDE](https://user-images.githubusercontent.com/92003922/161381175-0a3dc61d-ad0c-4e84-b1e3-64f5e4be7d64.png)

移动预测：
![F8NYJL2~W@I7I)G(MGH$O1J](https://user-images.githubusercontent.com/92003922/161381201-744e5d9c-3499-4b72-b373-f820a89caf6b.png)

大符：
![HOCV1MV)V02@Z4PB{GN YX8](https://user-images.githubusercontent.com/92003922/161380996-f44ae21f-a786-4df0-a14f-3047debf5e9b.png)




