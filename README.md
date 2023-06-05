# biped_mpc

功能说明：基于单刚体模型，使用mpc算法优化双足机器人足底力。

运行环境：Matlab2017b

使用方法：运行main_mpc_controller.m

代码说明：
（1）控制方案参考论文
 Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control
（2）hoppingDynamics.m为单刚体动力学模型
（3）Mpc参考轨迹由gen_ref.m给出
（4）摆动腿轨迹由gen_paraboal.m给出
（5）draw_run.m为可视化程序
