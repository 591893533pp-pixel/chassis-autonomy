CHASSIS_ID_ME — 轴向反转及质量核实版 v2
生成时间：2026-11-16T19:37:25+08:00（北京时间）
源文档：https://cad.onshape.com/documents/cb96170df12cd30700c3a5ea/w/c4ab4dbbb476ebd8a2600a19/e/7b20cc179f7c09e84d58a4e4
本次源快照：9bd1fc50832501c55e1fb165

本次修改：
四个dof_wheel_*_inv连续关节的axis全部取反；关节名、父子关系、零位和连续旋转类型保留。
同一物理姿态对应 q_v2=-q_v1；角速度与广义驱动力矩符号也相应反转。
固定关节frame_imu保留，仍连接chassis_base -> yis320。
继续按要求不添加collision，不添加仿真传感器或控制插件。

质量核实：
59个有效实体全部从最新对应源版本读取到hasMass=true；已启用useMassPropertyOverrides=true。
未指定材料、但手动赋予质量的零件，采用覆盖质量及相应质心、惯量。
以本次API数据刷新59件的质量、质心和惯量；RS02按下述已确认质量修正。
惯量张量已从源Part Studio坐标旋转到对应URDF惯性坐标系。
四个车轮各0.9 kg；YIS320为0.012 kg；MID-360各0.265 kg。
BATT_SM_L为1.14977191038427 kg；BATT_SM_R为1.14977194595290 kg。
原先未能确认质量的19个实例本次均已取得有效值。

仍需关注的源单位差异：
RS02源覆盖质量仍为0.00038 kg（0.38 g），与此前用户明确确认的0.38 kg不一致。
本包四个RS02采用0.38 kg/件，并将该源API惯量乘1000，质心不变。
请在Onshape中核对这项输入单位；本次没有修改Onshape源文档。
若完全按当前API的RS02源值计算，源总质量为19.258196733136 kg。
本包采用已确认RS02质量后的总质量为20.77667673313624572 kg。
此为CAD质量属性合计，不是实测称重。

文件：
- urdf/chassis_id_me.urdf：ROS包路径版。
- chassis_id_me.urdf：便携副本，使用相对meshes/路径。
- meshes/：25个精细STL文件，共59个实体使用。
- mass_list.txt：全部59件的最终质量表。
- mass_review.json：逐实例源记录、质量、惯量和坐标变换。
- source_mass_properties/：4份最新质量API原始响应，便于追溯。
- axis_changes.json：四轮轴向前后对照。
- export_info.json、validation.json：导出来源和验证结果。

结构：60个link（含虚拟root），59个joint（4个continuous、55个fixed）。
连续关节未设位置上下限；真实力矩、速度限制尚未提供，需要在控制/仿真配置中设置。
ROS 1：将chassis_id_me放入catkin工作空间src，依次执行catkin_make、source devel/setup.bash、roslaunch chassis_id_me chassis_id_me.launch。
包内launch适用于ROS 1；ROS 2需相应注册与启动配置。维护者邮箱为占位符。
未在外部物理引擎实际运行；尚无碰撞几何。交付文件不含API密钥。
