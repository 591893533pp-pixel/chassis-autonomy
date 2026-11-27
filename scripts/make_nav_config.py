#!/usr/bin/env python3
import json,yaml
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
old=ROOT/'build_support/nav2_legacy_template.yaml'
c=yaml.safe_load(old.read_text());g=json.loads((ROOT/'model/body_geometry.json').read_text());fp=json.dumps(g['footprint'])
# This is an isolated candidate. Navigation output is never the legacy bridge's topic.
keep=['amcl','bt_navigator','controller_server','local_costmap','global_costmap','planner_server','map_server','collision_monitor']
c={k:v for k,v in c.items() if k in keep}
a=c['amcl']['ros__parameters'];a.update(base_frame_id='base_link_trial',odom_frame_id='trial_odom',scan_topic='/chassis_nav/localization_scan',laser_min_range=.1,min_particles=1000,max_particles=4000,transform_tolerance=.2)
b=c['bt_navigator']['ros__parameters'];b.update(robot_base_frame='base_link_trial',odom_topic='/chassis_nav/odom',default_nav_to_pose_bt_xml='__CHASSIS_NAV_BT__',navigators=['navigate_to_pose']);b.pop('navigate_through_poses',None)
c['map_server']={'ros__parameters':{'yaml_filename':'','frame_id':'map','topic_name':'map'}}
d=c['controller_server']['ros__parameters'];d['odom_topic']='/chassis_nav/odom';d['progress_checker'].update(required_movement_radius=.04,movement_time_allowance=30.)
d['FollowPath'].update(min_vel_x=-.05,max_vel_x=.05,min_vel_y=-.05,max_vel_y=.05,max_speed_xy=.05,max_vel_theta=.04,acc_lim_x=.05,acc_lim_y=.05,acc_lim_theta=.08,decel_lim_x=-.1,decel_lim_y=-.1,decel_lim_theta=-.08,linear_granularity=.01,angular_granularity=.01,transform_tolerance=.2,sim_time=3.,vx_samples=9,vy_samples=9,vtheta_samples=9)
for scope in ['local_costmap','global_costmap']:
    p=c[scope][scope]['ros__parameters'];p.update(robot_base_frame='base_link_trial',global_frame='trial_odom' if scope=='local_costmap' else 'map',footprint=fp,footprint_padding=.03,resolution=.025,track_unknown_space=True)
    p.pop('robot_radius',None)
    p['inflation_layer'].update(inflation_radius=.45,cost_scaling_factor=4.)
    obs={'plugin':'nav2_costmap_2d::ObstacleLayer','enabled':True,'observation_sources':'front_left rear_right'}
    for name in ['front_left','rear_right']:
        obs[name]={'topic':f'/chassis_nav/{name}/obstacles','data_type':'PointCloud2','clearing':True,'marking':True,'observation_persistence':0.,'expected_update_rate':.3,'raytrace_min_range':.1,'raytrace_max_range':10.,'obstacle_min_range':.1,'obstacle_max_range':10.,'min_obstacle_height':-2.,'max_obstacle_height':2.}
    p['obstacle_layer']=obs
# Hybrid A* checks a non-circular footprint over heading states. Conservative
# car-like global paths initially; no claim of full lateral lattice manoeuvres.
p=c['planner_server']['ros__parameters'];p['GridBased']={'plugin':'nav2_smac_planner::SmacPlannerHybrid','tolerance':.1,'downsample_costmap':False,'allow_unknown':False,'max_iterations':1000000,'max_on_approach_iterations':1000,'max_planning_time':5.,'motion_model_for_search':'REEDS_SHEPP','angle_quantization_bins':72,'minimum_turning_radius':1.25,'reverse_penalty':2.,'change_penalty':0.,'non_straight_penalty':1.2,'cost_penalty':2.,'retrospective_penalty':.015,'analytic_expansion_ratio':3.5,'analytic_expansion_max_length':4.,'smooth_path':False}
lo=g['min_m'];hi=g['max_m'];pad=.06
stop=[[lo[0]-pad,lo[1]-pad],[hi[0]+pad,lo[1]-pad],[hi[0]+pad,hi[1]+pad],[lo[0]-pad,hi[1]+pad]]
c['collision_monitor']={'ros__parameters':{'base_frame_id':'base_link_trial','odom_frame_id':'trial_odom','cmd_vel_in_topic':'/chassis_nav/cmd_vel_raw','cmd_vel_out_topic':'/chassis_nav/cmd_vel_safe','state_topic':'/chassis_nav/collision_state','transform_tolerance':.15,'source_timeout':.3,'base_shift_correction':True,'stop_pub_timeout':1.,'enable_stamped_cmd_vel':False,'polygons':['StopFootprint'],'StopFootprint':{'type':'polygon','points':json.dumps(stop),'action_type':'stop','min_points':1,'visualize':True,'polygon_pub_topic':'/chassis_nav/stop_polygon','enabled':True},'observation_sources':['front_left','rear_right']}}
for name in ['front_left','rear_right']:
 c['collision_monitor']['ros__parameters'][name]={'type':'pointcloud','topic':f'/chassis_nav/{name}/obstacles','min_height':float(lo[2]+.02),'max_height':float(hi[2]+.05),'enabled':True}
(ROOT/'config/nav2.yaml').write_text(yaml.safe_dump(c,sort_keys=False))
lio=yaml.safe_load((ROOT/'mapping_trial/config/lio.yaml').read_text());p=lio['/**']['ros__parameters'];p.update(sam_enable=False,**{'publish/map_to_odom_tf_en':False,'lio_sam/loopClosureEnableFlag':False,'lio_sam/keyframe_export_en':False,'lio_sam/keyframe_global_pcd_en':False,'publish/path_en':False})
(ROOT/'config/localization_lio.yaml').write_text(yaml.safe_dump(lio,sort_keys=False))
if not (ROOT/'config/acceptance.json').exists(): (ROOT/'config/acceptance.json').write_text(json.dumps({'schema':'chassis_nav_acceptance_v1','motion_validated':False,'checks':{k:{'passed':False,'evidence':None} for k in ['body_anchor','physical_envelope','nearfield_obstacles','ptp_motion','map_and_relocalization','nav2_plan_and_cancel','stop_and_watchdogs']},'note':'Do not mark passed based on compilation or this CAD model. Attach target NUC test evidence for each item.'},ensure_ascii=False,indent=2)+'\n')
