#!/usr/bin/env python3
"""Display footprint and zero wheel angles only. No encoder or control functionality."""
import json,xml.etree.ElementTree as ET
from pathlib import Path
import rclpy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PolygonStamped,Point32
ROOT=Path(__file__).resolve().parents[1]
def main():
 rclpy.init();n=rclpy.create_node('chassis_footprint_display');g=json.loads((ROOT/'model/body_geometry.json').read_text());j=n.create_publisher(JointState,'/joint_states',10);p=n.create_publisher(PolygonStamped,'/chassis/footprint',10)
 tree=ET.parse(ROOT/'model/chassis_visual.urdf');names=[e.get('name') for e in tree.findall('joint') if e.get('type')=='continuous']
 def tick():
  msg=JointState();msg.header.stamp=n.get_clock().now().to_msg();msg.name=names;msg.position=[0.]*len(names);j.publish(msg)
  poly=PolygonStamped();poly.header.stamp=msg.header.stamp;poly.header.frame_id='chassis_model_base';poly.polygon.points=[Point32(x=float(x),y=float(y),z=0.) for x,y in g['footprint']];p.publish(poly)
 n.create_timer(.1,tick)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 finally:n.destroy_node();rclpy.try_shutdown()
if __name__=='__main__':main()
