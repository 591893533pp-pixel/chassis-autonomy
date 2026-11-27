from vendor.navslam_core import Config, TeleopCore
from test_button_core import feed,arm

def test_hold_gear_release_and_estop():
 c=TeleopCore(Config(speed_levels=(.5,.625,.75,.875,1.),max_linear_speed=.1,max_angular_speed=.2,initial_speed_level=0,button_auto=-1));arm(c)
 for i in range(100):assert abs(feed(c,.1+i*.03,{7:1}).x-.05)<1e-10
 assert feed(c,3.2).x==0
 assert abs(feed(c,3.3,{7:1},(7,)).x-.0625)<1e-10
 # Held RB changes one gear only.
 assert abs(feed(c,3.4,{7:1},(7,)).x-.0625)<1e-10
 feed(c,3.5)
 assert abs(feed(c,3.6,{7:1},(6,)).x-.05)<1e-10
 assert feed(c,3.7).x==0
 assert feed(c,3.8,{2:1}).yaw>0
 assert feed(c,3.9).yaw==0
 assert feed(c,4.,{7:1},(8,9)).x==0
 assert not c.enabled

def test_lift_is_hold_request():
 c=TeleopCore(Config(button_auto=-1));arm(c)
 assert feed(c,.1,buttons=(3,)).lift==1
 assert feed(c,.2).lift==0
 assert feed(c,.3,buttons=(4,)).lift==-1
 assert feed(c,.4).lift==0


def test_manual_005_steps_edges_and_ceiling():
 from run_remote import manual_config
 c=TeleopCore(manual_config());arm(c)
 t=.1
 for level in range(1,11):
  assert abs(feed(c,t,{7:1}).x-level*.05)<1e-10
  t+=.1
  if level<10:
   assert abs(feed(c,t,{7:1},(7,)).x-(level+1)*.05)<1e-10
   t+=.1
   assert abs(feed(c,t,{7:1},(7,)).x-(level+1)*.05)<1e-10
   t+=.1
 assert abs(feed(c,t,{7:1},(7,)).x-.5)<1e-10
 for level in range(9,0,-1):
  t+=.1;feed(c,t,{7:1})
  t+=.1;assert abs(feed(c,t,{7:1},(6,)).x-level*.05)<1e-10
 t+=.1;feed(c,t,{7:1})
 t+=.1;assert abs(feed(c,t,{7:1},(6,)).x-.05)<1e-10
 assert feed(c,t+.1).x==0


def test_manual_stick_yaw_independent_of_translation_gear():
 from run_remote import manual_config
 for gear in (1, 10, 30):
  c=TeleopCore(manual_config(1.5, gear));arm(c)
  cmd=feed(c,.1,{7:1,2:1})
  assert abs(cmd.x-gear*.05)<1e-10
  assert abs(cmd.yaw-.4)<1e-10
  assert abs(feed(c,.2,{2:-1}).yaw+.4)<1e-10
  assert abs(feed(c,.3,{2:.55}).yaw-.2)<1e-10
  assert feed(c,.4,{2:.1}).yaw==0
  assert abs(feed(c,.5,{2:1,5:-1}).yaw-.2)<1e-10
  assert abs(feed(c,.6,{2:1,4:-1}).yaw-.4)<1e-10
  assert feed(c,.7).yaw==0
  feed(c,.8,{2:1})
  assert c.output(1.4).yaw==0 and not c.enabled


def test_manual_arc_keeps_gear_scaling_and_estop():
 from run_remote import manual_config
 for gear in (1,30):
  c=TeleopCore(manual_config(1.5,gear));arm(c)
  assert abs(feed(c,.1,{7:1},(8,)).yaw-.4*.6*gear/30)<1e-10
  assert feed(c,.2,{2:1},(8,9)).yaw==0
  assert c.estop_latched and not c.enabled
