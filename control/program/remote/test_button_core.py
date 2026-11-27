from button_core import ButtonCore
N=[0.,0.,0.,0.,1.,1.,0.,0.]
def feed(c,t,axis=None,buttons=()):
 a=N.copy()
 if axis:
  for k,v in axis.items(): a[k]=v
 b=[0]*12
 for i in buttons:b[i]=1
 c.receive_joy(a,b,t)
 return c.output(t)
def arm(c):
 feed(c,0);feed(c,.01,buttons=(11,));feed(c,.02)
def test_sticks_ignored():
 c=ButtonCore();arm(c);assert feed(c,.1,{0:1,1:1,2:1,3:1}).x==0
 assert feed(c,.2,{0:1,1:1,2:1,3:1}).yaw==0
def test_hold_release_and_fixed_speed():
 c=ButtonCore();arm(c)
 assert feed(c,.1,{7:1}).x==.05
 assert feed(c,.4,{7:1}).x==.05
 assert feed(c,.61,{7:1}).x==.05
 assert feed(c,.7,{7:1}).x==.05
 for i in range(30):assert feed(c,.71+i*.002,{7:1}).x==.05
 feed(c,.8);assert feed(c,.9,{7:-1}).x==-.05
 assert feed(c,1.).x==0
def test_conflict_and_rotation():
 c=ButtonCore();arm(c)
 assert feed(c,.1,{7:1,6:1}).x==0
 assert feed(c,.2,{7:1}).x==0
 feed(c,.3);assert feed(c,.4,buttons=(6,)).yaw==.04
 assert feed(c,.5,buttons=(7,)).yaw==0
 feed(c,.6);assert feed(c,.7,buttons=(7,)).yaw==-.04
def test_neutral_and_estop_timeout():
 c=ButtonCore();feed(c,0,buttons=(6,));feed(c,.1,buttons=(6,11));assert not c.enabled
 c=ButtonCore();arm(c);feed(c,.1,{7:1});assert feed(c,.2,buttons=(8,9)).x==0
 assert c.estop_latched and not c.enabled
 c=ButtonCore();arm(c);feed(c,.1,{7:1});assert c.output(.7).x==0;assert not c.enabled
def test_trigger_never_boosts():
 c=ButtonCore();arm(c);assert feed(c,.1,{7:1,4:-1}).x==0

def test_all_translation_buttons_speed_and_release():
 c=ButtonCore();arm(c)
 for t,axis,sign in [(0.1,7,1),(0.3,7,-1),(0.5,6,1),(0.7,6,-1)]:
  cmd=feed(c,t,{axis:sign})
  assert (cmd.x if axis==7 else cmd.y)==sign*.05
  stopped=feed(c,t+.05)
  assert stopped.x==0 and stopped.y==0 and stopped.yaw==0
