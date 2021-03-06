```python
%load_ext autoreload
%autoreload 2
# pull in new changes to python modules without having to restart notebook

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import json 
from jsbsim_utils import Logger, trim, simulate, linearize, rootlocus, clean_tf
import control 
```


```python
all_data = dict()
```

Nonlinear-dynamics of aircraft

$$\dot{x} = f(x, u)$$

Find equilibrium:

Minimize $\dot{x}$, or a subset for your problem, by changeing $x_0$, $u_0$

$$0 = f(x_0, u_0)$$

This is an optimization problem.

* Matlab: fmincon, fminsearch
* Python: scipy.minimize (Nelder-Mead, SLSQP)

Can simplify this problem with mixing matrices, to decouple the dynamcis into SISO (single-input-single output) systems.

Due to Taylor series error on the order of (dx^2). We know the linear model approx, does well near the equilibrium point. 

$\dot{\vec{x}} = \vec{f}(x, u) = A \vec{x} + B \vec{u}$

$\dot{\vec{y}} = C \vec{x} + D \vec{u}$


$A = \dfrac{\delta \vec{f}(x0, u0)}{\delta \vec{x}}$

$B = \dfrac{\delta \vec{f}(x0, u0)}{\delta \vec{u}}$

$C = \dfrac{\delta \vec{y}(x0, u0)}{\delta \vec{x}}$

$D = \dfrac{\delta \vec{y}(x0, u0)}{\delta \vec{u}}$

## Ground Trimming 


```python
ct = 0
temp = []
def cost_ground(fdm):
    # modify cost to also penalize throttle   
    udot = fdm['accelerations/udot-ft_sec2']
    vdot = fdm['accelerations/vdot-ft_sec2']
    wdot = fdm['accelerations/wdot-ft_sec2']
    pdot = fdm['accelerations/pdot-rad_sec2']
    qdot = fdm['accelerations/qdot-rad_sec2']
    rdot = fdm['accelerations/rdot-rad_sec2']
    temp.append(fdm.get_property_catalog('fbz'))
    return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2  
        
op_ground, props, res = trim(
    aircraft='F-35B-2',
    ic={
          'ic/vt-fps': 0,
          'ic/psi-true-deg': 280,
          'ap/gear-enable': 1,
          'fcs/left-brake-cmd-norm': 1,
          'fcs/right-brake-cmd-norm': 1,
          'fcs/center-brake-cmd-norm': 1,
#         'ic/vt-fps': 0,
#         'gear/gear-cmd-norm': 1,
#         'propulsion/engine/pitch-angle-rad': np.deg2rad(0),
#         'fcs/throttle-cmd-norm': 0,
#         'fcs/aileron-cmd-norm': 0,
#         'fcs/elevator-cmd-norm': 0,
#         'fcs/rudder-cmd-norm': 0,
#         'fcs/left-brake-cmd-norm': 1,
#         'fcs/right-brake-cmd-norm': 1,
#         'fcs/center-brake-cmd-norm': 1,
    },
    design_vector=['ic/theta-rad', 'ic/h-agl-ft'],
    x0=[0, 0.31],
    verbose=True,
    method='Nelder-Mead', # works better with ground interaction
    tol=1e-12,
    #bounds=[[np.deg2rad(-40), np.deg2rad(40)], [0, 20]],
    cost=cost_ground,
)

print(res)
print(op_ground)
print(temp[0])
print(temp[-1])

data = dict()
data['res'] = res
data['op_ground'] = op_ground
data['fbz_initial'] = temp[0]
data['fbz_final'] = temp[-1]
all_data = dict()
all_data['groundTrim'] = data
```


```python
log_ground = simulate(
    aircraft='F-35B-2',
    op_0=op_ground,
    tf=5,
    realtime=True)
```


```python
log_ground['position/h-agl-ft'].plot()
plt.grid(True)
plt.ylabel('ft, altitude')
```


```python
log_ground['attitude/theta-deg'].plot()
plt.grid(True)
plt.ylabel('deg, aircraft pitch')
```

## Hover Trimming


```python
op_hover, props, res = trim(
    aircraft='F-35B-2',
    ic={  
          'ic/h-sl-ft': 650,
          'ic/vt-fps': 0,
          'ic/psi-true-deg': 0,
          'ap/gear-enable': 1,
#         'ic/h-agl-ft': 10,
#         'ic/vd-fps': 0,
#         'ic/vn-fps': 0*np.cos(np.deg2rad(280)),
#         'ic/ve-fps': 0*np.sin(np.deg2rad(280)),
#         'ic/theta-rad': 0,
#         'gear/gear-cmd-norm': 1,
#         'fcs/left-brake-cmd-norm': 0,
#         'fcs/right-brake-cmd-norm': 0,
#         'fcs/center-brake-cmd-norm': 0,
    },
    eq_constraints = [
        lambda fdm: fdm['accelerations/udot-ft_sec2'],
        #lambda fdm: fdm['accelerations/vdot-ft_sec2'],
        lambda fdm: fdm['accelerations/wdot-ft_sec2'],
        #lambda fdm: fdm['accelerations/pdot-rad_sec2'],
        lambda fdm: fdm['accelerations/qdot-rad_sec2'],
        #lambda fdm: fdm['accelerations/rdot-rad_sec2'],
    ],
    design_vector=[
        'fcs/throttle-cmd-norm',
        'fcs/elevator-cmd-norm',
        'propulsion/engine/pitch-angle-rad',
        'propulsion/engine[1]/pitch-angle-rad',
        'propulsion/engine[2]/pitch-angle-rad',
        'propulsion/engine[3]/pitch-angle-rad',
    ],
#     design_vector=[
#         'fcs/throttle-cmd-norm',
#         'fcs/elevator-cmd-norm',
#         'propulsion/engine/pitch-angle-rad',
#         'propulsion/engine[1]/pitch-angle-rad',
#         'propulsion/engine[2]/pitch-angle-rad',
#         'propulsion/engine[3]/pitch-angle-rad',
#     ],
    x0=[0.9, 0.2, np.deg2rad(90), np.deg2rad(90), np.deg2rad(90), np.deg2rad(90)],
    cost= lambda fdm: fdm['fcs/throttle-cmd-norm'],
    verbose=True,
    method='SLSQP',
    bounds=[[0, 1], [-1, 1], [np.deg2rad(0), np.deg2rad(120)], 
           [np.deg2rad(0), np.deg2rad(120)],
           [np.deg2rad(0), np.deg2rad(120)],
           [np.deg2rad(0), np.deg2rad(120)]],
    tol=1e-12
)

print(op_hover)
data1 = dict()
data1['res'] = res
data1['op_hover'] = op_hover
all_data['hoverTrim'] = data1
```


```python
log_hover = simulate(
    aircraft='F-35B-2',
    op_0=op_hover,
    tf=10,
    realtime=True)
```


```python
log_hover['position/h-agl-ft'].plot()
plt.grid(True)
plt.ylabel('ft, altitude')
```


```python
plt.subplot()
log_hover['propulsion/engine/thrust-lbs'].plot()
log_hover['propulsion/engine[1]/thrust-lbs'].plot()
log_hover['propulsion/engine[2]/thrust-lbs'].plot()
log_hover['propulsion/engine[3]/thrust-lbs'].plot()
plt.grid(True)
plt.legend(['rcp', 'rhp', 'lcp', 'lhp'])
plt.ylabel('lbs, engine thrusts')
plt.show()
```


```python
plt.subplot()
log_hover['propulsion/engine/pitch-angle-rad'].plot()
log_hover['propulsion/engine[1]/pitch-angle-rad'].plot()
log_hover['propulsion/engine[2]/pitch-angle-rad'].plot()
log_hover['propulsion/engine[3]/pitch-angle-rad'].plot()
plt.grid(True)
plt.legend(['rcp', 'rhp', 'lcp', 'lhp'])
plt.ylabel('deg, engine pitch angle')
plt.show()
```

## Hover Auto Pilot


```python
op_hover_auto = dict(op_hover)
op_hover_auto['ic/theta-deg'] = 5
op_hover_auto['ic/phi-deg'] = 5

op_hover_auto['ap/heading-cmd-deg'] = 0
op_hover_auto['ap/gear-enable'] = 1
op_hover_auto['ap/roll-enable'] = 1
op_hover_auto['ap/pitch-enable'] = 1
op_hover_auto['ap/yaw-enable'] = 0
op_hover_auto['ap/h-enable'] = 1
op_hover_auto['ap/h-sl-cmd-ft'] = 650

log_hover_auto = simulate(
    aircraft='F-35B-2',
    op_0=op_hover_auto,
    tf=20,
    realtime=False)
```


```python
log_hover_auto['fcs/throttle-pos-norm'].plot(label='0')
log_hover_auto['fcs/throttle-pos-norm[1]'].plot(label='1')
log_hover_auto['fcs/throttle-pos-norm[2]'].plot(label='2')
log_hover_auto['fcs/throttle-pos-norm[3]'].plot(label='3')

plt.legend()
```


```python
log_hover_auto['fcs/elevator-cmd-norm'].plot(label='0')
log_hover_auto['fcs/aileron-cmd-norm'].plot(label='1')
log_hover_auto['fcs/rudder-cmd-norm'].plot(label='2')
log_hover_auto['fcs/throttle-cmd-norm'].plot(label='3')

plt.legend()
```


```python
plt.subplot()
log_hover_auto['propulsion/engine/pitch-angle-rad'].plot()
log_hover_auto['propulsion/engine[1]/pitch-angle-rad'].plot()
log_hover_auto['propulsion/engine[2]/pitch-angle-rad'].plot()
log_hover_auto['propulsion/engine[3]/pitch-angle-rad'].plot()
plt.grid(True)
plt.legend(['rcp', 'rhp', 'lcp', 'lhp'])
plt.ylabel('deg, engine pitch angle')
plt.show()
```

## Cruise Trimming


```python
def cost_cruise(fdm):
    # modify cost to also penalize throttle
    delta = fdm['propulsion/engine/pitch-angle-rad']
    theta = fdm['attitude/theta-rad']
    drag = fdm['forces/fwx-aero-lbs']
    lift = fdm['forces/fwz-aero-lbs']
    alpha = fdm['aero/alpha-rad']
    throttle = fdm['fcs/throttle-cmd-norm']
    
    udot = fdm['accelerations/udot-ft_sec2']
    vdot = fdm['accelerations/vdot-ft_sec2']
    wdot = fdm['accelerations/wdot-ft_sec2']
    pdot = fdm['accelerations/pdot-rad_sec2']
    qdot = fdm['accelerations/qdot-rad_sec2']
    rdot = fdm['accelerations/rdot-rad_sec2']
    return udot**2 + vdot**2 + wdot**2 + pdot**2 + qdot**2 + rdot**2 - 1e-3*(lift/drag)**2 + 1e-3*(theta < 0) + 1e-1*throttle + 2e-1*delta**2 
        

op_cruise, prop, res = trim(
    aircraft='F-35B-2',
    ic={
        'ic/gamma-rad': 0,
        'ic/vt-fps': 677,
        'ic/h-agl-ft': 38*1e3,
        'gear/gear-cmd-norm': 0,
        'fcs/left-brake-cmd-norm': 0,
        'fcs/right-brake-cmd-norm': 0,
        'fcs/center-brake-cmd-norm': 0,
        'propulsion/engine/pitch-angle-rad': 0,
        'propulsion/engine[1]/pitch-angle-rad': 0,
        'propulsion/engine[2]/pitch-angle-rad': 0,
        'propulsion/engine[3]/pitch-angle-rad': 0,
    },
#     design_vector=[
#         'fcs/throttle-cmd-norm',
#         'fcs/elevator-cmd-norm',
#         'fcs/rudder-cmd-norm',
#         'fcs/aileron-cmd-norm',
#         'ic/alpha-rad',
#         'ic/beta-rad',
#         'propulsion/engine/pitch-angle-rad',
#     ],
#     cost=cost_cruise,
    
    design_vector=[
        'fcs/throttle-cmd-norm',
        'fcs/elevator-cmd-norm',
        'fcs/rudder-cmd-norm',
        'fcs/aileron-cmd-norm',
        'ic/alpha-rad',
        'ic/beta-rad',
    ],
    method='SLSQP',
    eq_constraints= [
        lambda fdm: fdm['accelerations/udot-ft_sec2'],
        lambda fdm: fdm['accelerations/vdot-ft_sec2'],
        lambda fdm: fdm['accelerations/wdot-ft_sec2'],
        lambda fdm: fdm['accelerations/pdot-rad_sec2'],
        lambda fdm: fdm['accelerations/qdot-rad_sec2'],
        lambda fdm: fdm['accelerations/rdot-rad_sec2'],
    ],
#     cost=lambda fdm: fdm['fcs/throttle-cmd-norm'],
    cost=cost_cruise,
    x0=[0.4, 0, 0, 0, 0, 0],
    verbose=True,
    bounds=[[0, 1], [-1, 1], [-1, 1], [-1, 1], [-1, 1], [-1, 1]],
    tol=1e-12,
)
print(op_cruise)

data2 = dict()
data2['res'] = res
data2['op_cruise'] = op_cruise

all_data['cruiseTrim'] = data2
```


```python
log_cruise = simulate(
    aircraft='F-35B-2',
    op_0=op_cruise,
    tf=10,
    realtime=True)
```


```python
plt.subplot()
log_cruise['propulsion/engine/pitch-angle-rad'].plot()
log_cruise['propulsion/engine[1]/pitch-angle-rad'].plot()
log_cruise['propulsion/engine[2]/pitch-angle-rad'].plot()
log_cruise['propulsion/engine[3]/pitch-angle-rad'].plot()
plt.grid(True)
plt.legend(['rcp', 'rhp', 'lcp', 'lhp'])
plt.ylabel('lb, pounds')
plt.show()
```


```python
log_cruise['forces/fwx-aero-lbs'].plot()
plt.ylabel('lbs, drag')
plt.grid(True)
```


```python
log_cruise['forces/fwy-aero-lbs'].plot()
plt.grid(True)
plt.ylabel('lbs, side-force')
```


```python
log_cruise['forces/fwz-aero-lbs'].plot()
plt.grid(True)
plt.ylabel('lbs, lift')
```


```python
log_cruise['aero/alpha-deg'].plot()
plt.ylabel('deg, angle of attack')
plt.grid(True)
```


```python
log_cruise['fcs/elevator-pos-deg'].plot()
plt.grid(True)
plt.ylabel('deg, elevator angle')
```


```python
plt.subplot()
log_cruise['fcs/left-aileron-pos-deg'].plot()
log_cruise['fcs/right-aileron-pos-deg'].plot()
plt.grid(True)
plt.legend(['left aileron', 'right aileron'])
plt.ylabel("deg, aileron angle")
plt.show()
```


```python
log_cruise['fcs/rudder-pos-deg'].plot()
plt.grid(True)
plt.ylabel('deg, rudder angle')
```


```python
log_cruise['velocities/vt-fps'].plot()
plt.grid(True)
plt.ylabel("fps, velocity")
```


```python
del all_data['cruiseTrim']['res']['hess_inv']
```


```python
del all_data['hoverTrim']['res']['hess_inv']
```


```python
del all_data['cruiseTrim']['res']['message']
del all_data['groundTrim']['res']['message']
del all_data['hoverTrim']['res']['message']
```


```python
# encode numpy array for JSON serializable
class NumpyEncoder(json.JSONEncoder):
    def default(self, obj):
        if isinstance(obj, np.ndarray):
            return obj.tolist()
        return json.JSONEncoder.default(self, obj)

# save all output as JSON file 
with open('jsbsim_opt.json', '+w') as jfile:
    json.dump(all_data, jfile, indent=4, cls=NumpyEncoder)
```


```python
result = log_cruise.to_json(orient="split")
parsed = json.loads(result)
with open('log_cruise.json', 'w') as jfile: 
    json.dump(parsed, jfile, indent=4)
```


```python
result = log_ground.to_json(orient="split")
parsed = json.loads(result)
with open('log_ground.json', 'w') as jfile: 
    json.dump(parsed, jfile, indent=4)
```


```python
result = log_hover.to_json(orient="split")
parsed = json.loads(result)
with open('log_hover.json', 'w') as jfile: 
    json.dump(parsed, jfile, indent=4)
```

## Transition 


```python
def trim_transition(vt_fps, gamma_deg, accel_g):
    print('trimming @ Vt=', vt_fps, 'fps', 'gamma = ', gamma_deg, 'deg')
    
    def accel_gamma(fdm, accel_g, gamma_deg):
        gamma = np.deg2rad(gamma_deg)
        g = 32.2
        theta = fdm['attitude/theta-rad']
        C_nb = np.array([
            [np.cos(theta), -np.sin(theta)],
            [np.sin(theta), np.cos(theta)]
        ])
        a_n = accel_g*g*np.array([np.cos(gamma), np.sin(gamma)])
        a_b = C_nb.T.dot(a_n)
        return a_b
    
    if vt_fps < 250:
        x0 = [0.9, 0, 0, 0, np.deg2rad(90), 
                            np.deg2rad(90),
                            np.deg2rad(90),
                            np.deg2rad(90),
              np.deg2rad(0), 0],
    elif vt_fps < 260:
        x0 = [0.9, 0, 0, 0, np.deg2rad(90), 
                            np.deg2rad(80),
                            np.deg2rad(90),
                            np.deg2rad(80),
              np.deg2rad(0), 0],
    elif vt_fps < 270:
        x0 = [0.9, 0, 0, 0, np.deg2rad(80), 
                            np.deg2rad(60),
                            np.deg2rad(80),
                            np.deg2rad(60),
              np.deg2rad(0), 0],
    elif vt_fps < 280:
        x0 = [0.9, 0, 0, 0, np.deg2rad(60), 
                            np.deg2rad(45),
                            np.deg2rad(60),
                            np.deg2rad(45),
              np.deg2rad(0), 0],
    elif vt_fps < 285:
        x0 = [0.9, 0, 0, 0, np.deg2rad(40), 
                            np.deg2rad(20),
                            np.deg2rad(40),
                            np.deg2rad(20),
              np.deg2rad(0), 0],
    elif vt_fps < 290:
        x0 = [0.9, 0, 0, 0, np.deg2rad(15), 
                            np.deg2rad(5),
                            np.deg2rad(15),
                            np.deg2rad(5),
              np.deg2rad(0), 0],
    elif vt_fps < 300:
        x0 = [0.9, 0, 0, 0, np.deg2rad(3), 
                            np.deg2rad(3),
                            np.deg2rad(3),
                            np.deg2rad(3),
              np.deg2rad(0), 0],
    else:
        x0 = [0.9, 0, 0, 0, np.deg2rad(0),
                            np.deg2rad(0),
                            np.deg2rad(0),
                            np.deg2rad(0),
              np.deg2rad(0), 0],

    op, props, res = trim(
        aircraft='F-35B-2',
        ic={
            'ic/h-sl-ft': 800,
            'ic/vt-fps': vt_fps,
            'ic/gamma-deg': gamma_deg,
            'ap/gear-enable': 1,
        },
        design_vector=[
            'fcs/throttle-cmd-norm',
            'fcs/elevator-cmd-norm',
            'fcs/rudder-cmd-norm',
            'fcs/aileron-cmd-norm',
            'propulsion/engine/pitch-angle-rad',
            'propulsion/engine[1]/pitch-angle-rad',
            'propulsion/engine[2]/pitch-angle-rad',
            'propulsion/engine[3]/pitch-angle-rad',
            'ic/alpha-rad',
            'ic/beta-rad',
        ],
        x0=x0,
        verbose=False,
        method='SLSQP',
        eq_constraints= [
            lambda fdm: fdm['accelerations/udot-ft_sec2'] - accel_gamma(fdm, accel_g, gamma_deg)[0],
            lambda fdm: fdm['accelerations/vdot-ft_sec2'],
            lambda fdm: fdm['accelerations/wdot-ft_sec2'] - accel_gamma(fdm, accel_g, gamma_deg)[1],
            lambda fdm: fdm['accelerations/pdot-rad_sec2'],
            lambda fdm: fdm['accelerations/qdot-rad_sec2'],
            lambda fdm: fdm['accelerations/rdot-rad_sec2'],
        ],
        cost=lambda fdm: fdm['fcs/throttle-cmd-norm'],
        bounds=[[0, 1], [-1, 1], [-1, 1], [-1, 1],
                [np.deg2rad(0), np.deg2rad(120)],
                [np.deg2rad(0), np.deg2rad(120)],
                [np.deg2rad(0), np.deg2rad(120)],
                [np.deg2rad(0), np.deg2rad(120)],
                [-0.1, 0.1], [-0.1, 0.1]],
        tol=1e-12)
    return op

ops_trim = [trim_transition(vt_fps=vt, gamma_deg=0, accel_g=0)
                  for vt in [10, 50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 790]]
for op in ops_trim:
    print('\nvt fps', op['ic/vt-fps'])
    print('theta deg', op['ic/gamma-deg'] + np.rad2deg(op['ic/alpha-rad']))
    print('elevator', op['fcs/elevator-cmd-norm'])
    print('aileron', op['fcs/aileron-cmd-norm'])
    print('rudder', op['fcs/rudder-cmd-norm'])
    print('throttle', op['fcs/throttle-cmd-norm'])
```


```python
ops_transition = [trim_transition(vt_fps=vt, gamma_deg=10, accel_g=0.1)
                  for vt in [10, 50, 100, 150, 200, 250, 300, 400, 500, 600, 650]]

ops_transition_auto = []
for op in ops_transition:
    op = dict(op)
    print('\nvt fps', op['ic/vt-fps'])
    print('theta deg', op['ic/gamma-deg'] + np.rad2deg(op['ic/alpha-rad']))
    print('elevator', op['fcs/elevator-cmd-norm'])
    print('aileron', op['fcs/aileron-cmd-norm'])
    print('rudder', op['fcs/rudder-cmd-norm'])
    print('throttle', op['fcs/throttle-cmd-norm'])
    op['ap/roll-enable'] = 1
    op['ap/pitch-enable'] = 1
    op['ap/yaw-enable'] = 1
    op['ap/h-enable'] = 
    op['ap/h-sl-cmd-ft'] = 1000
    ops_transition_auto.append(op)
```


```python
log_transition_auto = []
for op in ops_transition_auto:
    log = simulate(
        aircraft='F-35B-2',
        op_0=op,
        tf=10,
        realtime=False)
    log_transition_auto.append(log)
```

## Auto takeoff 


```python
log_takeoff_auto = simulate(
    aircraft='F-35B-2',
    op_0=op_ground,
    op_list=[('hover', op_hover_auto, lambda fdm: fdm.get_sim_time() > 1),
             ('10 fps', ops_transition_auto[0], lambda fdm: fdm.get_sim_time() > 10),
             ('50 fps', ops_transition_auto[1], lambda fdm: fdm['velocities/vt-fps'] > 50),
             ('100 fps', ops_transition_auto[2], lambda fdm: fdm['velocities/vt-fps'] > 100),
             ('150 fps', ops_transition_auto[3], lambda fdm: fdm['velocities/vt-fps'] > 150),
             ('200 fps', ops_transition_auto[4], lambda fdm: fdm['velocities/vt-fps'] > 200),
             ('250 fps', ops_transition_auto[5], lambda fdm: fdm['velocities/vt-fps'] > 250),
             ('300 fps', ops_transition_auto[6], lambda fdm: fdm['velocities/vt-fps'] > 300),
             ('400 fps', ops_transition_auto[7], lambda fdm: fdm['velocities/vt-fps'] > 400),
             ('500 fps', ops_transition_auto[7], lambda fdm: fdm['velocities/vt-fps'] > 500),
             ('600 fps', ops_transition_auto[7], lambda fdm: fdm['velocities/vt-fps'] > 600),
    ],
    tf=120,
    realtime=True, verbose=True)
```

# Hover Controller Design

## Pitch


```python
sys = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/q-rad_sec'],
    states_deriv = ['accelerations/qdot-rad_sec2'],
    inputs=['fcs/elevator-cmd-norm'],
    outputs=['ic/q-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=10
))
s = control.tf([1, 0], [1])
rad2deg = 180/np.pi
G_elev_to_pitch = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-10))/s  # in degrees
G_elev_to_pitch
```


```python
# # Add the actuator 
# tau = 0.4
# G_act = tau / (s + tau)
# G_elev_to_pitch *= G_act
# G_elev_to_pitch
```


```python
# PD gains to lead-lag gains 
Kp = 0.005
Kd = 0.3
N = 15

c1 = N*Kd + Kp
c2 = N*Kp
c3 = 1
c4 = N


print('c1: ', c1)
print('c2: ', c2)
print('c3: ', c3)
print('c4: ', c4)


H_elev_to_pitch = (c1 * s + c2) / (c3 * s + c4)
# H_elev_to_pitch = 1 + 0.01*s

plt.figure()
rootlocus(G_elev_to_pitch*H_elev_to_pitch)
plt.plot([0, -10], [0, 10], '--')

plt.figure()
rootlocus(G_elev_to_pitch*H_elev_to_pitch)
plt.xlim([-5, 0])
plt.plot([0, -10], [0, 10], '--')

Gc_elev_to_pitch = G_elev_to_pitch*H_elev_to_pitch/(1 + G_elev_to_pitch*H_elev_to_pitch)

plt.figure()
step_size = 10
t, y = control.step_response(step_size*Gc_elev_to_pitch, T=np.linspace(0, 30, 1000))
plt.plot(t, y)
plt.ylabel('pitch, deg')
plt.xlabel('t, sec')
plt.title('output')

plt.figure()
# actual error was computed in radians, so, converting back here
e = np.deg2rad(step_size-y)
t, u, _= control.forced_response(H_elev_to_pitch, T=t, U=e)
plt.plot(t, u)
plt.hlines([-1, 1], t[0], t[-1], linestyles='dashed')
plt.title('input')
plt.ylabel('elevator, norm')
plt.xlabel('t, sec')

plt.figure(figsize=(15, 7))
control.gangof4(G_elev_to_pitch, H_elev_to_pitch, Hz=False, dB=True)

plt.figure()
control.nyquist(Gc_elev_to_pitch, omega=np.logspace(-3, 3, 1000))
plt.plot(np.cos(np.linspace(0,2*np.pi,100)), np.sin(np.linspace(0,2*np.pi,100)), '--')
plt.axis('equal')

gm, pm, wg, wp = control.margin(Gc_elev_to_pitch)
print('gain margin: ', gm)
print('phase margin (in degrees): ', pm)
print('Frequency for gain margin (at phase crossover, phase = -180 degrees): ', wg)
print('Frequency for phase margin (at gain crossover, gain = 1): ', wp)
```

## Roll


```python
sys = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/p-rad_sec'],
    states_deriv = ['accelerations/pdot-rad_sec2'],
    inputs=['fcs/aileron-cmd-norm'],
    outputs=['ic/p-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
rad2deg = 180/np.pi
s = control.tf([1, 0], [1])
G_aileron_to_roll = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s
G_aileron_to_roll
```


```python
# # Add the actuator 
# tau = 0.4
# G_act = tau / (s + tau)
# G_aileron_to_roll *= G_act
# G_aileron_to_roll
```


```python
# PD gains to lead-lag gains 
Kp = 0.0205
Kd = 0.0215
N = 10

c1 = N*Kd + Kp
c2 = N*Kp
c3 = 1
c4 = N

print('c1: ', c1)
print('c2: ', c2)
print('c3: ', c3)
print('c4: ', c4)


H_aileron_to_roll = (c1 * s + c2) / (c3 * s + c4)

plt.figure()
rootlocus(G_aileron_to_roll*H_aileron_to_roll)
plt.plot([0, -1], [0, 1], '--')

plt.figure()
rootlocus(G_aileron_to_roll*H_aileron_to_roll)
plt.xlim([-1.5, 0])
plt.plot([0, -1], [0, 1], '--')

Gc_aileron_to_roll  = G_aileron_to_roll*H_aileron_to_roll/(1 + G_aileron_to_roll*H_aileron_to_roll)


plt.figure()
step_size = 10
t, y = control.step_response(step_size*Gc_aileron_to_roll, T=np.linspace(0, 30, 1000))
plt.plot(t, y)
plt.xlabel('t, sec')
plt.ylabel('roll, deg')
plt.title('output')

plt.figure()
# actual error was computed in radians, so, converting back here
e = np.deg2rad(step_size-y)
t, u, _= control.forced_response(H_aileron_to_roll, T=t, U=e)
plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
plt.plot(t, u)
plt.xlabel('t, sec')
plt.ylabel('aileron %')
plt.title('input')

plt.figure()
control.nyquist(Gc_aileron_to_roll, omega=np.logspace(-3, 3, 1000))
plt.plot(np.cos(np.linspace(0,2*np.pi,100)), np.sin(np.linspace(0,2*np.pi,100)), '--')
plt.axis('equal')

plt.figure(figsize=(15, 7))
control.gangof4(G_aileron_to_roll, H_aileron_to_roll, Hz=False, dB=True)

gm, pm, wg, wp = control.margin(Gc_aileron_to_roll)
print('gain margin: ', gm)
print('phase margin (in degrees): ', pm)
print('Frequency for gain margin (at phase crossover, phase = -180 degrees): ', wg)
print('Frequency for phase margin (at gain crossover, gain = 1): ', wp)
```


```python
# Open loop 
G_aileron_to_roll*H_aileron_to_roll
```


```python
# Closed loop 
Gc_aileron_to_roll
```

## Yaw

## Engine0 (right cold post) 


```python
# import scipy.signal as scysig

sys1 = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/r-rad_sec'],
    states_deriv = ['accelerations/rdot-rad_sec2'],
    inputs=['propulsion/engine/pitch-angle-rad'],
    outputs=['ic/r-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])

# # G_rudder_to_yaw = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s
# print(sys)
# temp = scysig.ss2tf(sys.A, sys.B, sys.C, sys.D)
# print(temp)

G_rudder_to_yaw1 = -clean_tf(control.minreal(control.ss2tf(sys1), 1e-3))/s
G_rudder_to_yaw1
```

## Engine1 (right hot post)


```python
sys2 = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/r-rad_sec'],
    states_deriv = ['accelerations/rdot-rad_sec2'],
    inputs=['propulsion/engine[1]/pitch-angle-rad'],
    outputs=['ic/r-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])
G_rudder_to_yaw2 = -clean_tf(control.minreal(control.ss2tf(sys2), 1e-3))/s
G_rudder_to_yaw2
```

## Engine2 (left cold post)


```python
sys3 = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/r-rad_sec'],
    states_deriv = ['accelerations/rdot-rad_sec2'],
    inputs=['propulsion/engine[2]/pitch-angle-rad'],
    outputs=['ic/r-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])
G_rudder_to_yaw3 = -clean_tf(control.minreal(control.ss2tf(sys3), 1e-3))/s
G_rudder_to_yaw3
```

## Engine3 (left hot post)


```python
sys4 = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/r-rad_sec'],
    states_deriv = ['accelerations/rdot-rad_sec2'],
    inputs=['propulsion/engine[3]/pitch-angle-rad'],
    outputs=['ic/r-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])
G_rudder_to_yaw4 = -clean_tf(control.minreal(control.ss2tf(sys4), 1e-3))/s
G_rudder_to_yaw4
```


```python
sys1 = control.series(sys1, -1/s)
sys1 = control.tf2ss(sys1)
sys2 = control.series(sys2, -1/s)
sys2 = control.tf2ss(sys2)
sys3 = control.series(sys3, -1/s)
sys3 = control.tf2ss(sys3)
sys4 = control.series(sys4, -1/s)
sys4 = control.tf2ss(sys4)

sys_yaw = control.append(sys1, sys2, sys3, sys4)
sys_yaw = control.ss2tf(sys_yaw)
sys_yaw
```


```python
# Engine nozzle pitch actuator 
tau_enp = 2
G_enp_act = tau_enp / (s + tau_enp)
G_aileron_to_roll *= G_enp_act
G_aileron_to_roll
```

## Altitude 


```python
sys = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/w-fps'],
    states_deriv = ['accelerations/wdot-ft_sec2'],
    inputs=['fcs/throttle-cmd-norm'],
    outputs=['ic/w-fps'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
G_throttle_to_alt = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s
G_throttle_to_alt
```


```python
# # Add the actuator 
# tau = 0.4
# G_act = tau / (s + tau)
# G_throttle_to_alt *= G_act
# G_throttle_to_alt
```


```python
# PD gains to lead-lag gains 
Kp = 0.0004
Kd = 0.0045
N = 0.6

c1 = N*Kd + Kp
c2 = N*Kp
c3 = 1
c4 = N


print('c1: ', c1)
print('c2: ', c2)
print('c3: ', c3)
print('c4: ', c4)


H_throttle_to_alt = (c1 * s + c2) / (c3 * s + c4)


plt.figure()
rootlocus(G_throttle_to_alt*H_throttle_to_alt)
plt.plot([0, -1], [0, 1], '--')


Gc_throttle_to_alt = G_throttle_to_alt*H_throttle_to_alt/(1 + G_throttle_to_alt*H_throttle_to_alt)

plt.figure()
step_size = 10
t, y = control.step_response(step_size*Gc_throttle_to_alt, T=np.linspace(0, 40, 1000))
plt.plot(t, y)
plt.xlabel('t, sec')
plt.ylabel('altitude, ft')
plt.title('output')

plt.figure()
# error computed in ft
e = step_size-y
t, u, _= control.forced_response(H_throttle_to_alt, T=t, U=e)
plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
plt.plot(t, u)
plt.xlabel('t, sec')
plt.ylabel('throtle %')
plt.title('input')

plt.figure()
control.nyquist(Gc_throttle_to_alt, omega=np.logspace(-3, 3, 1000))
plt.plot(np.cos(np.linspace(0,2*np.pi,100)), np.sin(np.linspace(0,2*np.pi,100)), '--')
plt.axis('equal')

plt.figure(figsize=(15, 7))
control.gangof4(G_throttle_to_alt, H_throttle_to_alt, Hz=False, dB=True)


gm, pm, wg, wp = control.margin(Gc_throttle_to_alt)
print('gain margin: ', gm)
print('phase margin (in degrees): ', pm)
print('Frequency for gain margin (at phase crossover, phase = -180 degrees): ', wg)
print('Frequency for phase margin (at gain crossover, gain = 1): ', wp)
```

# Cruise Controller Design

## Pitch


```python
sys = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/q-rad_sec'],
    states_deriv = ['accelerations/qdot-rad_sec2'],
    inputs=['fcs/elevator-cmd-norm'],
    outputs=['ic/q-rad_sec'],
    ic=op_cruise,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])
rad2deg = 180/np.pi
G_elev_to_pitch = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s  # in degrees
G_elev_to_pitch
```


```python
# # Add the actuator 
# tau = 0.4
# G_act = tau / (s + tau)
# G_elev_to_pitch *= G_act
# G_elev_to_pitch
```


```python
# !! <<<< THESE ARE FOR HOVER 
# PD gains to lead-lag gains 
Kp = 1.27231590533845e-09
Kd = 0.0127229959138648
N = 148.271702233295

c1 = N*Kd + Kp
c2 = N*Kp
c3 = 1
c4 = N

# >>>> SUBJECT TO CHANGE !!




H_elev_to_pitch = (c1 * s + c2) / (c3 * s + c4)
# H_elev_to_pitch = 1 + 0.01*s

plt.figure()
rootlocus(G_elev_to_pitch*H_elev_to_pitch)
plt.plot([0, -1], [0, 1], '--')

plt.figure()
rootlocus(G_elev_to_pitch*H_elev_to_pitch)
plt.xlim([-1.5, 0])
plt.plot([0, -1], [0, 1], '--')

Gc_elev_to_pitch = G_elev_to_pitch*H_elev_to_pitch/(1 + G_elev_to_pitch*H_elev_to_pitch)

plt.figure()
step_size = 10
t, y = control.step_response(step_size*Gc_elev_to_pitch, T=np.linspace(0, 30, 1000))
plt.plot(t, y)
plt.ylabel('pitch, deg')
plt.xlabel('t, sec')
plt.title('output')

plt.figure()
# actual error was computed in radians, so, converting back here
e = np.deg2rad(step_size-y)
t, u, _= control.forced_response(H_elev_to_pitch, T=t, U=e)
plt.plot(t, u)
plt.hlines([-1, 1], t[0], t[-1], linestyles='dashed')
plt.title('input')
plt.ylabel('elevator, norm')
plt.xlabel('t, sec')

plt.figure(figsize=(15, 7))
control.gangof4(G_elev_to_pitch, H_elev_to_pitch, Hz=True, dB=True)

plt.figure()
control.nyquist(Gc_elev_to_pitch, omega=np.logspace(-3, 3, 1000))

gm, pm, wg, wp = control.margin(Gc_elev_to_pitch)
print('gain margin: ', gm)
print('phase margin (in degrees): ', pm)
print('Frequency for gain margin (at phase crossover, phase = -180 degrees): ', wg)
print('Frequency for phase margin (at gain crossover, gain = 1): ', wp)
```

## Roll


```python
sys = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/p-rad_sec'],
    states_deriv = ['accelerations/pdot-rad_sec2'],
    inputs=['fcs/aileron-cmd-norm'],
    outputs=['ic/p-rad_sec'],
    ic=op_cruise,
    dx=1e-3,
    n_round=3
))
rad2deg = 180/np.pi
s = control.tf([1, 0], [1])
G_aileron_to_roll = rad2deg*clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s
G_aileron_to_roll
```


```python
# Add the actuator 
tau = 0.4
G_act = tau / (s + tau)
G_aileron_to_roll *= G_act
G_aileron_to_roll
```


```python
# !! <<<< THESE ARE FOR HOVER 
# PD gains to lead-lag gains 
Kp = 7.66302615547072e-10
Kd = 0.0076620813293718
N = 505.040986830046

c1 = N*Kd + Kp
c2 = N*Kp
c3 = 1
c4 = N

# >>>> SUBJECT TO CHANGE !!



H_aileron_to_roll = (c1 * s + c2) / (c3 * s + c4)

plt.figure()
rootlocus(G_aileron_to_roll*H_aileron_to_roll)
plt.plot([0, -1], [0, 1], '--')

plt.figure()
rootlocus(G_aileron_to_roll*H_aileron_to_roll)
plt.xlim([-1.5, 0])
plt.plot([0, -1], [0, 1], '--')

Gc_aileron_to_roll  = G_aileron_to_roll*H_aileron_to_roll/(1 + G_aileron_to_roll*H_aileron_to_roll)


plt.figure()
step_size = 10
t, y = control.step_response(step_size*Gc_aileron_to_roll, T=np.linspace(0, 30, 1000))
plt.plot(t, y)
plt.xlabel('t, sec')
plt.ylabel('roll, deg')
plt.title('output')

plt.figure()
# actual error was computed in radians, so, converting back here
e = np.deg2rad(step_size-y)
t, u, _= control.forced_response(H_aileron_to_roll, T=t, U=e)
plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
plt.plot(t, u)
plt.xlabel('t, sec')
plt.ylabel('aileron %')
plt.title('input')

plt.figure()
control.nyquist(Gc_aileron_to_roll, omega=np.logspace(-3, 3, 1000))

plt.figure(figsize=(15, 7))
control.gangof4(G_aileron_to_roll, H_aileron_to_roll, Hz=True, dB=True)

gm, pm, wg, wp = control.margin(Gc_aileron_to_roll)
print('gain margin: ', gm)
print('phase margin (in degrees): ', pm)
print('Frequency for gain margin (at phase crossover, phase = -180 degrees): ', wg)
print('Frequency for phase margin (at gain crossover, gain = 1): ', wp)
```

## Yaw


```python
sys1 = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/r-rad_sec'],
    states_deriv = ['accelerations/rdot-rad_sec2'],
    inputs=['propulsion/engine/pitch-angle-rad'],
    outputs=['ic/r-rad_sec'],
    ic=op_hover,
    dx=1e-3,
    n_round=3
))
s = control.tf([1, 0], [1])

# # G_rudder_to_yaw = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s
# print(sys)
# temp = scysig.ss2tf(sys.A, sys.B, sys.C, sys.D)
# print(temp)

G_rudder_to_yaw1 = -clean_tf(control.minreal(control.ss2tf(sys1), 1e-3))/s
G_rudder_to_yaw1
```

## Altitude 


```python
sys = control.ss(*linearize(
    aircraft='F-35B-2',
    states=['ic/w-fps'],
    states_deriv = ['accelerations/wdot-ft_sec2'],
    inputs=['fcs/throttle-cmd-norm'],
    outputs=['ic/w-fps'],
    ic=op_cruise,
    dx=1e-3,
    n_round=3
))
G_throttle_to_alt = -clean_tf(control.minreal(control.ss2tf(sys), 1e-3))/s
G_throttle_to_alt
```


```python
# Add the actuator 
tau = 0.4
G_act = tau / (s + tau)
G_throttle_to_alt *= G_act
G_throttle_to_alt
```


```python
# !! <<<< THESE ARE FOR HOVER 
# PD gains to lead-lag gains 
Kp = 9.11459759968896e-10
Kd = 0.00911454537591535
N = 27.8343628363698

c1 = N*Kd + Kp
c2 = N*Kp
c3 = 1
c4 = N

# >>>> SUBJECT TO CHANGE !!






H_throttle_to_alt = (c1 * s + c2) / (c3 * s + c4)


plt.figure()
rootlocus(G_throttle_to_alt*H_throttle_to_alt)
plt.plot([0, -1], [0, 1], '--')

plt.figure()
rootlocus(G_throttle_to_alt*H_throttle_to_alt)
plt.xlim([-1.2, 0])
plt.plot([0, -1], [0, 1], '--')

Gc_throttle_to_alt = G_throttle_to_alt*H_throttle_to_alt/(1 + G_throttle_to_alt*H_throttle_to_alt)

plt.figure()
step_size = 10
t, y = control.step_response(step_size*Gc_throttle_to_alt, T=np.linspace(0, 40, 1000))
plt.plot(t, y)
plt.xlabel('t, sec')
plt.ylabel('altitude, ft')
plt.title('output')

plt.figure()
# error computed in ft
e = step_size-y
t, u, _= control.forced_response(H_throttle_to_alt, T=t, U=e)
plt.hlines([-0.1, 0.1], t[0], t[-1], linestyles='dashed')
plt.plot(t, u)
plt.xlabel('t, sec')
plt.ylabel('throtle %')
plt.title('input')

plt.figure()
control.nyquist(Gc_throttle_to_alt, omega=np.logspace(-3, 3, 1000))

plt.figure(figsize=(15, 7))
control.gangof4(G_throttle_to_alt, H_throttle_to_alt, Hz=True, dB=True)


gm, pm, wg, wp = control.margin(Gc_throttle_to_alt)
print('gain margin: ', gm)
print('phase margin (in degrees): ', pm)
print('Frequency for gain margin (at phase crossover, phase = -180 degrees): ', wg)
print('Frequency for phase margin (at gain crossover, gain = 1): ', wp)
```
