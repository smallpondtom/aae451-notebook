```python
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import json 
from jsbsim_utils import Logger, trim, simulate
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
        
op_ground, fdm, res = trim(
    aircraft='F-35B-2',
    ic={
        'ic/vt-fps': 0,
        'gear/gear-cmd-norm': 1,
        'propulsion/engine/pitch-angle-rad': np.deg2rad(0),
        'fcs/throttle-cmd-norm': 0,
        'fcs/aileron-cmd-norm': 0,
        'fcs/elevator-cmd-norm': 0,
        'fcs/rudder-cmd-norm': 0,
        'fcs/left-brake-cmd-norm': 1,
        'fcs/right-brake-cmd-norm': 1,
        'fcs/center-brake-cmd-norm': 1,
    },
    design_vector=['ic/theta-rad', 'ic/h-agl-ft'],
    x0=[0, 3.5],
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


```python
fdm.resume_integration()
fdm.get_propulsion().get_steady_state()
```


```python
op_hover, fdm, res = trim(
    aircraft='F-35B-2',
    ic={
        'ic/h-agl-ft': 10,
        'ic/vd-fps': 0,
        'ic/vn-fps': 0*np.cos(np.deg2rad(280)),
        'ic/ve-fps': 0*np.sin(np.deg2rad(280)),
        'ic/theta-rad': 0,
        'gear/gear-cmd-norm': 1,
        'fcs/left-brake-cmd-norm': 0,
        'fcs/right-brake-cmd-norm': 0,
        'fcs/center-brake-cmd-norm': 0,
    },
    design_vector=[
        'fcs/throttle-cmd-norm',
        'fcs/elevator-cmd-norm',
        'propulsion/engine/pitch-angle-rad',
        'propulsion/engine[1]/pitch-angle-rad',
        'propulsion/engine[2]/pitch-angle-rad',
        'propulsion/engine[3]/pitch-angle-rad',
    ],
    x0=[0.5, 0, np.deg2rad(90), np.deg2rad(90), np.deg2rad(90), np.deg2rad(90)],
    verbose=True,
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
print(fdm.get_property_catalog('dot'))
data1['dot_values'] = fdm.get_property_catalog('dot')

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
        
op_cruise, fdm, res = trim(
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
    },
    design_vector=[
        'fcs/throttle-cmd-norm',
        'fcs/elevator-cmd-norm',
        'fcs/rudder-cmd-norm',
        'fcs/aileron-cmd-norm',
        'ic/alpha-rad',
        'ic/beta-rad',
        'propulsion/engine/pitch-angle-rad',
    ],
    cost=cost_cruise,
    x0=[0.4, 0, 0, 0, 0, 0, 0],
    verbose=True,
    bounds=[[0, 1], [-1, 1], [-1, 1], [-1, 1], [-1, 1], [-1, 1], [np.deg2rad(0), np.deg2rad(120)]],
    tol=1e-12,
)
op_cruise

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
