# Nonlinear Optimization for Off-road Self Driving
## CMU 16-745 Optimal Control &amp; Reinforcement Learning Final Project with Gerald D'Ascoli, Jonathan Lord-Fonda, Jason Xiang

### Double Track Model
(Source: ["Yaw Stability Control System Development and Implementation for a Fully Electric Vehicle"](https://arxiv.org/ftp/arxiv/papers/2012/2012.04719.pdf#:~:text=The%20double%20track%20model%20is,model%20with%20all%20four%20wheels.))


![alt text](https://github.com/themorethegerryer/Nonlinear-Optimization-for-Off-road-Self-Driving/blob/main/media/double_track_model_graphic.PNG?raw=true)

| Symbol | Definition | Unit |
|--------|------------|------|
| *a<sub>x</sub>* | Vehicle longitudinal acceleration | m/s<sup>2</sup> |
| *a<sub>y</sub>* | Vehicle lateral acceleration | m/s<sup>2</sup> |
| *r* | Yaw rate | rad/s |
| *m* | Vehicle Mass | kg |
| *I<sub>w</sub>* | Wheel moment of inertia | kg⋅m<sup>2</sup> |
| *I<sub>z</sub>* | Yaw moment of inertia | kg⋅m<sup>2</sup> |
| *w<sub>i</sub>* | Wheel rotational velocity | rad/s |
| *T<sub>d</sub>* | Drive torque on wheels | N⋅m |
| *T<sub>bi</sub>* | Braking torque | N⋅m |
| *F<sub>xi</sub>* | Tire longitudinal velocity | N |
| *l<sub>w1</sub>* | Front track width | m |
| *l<sub>w2</sub>* | Rear track width | m |
| *l<sub>r</sub>* | Distance from rear axle to CoM | m |
| *l<sub>f</sub>* | Distance from front axle to CoM | m |
| *R<sub>w</sub>* | Effective tire radius | m |
| *F<sub>ri</sub>* | Tire rolling resistance force | N |
| *V<sub>xi</sub>* | Wheel longitudinal velocity | m/s |
| *V<sub>wi</sub>* | Wheel center of gravity velocity | m/s |
| *α<sub>i</sub>* | Wheel side slip angle | rad |
| *β* | Vehicle side slip angle | rad |
| *V<sub>x</sub>* | Vehicle longitudinal velocity | m/s |
| *V<sub>y</sub>* | Vehicle lateral velocity | m/s |
| *λ<sub>i</sub>* | Wheel slip ratio | - |


### State and Control Inputs

State: X
| Description | Variables | Indices |
|-------------|-----------|---------|
| Position of the center of mass in the world frame | *x, y, z* | 0:3 |
| Unit quaternion mapping vectors in the body frame to the world frame | *q<sub>s</sub>, q<sub>x</sub>, q<sub>y</sub>, q<sub>z</sub>* | 3:7 |
| Linear velocity in the world frame | *v<sub>x</sub>, v<sub>y</sub>, v<sub>z</sub>* | 7:10 |
| Angular velocity in the body frame | *ω<sub>x</sub>, ω<sub>y</sub>, ω<sub>z</sub>* | 10:13 |

Control: U
| Description | Variables | Indices |
|-------------|-----------|---------|
| Steering Angle | θ | 0 |
| Wheel Torque (All wheel drive) | 𝜏 | 1 |

### Terrain Creation
# https://www.youtube.com/watch?v=a1kxPd8Mdhw
#
# Install Requirements
# conda create --prefix ./env numpy pyopengl
# conda create --prefix ./env2 pytorch numpy pyopengl anaconda pyqt
#       conda install pyqtgraph
# conda create --prefix ./env3 numpy pytorch pyopengl pyqtgraph anaconda pyqt 
# Or
# conda install python=3.4
# conda install -c anaconda pyopengl
# conda install numpy
# conda install pyqtgraph
# conda install -c anaconda pyqt
# conda install PyQt5
# conda install -c dsdale24 pyqt5
# IT'S ALL A LIE, NONE OF IT WORKS
# conda create --prefix ./env
#       pip install pyqtgraph
#       If you get a numpy error
#           pip uninstall -y numpy
#           pip uninstall -y setuptools
#           pip install setuptools
#           pip install numpy
#       pip install pyopengl
#       pip install pyqtgraph
#       pip install pyqt5

# Plotly install
# conda create --prefix ./envPlotly numpy pytorch plotly scipy -c plotly plotly

# PyOpenGl Tutorial:
# https://www.youtube.com/watch?v=LqPPvPKUfV4&list=PL1P11yPQAo7opIg8r-4BMfh1Z_dCOfI0y&index=1
# pip install glfw      # 1.8.3
# pip install pyopengl  # 3.1.0
# pip install pyrr      # 0.10.3
# pip install pillow    # 6.1.0