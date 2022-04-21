# Nonlinear Optimization for Off-road Self Driving
## CMU 16-745 Optimal Control &amp; Reinforcement Learning Final Project with Gerald D'Ascoli, Jonathan Lord-Fonda, Jason Xiang

### Double Track Model
(Source: ["Yaw Stability Control System Development and Implementation for a Fully Electric Vehicle"](https://arxiv.org/ftp/arxiv/papers/2012/2012.04719.pdf#:~:text=The%20double%20track%20model%20is,model%20with%20all%20four%20wheels.))


![alt text](https://github.com/themorethegerryer/Nonlinear-Optimization-for-Off-road-Self-Driving/blob/main/double_track_model_graphic.PNG?raw=true)

| Symbol | Definition | Unit |
|--------|------------|------|
| *a<sub>x</sub>* | Vehicle longitudinal acceleration | m/s<sup>2</sup> |
| *a<sub>y</sub>* | Vehicle lateral acceleration | m/s<sup>2</sup> |
| *r* | Yaw rate | rad/s |
| *I<sub>w</sub>* | Wheel moment of inertia | kg⋅m<sup>2</sup> |
| *w<sub>i</sub>* | Wheel rotational velocity | rad/s |
| *T<sub>d</sub>* | Drive torque on wheels | N⋅m |
| *T<sub>bi</sub>* | Braking torque | N⋅m |
| *F<sub>xi</sub>* | Tire longitudinal velocity | N |
| *l<sub>w1</sub>* | Front track width | m |
| *l<sub>w2</sub>* | Rear track width | m |
| *R<sub>w</sub>* | Effective tire radius | m |
| *F<sub>ri</sub>* | Tire rolling resistance force | N |
| *V<sub>xi</sub>* | Wheel longitudinal velocity | m/s |
| *V<sub>wi</sub>* | Wheel center of gravity velocity | m/s |
| *α<sub>i</sub>* | Wheel side slip angle | rad |
| *V<sub>x</sub>* | Vehicle longitudinal velocity | m/s |
| *V<sub>y</sub>* | Vehicle lateral velocity | m/s |
| *λ<sub>i</sub>* | Wheel slip ratio | - |
