Model Predictive Control of a Single Battery Energy Storage
Consider the situation shown in Figure 1. The objective is to regulate the amount of
energy stored in the battery, E, by manipulating the total energy provided by the grid and
the solar system, Es, to the battery. The energy discharged from the battery to the load, El,
is considered as a disturbance and is not measured. The discrete-time dynamic model is
provided by:
𝐸(𝑘 + 1) = 𝐸(𝑘) + 𝐸s(𝑘) − 𝐸l(𝑘)
The battery has a 7kWh capacity and imposed capacity limits are Emin = 1kWh and Emax =
6kWh. The battery set point is Eset = 3kWh.

4 – Design an optimal controller to minimize the battery capacity variability assuming the
charging variation is minimal.
Use a LQR with inifinite horizon to control the states xk by setting Q to be large(100 let's say)
and the R to be small (1 let's say) because we don't need to care about the control effort as much.
Infinite-horizon Linear quadratic(LQ) optimal control for LTI system
make changes to the disturbance, 
10% and 90% 
before the setpoint it could go above 100% or 0% (transient state)
For the Value of Q and R we can experiment with more values later.
5 – Design a PI controller to control the battery capacity.
same pole  placement location?
6 – Design a MPC.
Use a LQR but with finite horizon of time step 1 or other fixed value(can experiment later).

observer with LQT(tracking)
final r for disturbance is assumed known but it's in fact unknown

note for self
okay so the observer estimate the load energy which is random gaussian distribution
with an offset. 
LQT take the known distrubance dk (assumes to be a constant), therefore in this case
 the LQT can't deal with the disturbance(load energy)
 if we try to use observer to estimate the load energy and feed it to LQT's control
 action, would it help?
 

