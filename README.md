# PMSM_FOC_control
# keywords: PMSM, FOC, d-q decoupling, PI gain scheduling, field weakening

A PMSM motor with a FOC control structure is modelled in Simulink. 

The PI gains were tuned according to pole-zero compensation method and provide good results. A gain scheduling method was implemented to improve robustness, noise rejection and stability of the controller in the full operating range.

The simulation results are used to assess the controller performance with an ideal PMSM. Modelling a PMSM based solely on mathematical equations does not consider a key factor that results in the discrepancy between simulation and real machine performance: the nonlinear variation of machine parameters.

Therefore, the model could be upgraded with an inverter to account for modulated waveforms instead of perfect sine waveforms to inject AC disturbances in the control loops. Additionally, the PMSM could be made more realistic by adding non-linearities such as PM flux harmonics (BEMF harmonics), differential inductances or by using FEA data.

-------------------------------------------------------

The PMSM is modelled in the rotating d-q reference frame to simplify our equations to DC variables and to directly control the PMSM by the Vd/Vq output of the FOC controller.
The modelled PMSM is considered as an ‘ideal’ PMSM, meaning it doesn’t account for any non-linearities such as differential inductances or variable permanent magnet flux linkage.
The modelling of the inverter is out of the project scope. If included, this would introduce modulated AC currents and voltages and additional delay in the FOC control loop. The modulated AC currents include harmonic content and would inject AC disturbances in the FOC control loops.
We model the PMSM as a series circuit with a resistor, an inductor and a BEMF voltage source.

The control system of the motor consists of two control loops in a cascaded system, an inner (current) and an outer loop (speed) loop. Unlike the motor, which is running in continuous time, the controller is modelled in discrete time. We have no additional delays in the control loop because we don’t consider an inverter or additional low pass filters for the speed feedback signal for example.
