# PMSM_FOC_control

A PMSM motor with a FOC control structure is modelled in Simulink. 

The PI gains were tuned according to pole-zero compensation method and provides good results. A gain scheduling method was implemented to improve robustness, noise rejection and stability of the controller in the full operating range.

The simulation results are used to assess the controller performance with an ideal PMSM. Modelling a PMSM based solely on mathematical equations does not consider a key factor that results in the discrepancy between simulation and real machine performance: the nonlinear variation of machine parameters.

Therefore, the model could be upgraded with an inverter to account for modulated waveforms instead of perfect sine waveforms to inject AC disturbances in the control loops. Additionally, the PMSM could be made more realistic by adding non-linearities such as PM flux harmonics (BEMF harmonics), differential inductances or by using FEA data.
