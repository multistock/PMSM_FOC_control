clear
close all
clc

%% PMSM Parameters
% Electrical parameters
pmsm.Power = 1000;                  %rated power [W]
pmsm.Vdc = 48;                      %rated DC link voltage [Vdc]
pmsm.pp = 4;                        %pole pairs [-]
pmsm.Rs = 500e-3;                   %stator phase resistance [Ohm]
pmsm.Ld = 1.5e-3;                   %stator d-axis inductance [H]
pmsm.Lq = 1.5e-3;                   %stator q-axis inductance [H]
pmsm.L0 = 0;                        %stator zero-sequence inductance [H]
pmsm.lambda_pm = 150e-03;           %permanent magnet flux linkage [Wb]
pmsm.n_rat = 3000;                  %maximum mechanical speed [rpm] 
pmsm.trq_rat = 3.2;                 %rated torque [Nm]
pmsm.Is_max = 25;                   %maximum current limit [Apeak]

% mechanical parameters    
pmsm.b =  0.002;                    %viscous friction of motor [Nm/(rad/s)]
pmsm.J = 0.01;                      %Inertia of motor [kg*m^2]

% load parameters
load.m = 22;                                %mass of load - bicycle [kg]
load.m_wheel = 1;                           %mass of wheel - bicycle [kg]
load.r = 0.7/2;                             %radius wheel [m]
load.J = load.m_wheel * load.r^2;           %load inertia [kg*m^2]
load.cf = 0.004;                            %coefficient rolling friction - bicycle tire on asphalt road [-]
load.trq_f = load.cf*load.m*9.81*load.r;    %friction torque. F*r [Nm]

% field weakening parameters
fw.mod = 1;                                             %relative modulation index, max = 1 (absolute max modulation index = 1.15)
fw.vs_max = fw.mod*pmsm.Vdc/sqrt(3);                    %maximum phase voltage for SVPWM without considering overmodulation [Vpeak]
fw.vs_max_doublecheck=1.15*pmsm.Vdc/2;                  %double check maximum phase voltage for SVPWM with modulation index of 1.15 [Vpeak]
fw.trq_max = 1.5*pmsm.pp*pmsm.lambda_pm*pmsm.Is_max;    %maximum electromagnetic torque [Nm]
fw.w_base = fw.vs_max / (sqrt((pmsm.Lq*pmsm.Is_max)^2+(pmsm.lambda_pm)^2)); %base speed [rad/s]
fw.n_base = fw.w_base * 60/2/pi/pmsm.pp;                                    %base speed [rpm]

%% Controller parameters
% Original PI gains given in the task
org.Kpd=0.1;
org.Kpq=0.1;
org.Kid=10;
org.Kiq=10;
org.Kpw=0.05;
org.Kiw=0.5;

% sampling time steps
foc.fsim = 10e3;                    %simulation sample frequency [Hz]
foc.Ts = 1/foc.fsim;                %simulation sample time step [s]

foc.fc = foc.fsim;                  %current control loop frequency [Hz]
foc.Ts_c = 1/foc.fc;                %current control sample time step [s]

foc.fs = foc.fc/10;                 %speed control loop frequency [Hs]
foc.Ts_s = 10*foc.Ts_c;             %speed control simulation time step [s]

% Current control loop
foc.BW_CL_i = 100;                      %current control loop - closed loop bandwidth request [Hz]
foc.BW_CL_i_radps = foc.BW_CL_i*2*pi;   %current control loop - closed loop bandwidth request [rad/s]
foc.tuner_c = 1;                        %init tuning parameter for current loop [-]
tmp.Tau_id = pmsm.Ld/(pmsm.Rs);         %plant time constant [s]
tmp.Tau_iq = pmsm.Lq/(pmsm.Rs);         %plant time constant [s]

foc.Kpd = pmsm.Ld*foc.BW_CL_i_radps;              %[Ohm] 
foc.Kpq = pmsm.Lq*foc.BW_CL_i_radps;              %[Ohm] 
foc.Kid = pmsm.Rs*foc.BW_CL_i_radps*foc.tuner_c;  %[Ohm*s^-1]
foc.Kiq = pmsm.Rs*foc.BW_CL_i_radps*foc.tuner_c;  %[Ohm*s^-1]

% Speed control loop
foc.BW_CL_w = foc.BW_CL_i/15;                   %speed control loop - closed loop bandwidth request [Hz]
foc.BW_CL_w_radps = foc.BW_CL_w*2*pi;           %speed control loop - closed loop bandwidth request [rad/s]
foc.tuner_w = 1;                                %init tuning parameter for speed loop [-]
tmp.Tau_speed = (pmsm.J/pmsm.b);                %motor time constant [s]

foc.Kpw = foc.BW_CL_w_radps*pmsm.J;             %[Nm/radps]
foc.Kiw = foc.BW_CL_w_radps*pmsm.b;             %[s^-1]  

% Dynamic gain scheduling
dyn.maxBW_i = 250;                                  %max bandwidth current loop [Hz]
dyn.maxBW_w = 10;                                   %max bandwidth speed loop [Hz]
dyn.speedVec_rpm = linspace(0,pmsm.n_rat,10);       %speed vector [rpm]
dyn.bwVec_i_Hz = linspace(10,dyn.maxBW_i,10);       %bandwidth vector for current loop[Hz]
dyn.bwVec_i_radps = dyn.bwVec_i_Hz .*2*pi;          %bandwidth vector for current loop [rad/s]
dyn.bwVec_w_Hz = linspace(2,dyn.maxBW_w,10);        %bandwidth vector for speed loop [Hz]
dyn.bwVec_w_radps = dyn.bwVec_w_Hz .*2*pi;          %bandwidth vector for speed loop[rad/s]
dyn.tuner = 1;                                      %tuning parameter [-]

%slew rate limiters
foc.slew_speed = 6000;                              %slew rate limiter for speed command [rpm/s]
foc.slew_torque = 25;                               %slew rate limiter for torque command [Nm/s]

%% Plot torque-speed envelope 
tmp.Id_axis = linspace(-pmsm.Is_max,0,50);
tmp.Iq_axis = linspace(0,pmsm.Is_max,50);
tmp.n_axis = linspace(0,pmsm.n_rat,50);   
tmp.Trq_vec = (3/2)*pmsm.pp*(pmsm.lambda_pm.*tmp.Iq_axis);
tmp.Trq_max = max(tmp.Trq_vec);
tmp.w_base = fw.vs_max / (sqrt((pmsm.Lq*pmsm.Is_max)^2+(pmsm.lambda_pm)^2));
tmp.n_base = tmp.w_base*60/2/pi/pmsm.pp;
tmp.power = tmp.Trq_max*tmp.w_base/pmsm.pp;
tmp.Trq_envelope = zeros(1,length(tmp.n_axis));
tmp.Pwr_envelope = zeros(1,length(tmp.n_axis));

for i = 1:length(tmp.n_axis)
    if tmp.n_axis(i) <= tmp.n_base
        tmp.Trq_envelope(i) = tmp.Trq_max;
    else 
        tmp.Trq_envelope(i) = tmp.power/(tmp.n_axis(i)*2*pi/60);
    end
end

for j = 1:length(tmp.n_axis)
    if tmp.n_axis(j) <= tmp.n_base
        tmp.Pwr_envelope(j) = tmp.Trq_max*tmp.n_axis(j)*2*pi/60;
    else 
        tmp.Pwr_envelope(j) = tmp.power;
    end
end

figure
grid on
yyaxis left
plot(tmp.n_axis,tmp.Trq_envelope)
ylabel("torque [Nm]") 
hold on
yyaxis right
plot(tmp.n_axis,tmp.Pwr_envelope)
ylabel("power [W]")
title('Torque/Power Speed envelope')
xlabel("speed [rpm]")

%% LUT current reference generation
pmsm.w_rat = 2*pi.*pmsm.n_rat.*pmsm.pp./60;

lut.LUTstep = 99;                                       %define step granularity of LUT
lut.trqLUT = linspace(0,fw.trq_max,lut.LUTstep+1);
lut.speedLUT = linspace(0,pmsm.n_rat,lut.LUTstep+1);
lut.idLUT = zeros(lut.LUTstep+1,lut.LUTstep+1);
lut.iqLUT = zeros(lut.LUTstep+1,lut.LUTstep+1);

lut.w_step = pmsm.w_rat/lut.LUTstep;
lut.t_step = fw.trq_max/lut.LUTstep;
y=1;
for w = 0:lut.w_step:pmsm.w_rat
    x=1;
    for t = 0:lut.t_step:fw.trq_max
        if w < tmp.w_base
            %MTPA approach
            lut.id_MTPA = 0;
            lut.iq_MTPA = t/(1.5*pmsm.pp*pmsm.lambda_pm);
            %allocate Id,Iq to LUT
            lut.idLUT(x,y)=(lut.id_MTPA);
            lut.iqLUT(x,y)=(lut.iq_MTPA);
            
            x=x+1;
        else
            %CVCP approach
            lut.id_fw = (tmp.w_base - w)*pmsm.lambda_pm/(w*pmsm.Ld);
            lut.id_sat = max(lut.id_fw,-pmsm.Is_max);
            lut.iq_fw = t/(1.5*pmsm.pp*pmsm.lambda_pm);
            lut.iq_lim = sqrt(pmsm.Is_max^2 - lut.id_sat^2);

            if lut.iq_fw < -lut.iq_lim
                lut.iq_sat = -lut.iq_lim;
            elseif lut.iq_fw > lut.iq_lim
                lut.iq_sat = lut.iq_lim;
            else 
                lut.iq_sat = lut.iq_fw;
            %allocate Id,Iq to LUT
            lut.idLUT(x,y)=(lut.id_sat);
            lut.iqLUT(x,y)=(lut.iq_sat);       
                
            x=x+1;
            end
        end
    end
    y=y+1;
end

figure
surf(lut.speedLUT,lut.trqLUT,lut.idLUT)
xlabel("rpm")
ylabel("Nm")
title("id lookup table")

figure
surf(lut.speedLUT,lut.trqLUT,lut.iqLUT)
xlabel("rpm")
ylabel("Nm")
title("iq lookup table")

open PMSM_simulink.slx

clear i;
clear j;
clear x;
clear y;
clear t;
clear w;