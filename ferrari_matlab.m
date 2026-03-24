%% vehicle dinamics parameters of a ferrari sf90 stradale
clc
clear all 
close all

%speed on x axis of the vehicle:
speedkm = 30; % [km/h]
v_long = speedkm/3.6; % [m/s]

%stiffness coefficient peak of a racing car
%c_af < c_ar set in oversteering mode because the sf90 stradale 
%is designed to slightly oversteer
c_af = 65000; % [N/rad]
c_ar = 70000; % [N/rad] 

%mass of the vehicle with weight distribution: 45 front/55 rear 
m = 1570; % [kg] 
wheelBase = 2.65; % [m]
lf = wheelBase*0.55; % [m] b
lr = wheelBase - lf; % [m] a
Iz = m*0.45*lf^2 + m*0.55*lr^2; % [kg*m^2]  [ I = m*r^2 ]

%Road radius approx.
curveRadius = 1000; % [m]

%longitudinal distance of the point ahead of the vehicle
ds = 7; %[m]

%% Error dynamics state-space system
%state-space model:

%state system matrix:
Ae = [0,                  1,            0,                    0;
      0, -((2*c_af+2*c_ar)/(m*v_long)), ((2*c_af+2*c_ar)/(m)), -((2*c_af*lf-2*c_ar*lr)/(m*v_long));
      0,                  0,            0,                    1;
      0, -((2*lf*c_af-2*lr*c_ar)/(Iz*v_long)), ((2*lf*c_af-2*lr*c_ar)/(Iz)), -((2*(lf^2)*c_af+2*(lr^2)*c_ar)/(Iz*v_long))];

%steering angle input matrix:
B1e = [0; 2*(c_af/m); 0; (2*lf*c_af)/Iz];

%desired yaw angle of the vehicle input (derived phi des) matrix: 
B2e = [0; -v_long-((2*lf*c_af-2*lr*c_ar)/(m*v_long)); 0; ((-2*(lf^2)*c_af-2*(lr^2)*c_ar)/(Iz*v_long))];

%output matrix:
C1e = [1 0 0 0]; %e1 the distance of the c.g. of the vehicle from the center line of the lane 
C2e = [0 0 1 0]; %e2 the orientation error of the vehicle with respect to the road

%feedthrough matrix:
D=0;

%% Transfer function SISO 
close all

s = tf ('s');

inverse = inv(s*eye(4)-Ae);

%transfer function between steering angle input 
%and lateral position error output
H1 = C1e*inverse;
W1e = H1*B1e + D;

%transfer function between steering angle input
%and yaw angle error output 
H2 = C2e*inverse;
W2e = H2*B1e + D;

%P(s) is the plant transfer function between the steering 
%angle input for the vehicle
%and the lateral position error measurement output
Pe = W1e + ds*W2e;

%% Stability

%SPECS:
%settling time under 5 seconds
%steady state error below 2% for step references
%reject measure disturbances from a sinusoidal input n(t)=(4*pi/180)*sin(2*pi*3*t)
%overshoot under 5%

close all

%computing the parameters kp and kc in order
%to meet the specification error 
%required for the static controller design:

Kp = dcgain(Pe); % 5.1532*10^16
% q = 1 -> q-1 = 0
% e_true,inf = lim(s,0) [ R/(s^(q-1) + kp*kc) ]
% e_true,inf = lim(s,0) [ R/(s^0 + kp*kc) ] = R/(1 + kp*kc)
% R/(1 + kp*kc) < R * 2/100
% 1/(1 + kp*kc) < 0.02
% kc < 1/(-5.1532*10^16) v kc > 49/(5.1532*10^16)

%static controller 
Cs = 1; %  r(s) = R/s
%dynamic controller
Cd = 1;
%overall controller
C = Cs * Cd; 
%loop function
L = C*Pe;

% We know for the bode criterion:
% -L crosses only once 0-dB axis: ok verified
figure();
margin(L); % low PM and GM inf -> unstable 
grid on;
% -process has no positive 
%  real-part poles: ok verified
pole(Pe)
% -static gain of the process should be positive:
%  ok verified -> 5.1532*10^16
dcgain(Pe)
% Bode criterion can be used

% It's possible to use Rlocus criterion
figure();
rlocus(L);
grid on;

%% Overall controller project
close all

% -adding a first order zero in the left half plane to attract the
%  branches in the left half plane.
% -adding a second order fast pole to make the controller function
%  proper

tau_z1 = 1/0.1;
tau_p1 = 1/0.01;

C1 = (s + tau_z1)/(s + tau_p1)^2;

% -adding a delay network for stability and better PM

alpha = 0.7;
tau = 1/10;
delayNet = (1+alpha*tau*s)/(1+tau*s);

Ce = C*C1*delayNet;

% -gain is set to obtain stability from rlocus
gain = 110;

%loop function
L = gain*Ce*Pe;

figure();
rlocus(L);
grid on;

figure();
margin(L);
grid on;

Wyr = minreal(L/(1+L));
pole(Wyr)
%poles of the Wyr are all with negative real part, we can proceed!

figure();
step(Wyr);
grid on;
stepinfo(Wyr)

%% Simulation of feedback line disturbances with a sinusoidal disturbance
close all
t = (0:0.01:10)';

nt = (4*pi/180)*sin(2*pi*3*t);

figure();
lsim(Wyr, nt, t);
grid on;

%% Rejection of sinusoidal disturbance
close all

%Building a second order notch filter for the rejection of the 
%nt disturbance, in particular it is a band-stop filter,
%having in the numerator the elements s^2+(omega_n)^2.

%Experimentally it is choosen the values ​​for omega_n and zita:
omega_n = 2*pi*3;
zita = 0.6;
H = (s^2+(omega_n)^2)/(s^2 + 2*zita*omega_n*s + omega_n^2);

%loop function
L = gain*Ce*Pe*H;

figure();
rlocus(L);
grid on;

figure();
margin(L);
grid on;

Wyr = minreal(L/(1+L));
pole(Wyr)
%poles of the Wyr are all with negative real part, we can proceed!

figure();
step(Wyr);
grid on;
stepinfo(Wyr)

%% Simulation of feedback line disturbance attenuation with a sinusoidal disturbance
close all
t = (0:0.01:10)';

nt = (4*pi/180)*sin(2*pi*3*t);

figure();
lsim(Wyr, nt, t);
grid on;
stepinfo(Wyr)
    
%% Performance: low pass filter
close all

Fr = 1/(1*s+1);
Wyr1 = minreal(Fr*L/(1+L));

stepinfo(Wyr1)
%the controller performance was effectively improved

figure();
step(Wyr1);
grid on;

%% Simulation of feedback line disturbance attenuation with a sinusoidal disturbance
close all
t = (0:0.01:10)';

nt = (4*pi/180)*sin(2*pi*3*t);

figure();
lsim(Wyr1, nt, t);
title("Reiezione del disturbo sinusoidale");
grid on;
stepinfo(Wyr1)

%% dynamic performance
close all

%control sensitivity function
Wur = minreal(Ce/(1+L));

%Design goal: Since one of the control system requirements is
%to keep the control effort 'small', it would be desirable
%for Wur to be "small"

%sensitivity function
Wer = minreal(1/(1+L)); 

%indicates a greater or lesser sensitivity to disturbances,
%therefore indicates a greater or lesser robustness of the system

figure();
step(Wur);
title('Wur');
grid on;

figure();
step(Wer);
title('Wer');
grid on;

%% Simulation for step reference
close all

t = (0:0.01:10)';

r = ones(size(t));

lsim(Wyr1,r,t)
title("simulazione per riferimento a gradino");
grid on;

%% more...

%stepinfo(out.out.Data, out.out.Time)

%% simple desaturation

desat = 1/(s+0.01);

% new low pass filter
Fr1 = 1/(2*s+1);

