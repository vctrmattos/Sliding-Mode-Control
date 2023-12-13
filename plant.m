function [sys,x0,str,ts,simStateCompliance] = plant(t,x,u,flag)
switch flag

  case 0
    [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes;

  case 1
    sys=mdlDerivatives(t,x,u);

  case 3
    sys=mdlOutputs(t,x,u);

  case {2, 4, 9}
    sys=mdlTerminate(t,x,u);

  otherwise
    DAStudio.error('Simulink:blocks:unhandledFlag', num2str(flag));

end

function [sys,x0,str,ts,simStateCompliance]=mdlInitializeSizes

sizes = simsizes;

sizes.NumContStates  = 2;
sizes.NumDiscStates  = 0;
sizes.NumOutputs     = 2;
sizes.NumInputs      = 1;
sizes.DirFeedthrough = 0;
sizes.NumSampleTimes = 1;   % at least one sample time is needed

sys = simsizes(sizes);

% initialize the initial conditions
x0  = [-8 13];
str = [];
ts  = [0 0];

simStateCompliance = 'UnknownSimState';

function sys=mdlDerivatives(t,x,u)
x1 = x(1);
x2 = x(2);

%Define the sys by it's state space equations
dx1 = x2;
f = x1 + x2 - 2*x1^3;
dx2 = u - f;

sys = [dx1 dx2];

function sys=mdlOutputs(t,x,u)
sys = [x(1) x(2)];

function sys=mdlTerminate(t,x,u)

sys = [];
