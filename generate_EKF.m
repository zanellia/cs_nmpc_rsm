clc;
clear all;
close all;

SIM_EXPORT = 1;
SIM_COMPILE = 1;
LINEARIZE = 0;
jac_Psi = [0.121, 0; 0, 0.0769];

DifferentialState       psid  psiq distd distq;
Control                 ud uq w;
AlgebraicState          id iq;

%% Parameters
p = 2;
theta = 0.0352;
Rs = 0.4;
m_load = 0.0;
J = [0 -1; 1 0];
kappa = 2/3;

f = acado.DifferentialEquation();       % Set the differential equation object

if LINEARIZE
    Psi = jac_Psi*[id;iq];
else
    Psi = [ psi_d_num(id,iq); psi_q_num(id,iq)];
end

f.add(dot(psid)     ==  ud - Rs*id + w*psiq + distd);
f.add(dot(psiq)     ==  uq - Rs*iq - w*psid + distq);
f.add(dot(distd)    ==  0);
f.add(dot(distq)    ==  0);
f.add(0             ==  psid - Psi(1));
f.add(0             ==  psiq - Psi(2));

NX = length(diffStates);
NU = length(controls);

Ts = 0.000250;

%% SIMexport
acadoSet('problemname', 'ekf');

numSteps = 1;
sim = acado.SIMexport( Ts );
sim.setModel(f);
sim.set( 'INTEGRATOR_TYPE',             'INT_IRK_GL2'   );
sim.set( 'NUM_INTEGRATOR_STEPS',        numSteps        );
sim.setNOD(2);

if SIM_EXPORT
    if LINEARIZE
        sim.exportCode( 'export_EKF_LMPC' );
    else
        sim.exportCode( 'export_EKF' );
    end
end

if SIM_COMPILE
    cd export_EKF
    make_acado_integrator('../ekf_rsm')
    
    modify_EKF_code
    make_sfunction_ekf
    copyfile('acado_ekf_custom_sfun.mex*', '../')
    cd ..
end