% Load parameter files
clear all
load config.mat
load friction.mat

COMPILE_SFUN = 1;

% Set simulation parameters
% h = 1e-7;
% h = 1e-6;
h = 1e-5;
% omegam_ref = 171.2;
omegam_ref = 157;
% omegam_ref = 50;
% omegam_ref = 157;
% omegam_ref = 50;

log_ts = 1e-5;

Psi_jac = [0.121, 0; 0, 0.0769];

u_DC = 556;

% T_pwm = SysConfig_SamplingTimeA;

if COMPILE_SFUN == 1
    % build acados controller
    clear mex
    cd('../acados_controller/c_generated_code')
    % build MOD acados controller (parametric V_DC)
    MOD_make_sfun
    copyfile 'MOD_acados_solver_sfunction_rsm.mexa64' '../../new_model'

    % build algebraic solver
    make_algebraic_solver_sfun
    copyfile 'acados_algebraic_solver_sfun.mexa64' '../../new_model'
    
%     % build LMPC controller
%     cd('../c_generated_code_LMPC')
%     MOD_make_sfun_LMPC
%     copyfile 'MOD_acados_solver_sfunction_rsm_LMPC.mexa64' '../../new_model'
    
    % build EKF
    cd('../../export_EKF')
    modify_EKF_code
    make_sfunction_ekf
    copyfile 'acado_ekf_custom_sfun.mexa64'    '../new_model'
    
%     % build KF
%     cd('../export_EKF_LMPC')
%     modify_EKF_code_LMPC
%     make_sfunction_ekf_LMPC
%     copyfile 'acado_ekf_custom_sfun_LMPC.mexa64'    '../new_model'

    cd('../new_model')
end

W_dist_noise = diag([1e-2, 1e-2, 1e4, 1e4]);
V_dist_noise = diag([2e0, 2e0]);

P_init_dist = W_dist_noise;
x_e_init_dist = zeros(4,1);

J = [0 -1; 1 0];