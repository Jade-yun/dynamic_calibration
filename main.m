clear; close all; clc;

% % Define path to a urdf file
% path_to_urdf = 'ur10e.urdf';
% % addpath(genpath('D:\Desktop\codes\Matlab\dynamic_calibration'))
% 
% 
% 
% % Generate functions for dynamics based on Lagrange method
% % Note that it might take some time
% generate_rb_dynamics(path_to_urdf);
% generate_friction_eq();
% 
% 
% % Generate regressors for inverse dynamics of the robot, friction and load
% % Note that it might take some time
% generate_rb_regressor(path_to_urdf)
% % generate_load_regressor(path_to_urdf);
% 
% 
% % Run tests
% test_rb_inverse_dynamics()
% test_base_params()


% Perform QR decompostion in order to get base parameters of the robot
include_motor_dynamics = 0;
[pi_lgr_base, baseQR] = base_params_qr(include_motor_dynamics);


% Estimate drive gains
% drive_gains = estimate_drive_gains(baseQR, 'OLS');
% Or use those found in the paper by De Luca
drive_gains = [14.87; 13.26; 11.13; 10.62; 11.03; 11.47]; 


% Estimate dynamic parameters
path_to_est_data = 'ur-20_02_12-50sec_12harm.csv';    idxs = [355, 5090];
% path_to_est_data = 'ur-20_02_10-30sec_12harm.csv';      idxs = [635, 3510];
% path_to_data = 'ur-20_02_12-40sec_12harm.csv';    idxs = [500, 4460];    
% path_to_data = 'ur-20_02_05-20sec_8harm.csv';     idxs = [320, 2310];
% path_to_data = 'ur-20_02_12-50sec_12harm.csv';    idxs = [355, 5090];
% path_to_est_data = 'ur-19_12_23_free.csv'; idxs = [300, 2000];
sol = estimate_dynamic_params(path_to_est_data, idxs, ...
                              drive_gains, baseQR, 'OLS');

pi_full = baseQR.permutationMatrix(:, 1:baseQR.numberOfBaseParameters) * sol.pi_b;

% 2. 打开文件写入
filename = 'myIdentifyParameters.txt';
fid = fopen(filename, 'w');

% 3. 写入成 MATLAB 向量格式
fprintf(fid, 'pi_full = [\n');
for i = 1:length(pi_full)
    fprintf(fid, '  %.10f;\n', pi_full(i));
end
fprintf(fid, '];\n');

% 4. 关闭文件
fclose(fid);

                          
% Validate estimated parameters
path_to_val_data = 'ur-20_01_17-ptp_10_points.csv';     idxs = [700, 4200];

rre = validate_dynamic_params(path_to_val_data, idxs, ...
                              drive_gains, baseQR, sol.pi_b, sol.pi_fr)












