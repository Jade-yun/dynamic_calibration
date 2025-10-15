function frctn = frictionRegressor(qd_fltrd)
% ----------------------------------------------------------------------
% The function computes friction regressor for each joint of the robot.
% Fv*qd + Fc*sign(qd) + F0, and the second one is continous,
% ---------------------------------------------------------------------
noJoints = size(qd_fltrd,1);
frctn = zeros(noJoints, noJoints*3);
for i = 1:noJoints
    % frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
    frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), atan(qd_fltrd(i))];

    % epsilon = 1 * e-5;
    % if (abs(qd_fltrd(i)) > epsilon)
    %     frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
    % else
    %     frctn(i,3*i-2:3*i) = [qd_fltrd(i), 0, 1];
    %     k = 10;
    %     smooth_sign = 2 ./ (1 + exp(-k * qd_fltrd(i))) - 1;
    %     frctn(i,3*i-2:3*i) = [qd_fltrd(i), smooth_sign, 1];
    % end

    % dv = 0.0001;
    % 
    % qd = qd_fltrd(i);
    % abs_qd = abs(qd);
    % sign_qd = sign(qd);
    % exp_term = 1 - exp(-abs_qd / dv);  % dv factor separated into regressor
    % 
    % % Construct the regressor row for joint i
    % % The model is:
    % % tau_f = fv*qd + fc*sign(qd)*(1 - exp(-abs(qd)/dv))
    % % Regressor: [qd, sign(qd)*(1 - exp(-abs(qd))), sign(qd)*(1 - exp(-abs(qd)))*abs(qd)]
    % % frctn(i,3*i-2:3*i) = [qd, sign_qd * exp_term, sign_qd * exp_term * abs_qd];
    % frctn(i,3*i-2:3*i) = [qd, sign_qd * exp_term, 1];

    % if i < 6
    %     frctn(i,3*i-2:3*i) = [qd_fltrd(i), sign(qd_fltrd(i)), 1];
    % else
    %     % 第 6 关节及以后使用平滑 + 滤波处理
    %     k = 10;
    %     smooth_sign = 2 ./ (1 + exp(-k * qd_fltrd(i))) - 1;
    %     frctn(i,3*i-2:3*i) = [qd_fltrd(i), smooth_sign, 1];
    % end
end

% % ----------------------------------------------------------------------
% % Computes the friction regressor for dynamic nonlinear friction model
% % described in Olsson (1996) thesis.
% %
% % Inputs:
% %   qd_fltrd : [n x 1] joint velocities (filtered)
% %   z_prev   : [n x 1] previous internal state z
% %   Ts       : sampling time
% %
% % Output:
% %   frctn    : [n x 3n] regressor matrix per joint: [z, dz, v]
% % ----------------------------------------------------------------------
% 
% % --- PARAMETERS ---
% FC = 1;     % Coulomb friction level (tune via identification)
% FS = 1.5;   % Static friction level (tune via identification)
% vS = 0.01;  % Stribeck velocity (small positive)
% sigma_0 = 100;  % stiffness (example value)
% 
% z_prev = zeros(n,1);            % initial internal state
% Ts = 0.01; 
% 
% noJoints = size(qd_fltrd, 1);
% frctn = zeros(noJoints, noJoints * 3);
% z_next = zeros(noJoints, 1);  % updated internal state
% 
% for i = 1:noJoints
%     v = qd_fltrd(i);
%     z = z_prev(i);
% 
%     % Compute rho(v)
%     rho = (FC + (FS - FC) * exp(-(v / vS)^2)) / sigma_0;
% 
%     % Compute dz
%     dz = v - rho * z;
% 
%     % Euler integration to update z
%     z_next(i) = z + Ts * dz;
% 
%     % Fill in regressor for joint i
%     frctn(i, 3*i-2 : 3*i) = [z, dz, v];
% end
% 
% % Optional: return updated z_next if needed in future
% assignin('base', 'z_next', z_next);  % store in workspace (or change as needed)

