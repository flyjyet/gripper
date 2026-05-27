% Gripper motion curves for the TS2000 gripper linkage.
%
% Sign convention:
%   s      : nut stroke, positive downward from fully-open position, mm
%   phi    : jaw closing angle, positive in the closing direction, rad
%   vNut   : ds/dt, positive downward, mm/s
%   aNut   : d2s/dt2, positive downward, mm/s^2
%
% Kinematic relations:
%   omega = d(phi)/dt = J(s) * vNut
%   alpha = d2(phi)/dt2 = K(s) * vNut^2 + J(s) * aNut
% where:
%   J(s) = d(phi)/ds
%   K(s) = d2(phi)/ds2

clear; clc; close all;

%% User inputs
vNut = 1.0;       % mm/s, set this to the required nut speed
aNut = 0.0;       % mm/s^2, set this to the required nut acceleration
nPts = 1001;      % curve sample count

%% Geometry, units are mm
A = [-12.0, 0.0];          % fixed joint 1, left side
B0 = [-13.0, 28.5];        % nut/jaw joint 2 at fully open
C0 = [-47.99, 27.83];      % link joint 3 at fully open

L1 = 45.5;                 % A-C length
L2 = 35.0;                 % B-C length
stroke = 16.0;             % actual nut stroke

theta0 = atan2(C0(2) - B0(2), C0(1) - B0(1));
s = linspace(0, stroke, nPts).';

theta = zeros(nPts, 1);    % jaw rigid body angle relative to fully open
Cx = zeros(nPts, 1);
Cy = zeros(nPts, 1);

for i = 1:nPts
    B = [B0(1), B0(2) - s(i)];
    [P1, P2] = circle_intersections(A, L1, B, L2);

    % The physical branch is the left-side intersection.
    if P1(1) <= P2(1)
        C = P1;
    else
        C = P2;
    end

    Cx(i) = C(1);
    Cy(i) = C(2);
    theta(i) = wrap_pi(atan2(C(2) - B(2), C(1) - B(1)) - theta0);
end

% theta is negative during closing with the current coordinate system.
% phi is the positive closing angle, which is easier to read in plots.
phi = -theta;

J = gradient(phi, s);       % rad/mm
K = gradient(J, s);         % rad/mm^2

omega = J .* vNut;          % rad/s
alpha = K .* vNut.^2 + J .* aNut;  % rad/s^2

%% Print key points
fprintf('Stroke range: %.3f to %.3f mm\n', s(1), s(end));
fprintf('Closing angle: %.3f to %.3f deg\n', rad2deg(phi(1)), rad2deg(phi(end)));
fprintf('J=dphi/ds range: %.6f to %.6f rad/mm\n', min(J), max(J));
fprintf('K=d2phi/ds2 range: %.6f to %.6f rad/mm^2\n', min(K), max(K));
fprintf('At vNut=%.3f mm/s, omega range: %.3f to %.3f deg/s\n', ...
    vNut, min(rad2deg(omega)), max(rad2deg(omega)));
fprintf('At vNut=%.3f mm/s and aNut=%.3f mm/s^2, alpha range: %.3f to %.3f deg/s^2\n', ...
    vNut, aNut, min(rad2deg(alpha)), max(rad2deg(alpha)));

%% Plot three requested curves
fig = figure('Name', 'Gripper kinematic curves', 'Color', 'w');
tiledlayout(3, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

nexttile;
plot(s, rad2deg(phi), 'LineWidth', 1.8);
grid on;
xlabel('Nut stroke s (mm)');
ylabel('Jaw closing angle \phi (deg)');
title('Jaw angle - nut stroke');

nexttile;
yyaxis left;
plot(s, rad2deg(J), 'LineWidth', 1.8);
ylabel('Velocity ratio d\phi/ds (deg/mm)');
yyaxis right;
plot(s, rad2deg(omega), '--', 'LineWidth', 1.5);
ylabel(sprintf('Jaw angular velocity \\omega at vNut=%.3g mm/s (deg/s)', vNut));
grid on;
xlabel('Nut stroke s (mm)');
title('Jaw angular velocity - nut velocity');

nexttile;
yyaxis left;
plot(s, rad2deg(K), 'LineWidth', 1.8);
ylabel('Geometry term d^2\phi/ds^2 (deg/mm^2)');
yyaxis right;
plot(s, rad2deg(alpha), '--', 'LineWidth', 1.5);
ylabel(sprintf('Jaw angular acceleration \\alpha at aNut=%.3g mm/s^2 (deg/s^2)', aNut));
grid on;
xlabel('Nut stroke s (mm)');
title('Jaw angular acceleration - nut acceleration');

%% Save outputs next to this script
scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

pngPath = fullfile(scriptDir, 'gripper_motion_curves.png');
csvPath = fullfile(scriptDir, 'gripper_motion_curves.csv');

exportgraphics(fig, pngPath, 'Resolution', 200);

out = table( ...
    s, ...
    rad2deg(phi), ...
    J, ...
    rad2deg(J), ...
    K, ...
    rad2deg(K), ...
    omega, ...
    rad2deg(omega), ...
    alpha, ...
    rad2deg(alpha), ...
    Cx, ...
    Cy, ...
    'VariableNames', { ...
        'nut_stroke_mm', ...
        'jaw_closing_angle_deg', ...
        'dphi_ds_rad_per_mm', ...
        'dphi_ds_deg_per_mm', ...
        'd2phi_ds2_rad_per_mm2', ...
        'd2phi_ds2_deg_per_mm2', ...
        'jaw_omega_rad_per_s', ...
        'jaw_omega_deg_per_s', ...
        'jaw_alpha_rad_per_s2', ...
        'jaw_alpha_deg_per_s2', ...
        'joint3_x_mm', ...
        'joint3_y_mm' ...
    });
writetable(out, csvPath);

fprintf('Saved figure: %s\n', pngPath);
fprintf('Saved data:   %s\n', csvPath);

%% Local functions
function [P1, P2] = circle_intersections(A, r0, B, r1)
    dx = B(1) - A(1);
    dy = B(2) - A(2);
    d = hypot(dx, dy);

    if d > r0 + r1 || d < abs(r0 - r1) || d == 0
        error('No circle intersection for current linkage position.');
    end

    along = (r0^2 - r1^2 + d^2) / (2 * d);
    h2 = r0^2 - along^2;
    h = sqrt(max(0, h2));

    xm = A(1) + along * dx / d;
    ym = A(2) + along * dy / d;

    rx = -dy * h / d;
    ry = dx * h / d;

    P1 = [xm + rx, ym + ry];
    P2 = [xm - rx, ym - ry];
end

function y = wrap_pi(x)
    y = mod(x + pi, 2 * pi) - pi;
end
