clear ;
close all;
clc;

%%components

vMax = 24;
x_0 = -250;
y_0 = -250;
theta_0 = 0;
L = 115;
r = 12;
C = 2*pi*r;
x = x_0;
y = y_0;
theta = theta_0;
t = 0;
phiDot_1 = vMax;
phiDot_r = vMax;
fEnc = 10; %Hz
dtEnc = 1/fEnc; %encoder timestep
encVar = 0.1;

%beacon components
b1 = [2500, 2500, 1000];
b2 = [-2500, 2500, 1000];
b3 = [2500, -2500, 1000];
varBea = 1000;
rBeacons = 5000;
x_dot = @(vr, v1) (vr./2 + v1./2)* dtEnc;
y_dot = @(vr, v1) (vr./2 + v1./2)* dtEnc;
theta_dot = @(vr, v1) (1/L)*(vr-v1)* dtEnc;

%wheel speeds and plot ideal loc
count = 1;
x_pos(count, :) = x;
y_pos(count, :) = y;
theta_pos(count, :) = 0;



for ii = 1:7

x_0 = x;
y_0 = y;
theta_0 = 0;
passcount = count;
if ii < 7
if mod(ii,2) == 1

while abs(x-x_0)<2000
    count = count + 1;
    x = x + x_dot(phiDot_1, phiDot_r);
    x_pos(count,:) = x;
    wheel_speeds(count,:) = [phiDot_1 phiDot_r];
    y_pos(count,:) = y;
    theta_pos(count,:) = theta;
end

while theta - theta_0 < pi/2
count = count + 1;
theta = theta + theta_dot(vMax, -vMax);
theta_pos(count,:) = theta;
wheel_speeds(count,:) =[vMax -vMax];
x_pos(count,:) = x;
y_pos(count,:) = y;
end 

while (y-y_0)<150

    count = count+1;
     y = y + y_dot(vMax, vMax);
    y_pos(count,:) = y;
    wheel_speeds(count,:) = [vMax vMax];
    x_pos(count,:) = x;
    theta_pos(count,:) = theta;

end 

while theta - theta_0 < pi
count = count + 1;
theta = theta + theta_dot(vMax, -vMax);
theta_pos(count,:) = theta;
wheel_speeds(count,:) = [vMax -vMax];
x_pos(count,:) = x;
y_pos(count,:) = y;

end

else 

    while abs(x-x_0)<2000
    count = count + 1;
    x = x - x_dot(vMax, vMax);
    x_pos(count,:) = x;
    wheel_speeds(count,:) = [vMax vMax];
    y_pos(count,:) = y;
    theta_pos(count,:) = theta;

    end

while theta - theta_0 > pi/2
    count = count + 1;
theta = theta + theta_dot(-vMax, vMax);
theta_pos(count,:) = theta;
wheel_speeds(count,:) = [-vMax vMax];
x_pos(count,:) = x;
y_pos(count,:) = y;
end

while (y-y_0)<150

    count = count+1;
    y = y + y_dot(vMax, vMax);
    y_pos(count,:) = y;
    wheel_speeds(count,:) = [vMax vMax];
    x_pos(count,:) = x;
    theta_pos(count,:) = theta;
end

while theta - theta_0 > 0
    count = count + 1;
theta = theta + theta_dot(-vMax, vMax);
theta_pos(count,:) = theta;
wheel_speeds(count,:) = [-vMax vMax];
x_pos(count,:) = x;
y_pos(count,:) = y;

end
end

if ii == 7
end

else 

 while abs(x-x_0) < 2000

     count = count + 1;
    x = x + x_dot(phiDot_1, phiDot_r);
    x_pos(count,:) = x;
    wheel_speeds(count,:) = [phiDot_1 phiDot_r];
    y_pos(count,:) = y;
    theta_pos(count,:) = theta;

 end
end
end

pose_rail = [x_pos y_pos zeros(length(x_pos),1) theta_pos];

figure(1);
plot(x_pos, y_pos,'-x');


figure(2);
view(3);
hold on;
plot3(x_pos, y_pos, zeros(length(x_pos),1),'-x');
plot3(b1(1), b1(2), b1(3),'*r');
plot3(b2(1), b2(2), b2(3),'*r');
plot3(b3(1), b3(2), b3(3),'*r');
hold off;

xlabel('X(mm)');
ylabel('Y(mm)');
zlabel('Z(mm)');

%figure(3);
%subplot(1,2,1);
%plot(wheel_speeds(:,1));
%subplot(1,2,2);
%plot(wheel_speeds(:,2));

%% add spatial noise

enc_noise = @(encVar)sqrt(encVar)*randn(1);
x_dot = @(vr, v1,theta) (vr/2 + v1/2)*cos(theta) * dtEnc;
y_dot = @(vr, v1,theta) (vr/2 + v1/2)*sin(theta) * dtEnc;
theta_dot = @(vr, v1) (1/L)*(vr-v1)* dtEnc;

%% get speed errors 
for i = 1:length(wheel_speeds)
    encError(i,1) = enc_noise(encVar);
end
for i = 1:length(wheel_speeds)
    encError(i,2) = enc_noise(encVar);
end


%% new wheel speeds 

lw_e = wheel_speeds(:,2) + encError(:,2);
rw_e = wheel_speeds(:,1) + encError(:,1);


%% reinitialise 

xt = -250;
yt = -250;
theta = 0;

%figure(3)
for ii = 1:length(lw_e)

    pose_noise(ii,1:4) = [xt, yt, 0, theta];

    xt = pose_noise(ii,1) + x_dot(rw_e(ii), lw_e(ii), pose_noise(ii,4));
    yt = pose_noise(ii,2) + y_dot(rw_e(ii), lw_e(ii), pose_noise(ii,4));
    theta = pose_noise(ii,4) + theta_dot(rw_e(ii), lw_e(ii));

end

figure;
plot(x_pos, y_pos,'-x');
hold on;
plot(pose_noise(:,1), pose_noise(:,2));
title('Scan Differences');
xlabel('X Position (m)');
ylabel('Y Positon (m)');
legend('Origional Scan', 'Scan with Added Noise');

%% EKF

%from lecturinos

bea_dist_fun = @(x, y, z, b,varBea) sqrt((x - b(1)).^2 + (y - b(2)).^2 + (z - b(3)).^2) + sqrt(varBea)*randn(length(x),1);

dist_b1 = bea_dist_fun(pose_rail(:,1), pose_rail(:,2), pose_rail(:,3),b1,0);
dist_b2 = bea_dist_fun(pose_rail(:,1), pose_rail(:,2), pose_rail(:,3),b2,0);
dist_b3 = bea_dist_fun(pose_rail(:,1), pose_rail(:,2), pose_rail(:,3),b3,0);

dist_b1_noise = bea_dist_fun(pose_rail(:,1), pose_rail(:,2), pose_rail(:,3),b1,varBea);
dist_b2_noise = bea_dist_fun(pose_rail(:,1), pose_rail(:,2), pose_rail(:,3),b2,varBea);
dist_b3_noise = bea_dist_fun(pose_rail(:,1), pose_rail(:,2), pose_rail(:,3),b3,varBea);

posBeacons = [];

R = [varBea 0 0
    0 varBea 0
    0 0 varBea];


V = diag([1 1 1]);


Q = [encVar 0
    0 encVar];

P_ini = zeros(4,4);
P = P_ini;
X_kal = zeros(length(x_pos), 4);


%initial guess of state 

X_hat_prev = pose_rail(1,:);

for i = 1:length(x_pos)

    X_hat_minus = [X_hat_prev(1) + x_dot(rw_e(i), lw_e(i), X_hat_prev(4))
                   X_hat_prev(2) + y_dot(rw_e(i), lw_e(i), X_hat_prev(4))
                   X_hat_prev(3) 
                   X_hat_prev(4) + theta_dot(rw_e(i), lw_e(i))   ];





%process jacobian matrix

A = [1 0 0 -(1/L)*(rw_e(i) + lw_e(i))*sin(X_hat_prev(4))
     0 1 0 (1/L)*(rw_e(i) + lw_e(i))*cos(X_hat_prev(4))
     0 0 1 0
     0 0 0 1];

W = [cos(X_hat_prev(4))*dtEnc/2 (L/2)*sin(X_hat_prev(4))*dtEnc
     (L/2)*cos(X_hat_prev(4))*dtEnc (L/2)*sin(X_hat_prev(4))*dtEnc
     0   0 
     (dtEnc/L)     (dtEnc/L)];

L1 = sqrt((X_hat_minus(1) - b1(1))^2 + (X_hat_minus(2) - b1(2))^2 + (X_hat_minus(3) - b1(3))^2);
L2 = sqrt((X_hat_minus(1) - b2(1))^2 + (X_hat_minus(2) - b2(2))^2 + (X_hat_minus(3) - b2(3))^2);
L3 = sqrt((X_hat_minus(1) - b3(1))^2 + (X_hat_minus(2) - b3(2))^2 + (X_hat_minus(3) - b3(3))^2);

%noise pts (b1 noise - b2 noise => x axis approx, b1 noise - b3 noise => y axis approx)
xBeacon(i) = ((dist_b1_noise(i)^2 - dist_b2_noise(i)^2 + rBeacons^2)/(2*rBeacons) - 973.421);
yBeacon(i) = ((dist_b1_noise(i)^2 - dist_b3_noise(i)^2 + rBeacons^2)/(2*rBeacons) - 2075.919);

%measurement Jacobian

H = [ (X_hat_minus(1) - b1(1))/ L1  (X_hat_minus(2) - b1(2))/ L1  (X_hat_minus(3) - b1(3))/ L1  0
      (X_hat_minus(1) - b2(1))/ L2  (X_hat_minus(2) - b2(2))/ L2  (X_hat_minus(3) - b2(3))/ L2  0
      (X_hat_minus(1) - b3(1))/ L3  (X_hat_minus(2) - b3(2))/ L3  (X_hat_minus(3) - b3(3))/ L3  0];



z = [dist_b1_noise(i)
     dist_b2_noise(i)
     dist_b3_noise(i)];

h = [ bea_dist_fun(X_hat_minus(1), X_hat_minus(2), X_hat_minus(3), b1,0);
      bea_dist_fun(X_hat_minus(1), X_hat_minus(2), X_hat_minus(3), b2,0)
      bea_dist_fun(X_hat_minus(1), X_hat_minus(2), X_hat_minus(3), b3,0)];

% kalman filter implementation 
P = A*P*A' + W*Q*W';
K = P*H'*inv(H*P*H' + V*R*V');

%ekf correction

X_hat = X_hat_minus + K*(z - h);
X_kal(i,1:4) = X_hat(1:4)';
P = (eye(4) - K*H) * P;

X_hat_prev = X_hat;

end

figure(4);
hold on;
plot(xBeacon,yBeacon,'k.');
plot(x_pos,y_pos,'r','LineWidth',4)
plot(pose_noise(:,1), pose_noise(:,2), 'b','LineWidth',2)
plot(X_kal(:,1),X_kal(:,2),'g','LineWidth', 2)

%plot(posBeacons(:,1), posBeacons(:,2), 'k.');

title('Kalman Filter Positioning Comparison');
xlabel('X Position (m)');
ylabel('Y Position (m)');
legend('Beacon Position', 'Rail Position', 'Dead Reckoning', 'Kalman Filter', 'Location', 'bestoutside');





