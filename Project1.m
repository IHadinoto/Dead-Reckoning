    %% Problem 1
close all;
%.......................................................................

dt=0.001 ; % Step, for Euler approximation: 1ms (expressed in seconds)
duration = 7 ; % Simulation horizon
t = 0:dt:duration;
N = length(t)-1;
%.....................................

% Constants
A = 110;
B = 2.2;
C = 1.1;

% Part c1)u=0
% Starting Values
X = zeros(2,1);
X(1) = deg2rad(110);
u = 0;

for k = 1:N
    dX = [X(2,k);-A*sin(X(1,k))-B*X(2,k)+C*u];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
figure;plot(t,rad2deg(X));

% Part c2)u=3
X = zeros(2,1);
X(1) = deg2rad(110);
u = 10;

for k = 1:N
    dX = [X(2,k);-A*sin(X(1,k))-B*X(2,k)+C*u];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
figure;plot(t,rad2deg(X));

%% Problem 2
close all;

dt=0.01 ; % Step, for Euler approximation: 10ms (expressed in seconds)
duration = 10 ; % Simulation horizon
t = 0:dt:duration;
N = length(t)-1;

% Constants
L = 2.5;

% Part c1)ak=30deg
% Starting Values
X = zeros(3,1);
vk = 3.5;
ak = deg2rad(45);
    
for k = 1:N
    dX = [vk*cos(X(3,k));vk*sin(X(3,k));tan(ak)*vk/L];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
figure;plot(X(1,:),X(2,:));

% Part c2)ak changing
% Starting Values
X = zeros(3,1);
vk = 3.5;
ak = deg2rad(45);
turn = 0;

for k = 1:N
    turn = turn+1;
    if turn == N/2
        ak = -ak;
        turn = 0;
    end
    dX = [vk*cos(X(3,k));vk*sin(X(3,k));tan(ak)*vk/L];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
figure;plot(X(1,:),X(2,:));

% Part c4)L changing (takes longer to turn)
% Constants
L = 3;

% c1
% Starting Values
X = zeros(3,1);
vk = 3.5;
ak = deg2rad(45);
    
for k = 1:N
    dX = [vk*cos(X(3,k));vk*sin(X(3,k));tan(ak)*vk/L];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
figure;plot(X(1,:),X(2,:));

% c2
% Starting Values
X = zeros(3,1);
vk = 3.5;
ak = deg2rad(45);
turn = 0;

for k = 1:N
    turn = turn+1;
    if turn == N/2
        ak = -ak;
        turn = 0;
    end
    dX = [vk*cos(X(3,k));vk*sin(X(3,k));tan(ak)*vk/L];
    
    % Euler's Approximation
    X(:,k+1)=X(:,k)+dt*dX;
end
figure;plot(X(1,:),X(2,:));