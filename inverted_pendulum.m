% Inverted Pendulum on a Cart - Control System

% Parameters and Assumptions
g = 9.8;     % Acceleration due to gravity (m/s^2)
m_c = 20;      % Mass of the cart (kg)
m_p = 10;    % Mass of the pendulum (kg)
l = 0.8;      % Length of the pendulum (m)
b = 10;      % Damping factor (friction and air resistance) (N.s/m)

% System matrices
A = [0 1 0 0; -b/m_c 0  (m_p*g)/m_c 0; 0 0 0 1; -b/(m_c*l) 0 -(m_c+m_p)*g/(m_c*l) 0];
B = [0; -1/m_c; 0; -1/(m_c*l)];

% LQR control design
Q = diag([1, 1, 10, 10]);  % State penalty matrix
R = 0.01;                  % Control input penalty
K = lqr(A, B, Q, R);

% Simulation parameters
t_span = 0:0.1:15;   % Time span for simulation
initial_state = [0; 0; pi/6; 0];  % Initial state: [cart_position; cart_velocity; pendulum_angle; pendulum_angular_velocity]

% Closed-loop system
sys_cl = ss(A - B, B, eye(4), []);

% Simulate the system
[t, y] = ode45(@(t, x)pendulum_cart_dynamics(t, x, -K*x, A, B), t_span, initial_state);
figure;


% Animation
subplot(2, 2, 4);
animate_pendulum_cart(t, y, l);


% Plot cart and pendulum positions
plot(t, y(:, 1), 'k', 'LineWidth', 2); % Cart position in blue
hold on;
plot(t, y(:, 3) + y(:, 1), 'b', 'LineWidth', 2); % Pendulum position in red
xlabel('Time (s)');
ylabel('Position (m)');
title('Cart and Pendulum Positions');
legend('Cart', 'Pendulum');
grid on;

function dxdt = pendulum_cart_dynamics(~, x, u, A, B)

    % State-space representation
    dxdt = A * x + B * u;
end

function animate_pendulum_cart(t, y, l)
    for i = 1:length(t)
        draw_pendulum_cart(y(i, 1), y(i, 3), l);
        pause(0);
    end
end

function draw_pendulum_cart(x, theta, l)
    clf;
    
    % Cart
    plot([x-0.2, x+0.2, x+0.2, x-0.2, x-0.2], [0, 0, 0.2, 0.2, 0], 'k', 'LineWidth', 2);
    hold on;

    % Pendulum
    pendulum_x = x + l * sin(theta);
    pendulum_y = l * cos(theta);
    line([x, pendulum_x], [0, pendulum_y], 'LineWidth', 2);

    axis equal;
    xlim([-2, 2]);
    ylim([-1, 1]);
    title('Pendulum and Cart Animation');
    drawnow;
end