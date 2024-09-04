%Clear memory,graphs and output.

clc
clf
clear

%Initialize robot arm and its parameters

robot = mdl_ur10e();    %Initialize the robot arm SerialLink

q0 = [-0.140 ; -1.556 ; -1.359 ; 1.425 ; -1.053 ; -1.732]; %Initial joint positions.

q_dot_lim = [2*pi/3 ; 2*pi/3 ; pi ; pi ; pi ; pi]; %Maximum joint speeds.

q_ddot_lim = 250*ones(6,1); %Maximum joint accelaration.

fc = q_ddot_lim./(2*q_dot_lim); %LPF cut-off frequency.

%Initialize constant parameters (homogeneous trasnformations) and some
%initial values for variables

g_0c = SE3(eye(3),[0.4 0 0.2]);

g_be_d = SE3(roty(180),[0 0 0.45]);

x_cb = [1 ; 0 ; 0];
R_cb = [1 0 0 ; 0 0.9351 0.3543 ; 0 -0.3543 0.9351];

%Controller gains
k_p = 100;
k_l = 100;
K = [k_p*eye(3),zeros(3,3) ; zeros(3,3),k_l*eye(3)];

%Simulation parameters

Ts = 2*1e-3;    %Simulation time step

N = 5000;        %Simulation iterarions (number of steps)

ws = Wspace();  %Wspace (workspace) object initialization for simulating the moving ball.

%Simulation loop

q = q0;

    %Matrices used to store the simulation results
    P_be = zeros(3,N);
    R_be = zeros(4,N);
    Q_arm = zeros(6,N);

for i = 1:N
    [p_cb,v_cb,w_cb] = ws.sim_ball(Ts);

    if norm(v_cb) ~= 0  %Taking advantage of v_cb orientation to determine R_cb
        y_cb = sign(v_cb(2))*v_cb./norm(v_cb);
        z_cb = cross(x_cb,y_cb);
        R_cb = [x_cb,y_cb,z_cb];
    end

    g_cb = SE3(R_cb,p_cb);
    g_0b = g_0c*g_cb;

    Q_arm(1:6,i) = q;
    g_0e = robot.fkine(q);  %Solving the forward kinematic problem

    g_be = inv(g_0b)*g_0e;
    P_be(1:3,i) = g_be.t;
    [R_be(1,i),R_be(2:4,i)] = g_be.toangvec('deg');

    %Axis-angle modification for consistent representation.
    if R_be(3,i) < 0
        R_be(2:4,i) = -R_be(2:4,i);
        R_be(1,i) = -R_be(1,i);
        if R_be(1,i) < 0
            R_be(1,i) = R_be(1,i) + 360;
        end
    end

    %Calculating desired velocity and errors.
    g_d = g_0b*g_be_d;

    e_p = g_0e.t - g_d.t;

    g_error = g_0e * inv(g_d);
    [theta,k] = g_error.toangvec();
    e_l = theta*k';

    R_cb_dot = skew(w_cb)*g_cb.R;

    p_d_dot = R_cb_dot*g_be_d.t + v_cb;
    w_d = w_cb;

    u = [p_d_dot ; w_d] - K*[e_p ; e_l];
    j0 = robot.jacob0(q);
    q_dot = inv(j0)*u;

    q_dot = satq(q_dot,q_dot_lim);  %q_dot saturation
    
    if i == 1
        q_dot_filtered = q_dot;
    end

    q = q + q_dot_filtered*Ts;  %q integration

    q_dot_filtered = lpf(q_dot,q_dot_filtered,fc,Ts);   %lpf filter to contain q_ddot.
end

%Plots
figure(1)
plotp(P_be,Ts,N)

figure(2)
plotR(R_be,Ts,N)

figure(3)
plotq(Q_arm,Ts,N)

figure(4)
plotqd(Q_arm,Ts,N)

figure(5)
plotqdd(Q_arm,Ts,N)