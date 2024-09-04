function [] = plotqd(Q_arm,Ts,N)
    tiles = tiledlayout(3,2);

    nexttile
    plot(Ts*(1:(N-1)),1/Ts*diff(Q_arm(1,:)))
    title('$\dot{q_1}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(1:(N-1)),1/Ts*diff(Q_arm(2,:)))
    title('$\dot{q_2}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(1:(N-1)),1/Ts*diff(Q_arm(3,:)))
    title('$\dot{q_3}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(1:(N-1)),1/Ts*diff(Q_arm(4,:)))
    title('$\dot{q_4}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(1:(N-1)),1/Ts*diff(Q_arm(5,:)))
    title('$\dot{q_5}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(1:(N-1)),1/Ts*diff(Q_arm(6,:)))
    title('$\dot{q_6}$','Interpreter','latex','FontSize',13)
end