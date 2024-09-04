function [] = plotqdd(Q_arm,Ts,N)
    tiles = tiledlayout(3,2);

    nexttile
    plot(Ts*(2:(N-1)),1/Ts^2*diff(Q_arm(1,:),2))
    title('$\ddot{q_1}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(2:(N-1)),1/Ts^2*diff(Q_arm(2,:),2))
    title('$\ddot{q_2}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(2:(N-1)),1/Ts^2*diff(Q_arm(3,:),2))
    title('$\ddot{q_3}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(2:(N-1)),1/Ts^2*diff(Q_arm(4,:),2))
    title('$\ddot{q_4}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(2:(N-1)),1/Ts^2*diff(Q_arm(5,:),2))
    title('$\ddot{q_5}$','Interpreter','latex','FontSize',13)

    nexttile
    plot(Ts*(2:(N-1)),1/Ts^2*diff(Q_arm(6,:),2))
    title('$\ddot{q_6}$','Interpreter','latex','FontSize',13)
end