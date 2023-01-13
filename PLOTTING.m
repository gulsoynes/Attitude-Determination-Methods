function PLOTTING(n, time,str,Euler_ang_EKF,Euler_ang_TRIAD,Euler_ang_qMethod,Euler_ang_true)

figure(1) %Euler Angle Representation
subplot(3,1,1)
plot(  time(1:n),Euler_ang_EKF(1,1:n),'.--')
hold on
plot(  time(1:n), Euler_ang_TRIAD(1,1:n),'.--')
hold on
plot(  time(1:n),Euler_ang_qMethod(1,1:n),'.--')
hold on
plot(  time(1:n),Euler_ang_true(1,1:n))
ax=gca;
ax.YGrid = 'on';
ax.XLim = [0   time(n)];
xlabel('$ t $ : Time $(s)$','Interpreter', 'latex ')
ylabel('$ \phi$ : Roll Angle $(^{\circ})$','Interpreter','latex')
lg = legend(' $ EKF $ ',  ' $TRIAD $ ' ,'$ q-Method $ ','True','Interpreter','latex');
title(lg, str)
title(str,'Interpreter', 'latex')

subplot(3,1,2)
plot(  time(1:n),Euler_ang_EKF(2,1:n),'.--')
hold on
plot(  time(1:n),Euler_ang_TRIAD(2,1:n),'.--')
hold on
plot(  time(1:n), Euler_ang_qMethod(2,1:n),'.--')
hold on
plot(  time(1:n),Euler_ang_true(2,1:n))
ax=gca;
ax.YGrid = 'on';
ax.XLim = [0   time(n)];
xlabel('$ t $ : Time $(s)$','Interpreter', 'latex ')
ylabel('$ \theta $ : Pitch Angle $(^{\circ})$','Interpreter','latex')
lg = legend(' $ EKF $ ', ' $TRIAD $ ', '$ q-Method $ ','True' ,'Interpreter','latex');
title(lg, str)

subplot(3,1,3)
plot(  time(1:n),Euler_ang_EKF(3,1:n),'.--')
hold on
plot(  time(1:n),Euler_ang_TRIAD(3,1:n),'.--')
hold on
plot(  time(1:n), Euler_ang_qMethod(3,1:n),'.--')
hold on
plot(  time(1:n),Euler_ang_true(3,1:n))
ax=gca;
ax.YGrid = 'on';
ax.XLim = [0  time(n)];
xlabel('$ t $ : Time $(s)$','Interpreter', 'latex ')
ylabel('$ \psi$ : Yaw Angle $(^{\circ})$','Interpreter','latex')
lg = legend(' $ EKF $ ', ' $TRIAD $ ' , '$ q-Method $ ','True' ,'Interpreter','latex');
title(lg, str)
 end
