function [q,w_angular,C,Euler_ang_true] = True_States(q,w,J_x,J_y,J_z,N_t,delt,N)
%Inputs : Initial values of quaternions and angular velocity.

w_x = w(1); w_y = w(2); w_z = w(3);
q1 = q(1); q2 = q(2); q3 = q(3); q0 = q(4); %Scalar part of quaternion

for i=1:N   
    %Quaternion Rates (1/s)
    
    q0_dot = -.5 * ( q1(i) * w_x(i) + q2(i) * w_y(i) + q3(i) * w_z(i) );
    q1_dot =  .5 * ( q0(i) * w_x(i) - q3(i) * w_y(i) + q2(i) * w_z(i) );
    q2_dot =  .5 * ( q3(i) * w_x(i) + q0(i) * w_y(i) - q1(i) * w_z(i) );
    q3_dot = -.5 * ( q2(i) * w_x(i) - q1(i) * w_y(i) - q0(i) * w_z(i) );
    
    %Quaternions
    
    q1(i+1,1) = q1(i) + delt * q1_dot;
    q2(i+1,1) = q2(i) + delt * q2_dot;
    q3(i+1,1) = q3(i) + delt * q3_dot;
    q0(i+1,1) = q0(i) + delt * q0_dot;
    
    %The Angular Velocities (rad/s)
    
    w_x(i+1,1) = w_x(i) + delt / (J_x) * (J_y - J_z) * w_z(i) * w_y(i) + ...
        delt / (J_x) * N_t;
    w_y(i+1,1) = w_y(i) + delt / (J_y) * (J_z - J_x) * w_x(i) * w_z(i) + ...
        delt / (J_y) * N_t;
    w_z(i+1,1) = w_z(i) + delt / (J_z) * (J_x - J_y) * w_x(i) * w_y(i) + ...
        delt / (J_z) * N_t;
    
end

q = [q1 q2 q3 q0]';
w_angular = [w_x w_y w_z]';

for i=1:N+1
    
    %To obtain unit quaternion
    q(:,i) = q(:,i) ./ norm(q(:,i));
    
    %The Transformation Matrix
    C(i).a = qtoC(q(:,i));
    
    %Euler Angle Presentation
    Euler_ang_true(:,i) = qtoEuler(q(:,i));
end

end

