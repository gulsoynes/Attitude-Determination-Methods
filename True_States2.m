function [X] = True_States2(X,J_x,J_y,J_z,N_t,delt)
%Inputs : Initial values of quaternions and angular velocity.

w_x = X(5); w_y = X(6); w_z = X(7);
q1 = X(1); q2 = X(2); q3 = X(3); q0 = X(4); %Scalar part of quaternion

    %Quaternion Rates (1/s)
    
    q0_dot = -.5 * ( q1 * w_x + q2 * w_y + q3 * w_z );
    q1_dot =  .5 * ( q0 * w_x - q3 * w_y + q2 * w_z );
    q2_dot =  .5 * ( q3 * w_x + q0 * w_y - q1 * w_z );
    q3_dot = -.5 * ( q2 * w_x - q1 * w_y - q0 * w_z );
    
    %Quaternions
    
    q1(:,1) = q1 + delt * q1_dot;
    q2(:,1) = q2 + delt * q2_dot;
    q3(:,1) = q3 + delt * q3_dot;
    q0(:,1) = q0 + delt * q0_dot;
    
    %The Angular Velocities (rad/s)
    
    w_x(:,1) = w_x + delt / (J_x) * (J_y - J_z) * w_z * w_y + ...
        delt / (J_x) * N_t;
    w_y(:,1) = w_y + delt / (J_y) * (J_z - J_x) * w_x * w_z + ...
        delt / (J_y) * N_t;
    w_z(:,1) = w_z + delt / (J_z) * (J_x - J_y) * w_x * w_y + ...
        delt / (J_z) * N_t;
    
q = [q1 q2 q3 q0]';
q = q ./ norm(q);
w_angular = [w_x w_y w_z]';
X = [q; w_angular];
end


