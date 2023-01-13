function [Euler_angle] = qtoEuler(q)
%Function to obtain Euler Angles from quaternions

q1 = q(1); q2 = q(2); q3 = q(3); q0 = q(4); %Scalar part of quaternion

roll  = atan2( (2*(q2.*q3 + q1.*q0)), ...
    (1-2*(q2^2 + q1^2)));

aSinInput = 2*(q2.*q0 - q1.*q3);
aSinInput(aSinInput > 1) = 1;
aSinInput(aSinInput < -1) = -1;

pitch = asin( aSinInput );

yaw   = atan2((2*(q1.*q2 + q3.*q0)) , ...
    (1-2*(q2^2 + q3^2)));

Euler_angle = [roll; pitch; yaw];

end

