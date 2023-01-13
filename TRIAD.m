function [q_TRIAD, Cov, A_est] = TRIAD(r,b, sigma, sgn)
% Algorithm is written by
%  Markley, F. L., “Attitude Determination Using Two Vector Measurements,” 1998

r1 = r(:,1); %More accuarate measurement in reference frame
r2 = r(:,2); %Less accuarate measurement in reference frame

b1 = b(:,1); %More accuarate measurement in body frame
b2 = b(:,2); %Less accuarate measurement in body frame

sigma1 = sigma(1,:); %More accuarate measurement standart deviation
sigma2 = sigma(2,:); %Less accuarate

v1 = r1;    %1st component of TRIAD in reference frame
v2 = cross(r1, r2) / norm(cross(r1 , r2));
v3 = cross(r1, v2);
v = [v1 v2 v3]; %TRIAD frame in terms of reference vectors

w1 = b1;    %1st component of TRIAD in body frame
w2 = cross(b1 , b2) / norm(cross(b1 , b2));
w3 = cross(b1, w2);
w = [w1 w2 w3]; %TRIAD frame in terms of observation vectors in body

A_est = v * w'; %Estimated Attitude Matrix with TRIAD
A_est = transpose(A_est);
q_TRIAD =  Ctoq(A_est, sgn);

%COVARIANCE MATRIX of TRIAD 
% Shuster, M. D., and Oh, S. D., “Three-axis attitude determination from vector observations,” Journal of Guidance and Control,
% Vol. 4, No. 1, 1981, pp. 70–77. https://doi.org/10.2514/3.19717.

Cov = (sigma1^2 * eye(3)) + ...
    ((((sigma2^2 - sigma1^2) * (b1 * b1')) + ...
    (sigma1^2 * (dot(b1, b2) * ((b1* b2') + (b2 * b1'))))) / ...
    ((norm(cross(b1, b2)))^2));

end