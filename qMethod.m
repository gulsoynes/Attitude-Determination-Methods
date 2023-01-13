function [q_opt, Cov] = qMethod(r, b, sigma, sgn)
% Algoritm is written with,
% Markley, L., and Mortari, D., “Quaternion Attitude Estimation Using Vector Observations,” Journal of the Astronautical
% Sciences, Vol. 48, 2000, pp. 359–380. https://doi.org/10.1007/BF03546284.

b1 = b(:,1);
b2 = b(:,2);
sigma1 = sigma(1,:); %More accuarate measurement standart deviation

[~, N] = size(r);       %Number of Sensors
var = 1./ (sigma).^2;   %Weight of Sensors Measurement
B = zeros(3,3);         %Attitude Profile Matrix

for i = 1:N
    
    B = B + ((var(i,1) * b(:,i)) * r(:,i)');
    
end

z = [B(2,3) - B(3,2);
    B(3,1) - B(1,3);
    B(1,2) - B(2,1)];

%Symmetric Traceless Matrix
K = [B + B' - (trace(B) * eye(3)),  z ;
    z',  trace(B)];

%Find Normalized Eigenvectors and Eigenvalues of K Matrix
[evec, eval] = eig(K);

%Find Eigenvector of Maximum Eigenvalue of K Matrix
[~, index] = max(real(diag(eval)));

%Set Optimum Quaternion as Eigenvector of Maximum Eigenvalue of K Matrix
q_opt = evec(:,index);

%Sign check
index = find(sign(q_opt) ~= sgn);

if ~isempty(index)
    q_opt(index) = - q_opt(index);
end

temp = zeros(3,1);
for i = 1:N
    temp = temp + var(i) * b(:,i) * b(:,i)';
end
%Cov = 1/4 * sum(sigma)^2 * inv( eye(3,3) - temp );
Cov = 1/4 * (sum(sigma)^2 * eye(3,3) + norm(cross(b1,b2))^(-2) *...
    ((sigma1^2 - sum(sigma)^2) * (b2 *b2') + sum(sigma)^2 * dot(b1,b2) *...
    (b1*b2' + b2*b1')));

q4 = q_opt(4);
q3 = q_opt(3);
q2 = q_opt(2);
q1 = q_opt(1);

q_temp = [q4 -q3 q2 q1;
    q3 q4 -q1 q2;
    -q2 q1 q4 q3;
    -q1 -q2 -q3 q4];

% Covariance of q-Method
% Shuster, M. D., and Oh, S. D., “Three-axis attitude determination from vector observations,” Journal of Guidance and Control,
% Vol. 4, No. 1, 1981, pp. 70–77. https://doi.org/10.2514/3.19717.

Cov = q_temp * [Cov, zeros(3,1); zeros(1,4)] * q_temp';

end