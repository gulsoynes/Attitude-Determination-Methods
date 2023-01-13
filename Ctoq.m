function [q] = Ctoq(C, sgn)
%%% Ref:Markeley, page 48
% %Quaternions expressed in terms of direction cosines
q0 = sqrt(trace(C) + 1) / 2;
q1 = sqrt(1 + C(1,1) - C(2,2) - C(3,3)) / 2;
q2 = sqrt(1 - C(1,1) + C(2,2) - C(3,3)) / 2;
q3 = sqrt(1 - C(1,1) - C(2,2) + C(3,3)) / 2;

if imag(q0) > 0
    q0 = 0;
end

if imag(q1) > 0
    q1 = 0;
end

if imag(q2) > 0
    q2 = 0;
end

if imag(q3) > 0
    q3 = 0;
end

[~ , i] = max(abs([q1 q2 q3 q0]));

if i == 4
    q1 = -(C(3,2) - C(2,3)) / (4 * q0);
    q2 = -(C(1,3) - C(3,1)) / (4 * q0);
    q3 = -(C(2,1) - C(1,2)) / (4 * q0);
end

if i == 1
    q0 = -(C(3,2) - C(2,3)) / (4 * q1);
    q2 = (C(1,2) + C(2,1)) / (4 * q1);
    q3 = (C(3,1) + C(1,3)) / (4 * q1);
end

if i == 2
   q0 = -(C(1,3) - C(3,1)) / (4 * q2);
   q1 = (C(1,2) + C(2,1)) / (4 * q2);
   q3 = (C(2,3) + C(3,2)) / (4 * q2);
end

if i == 3
   q0 = -(C(2,1) - C(1,2)) / (4 * q3);
   q1 = (C(3,1) + C(1,3)) / (4 * q3);
   q2 = (C(2,3) + C(3,2)) / (4 * q3);
end

q = [q1 q2 q3 q0]';

%Sign check
index = find(sign(q) ~= sgn);

if ~isempty(index)
    q(index) = - q(index);
end

% R = transpose(C);
% 
% quat = zeros(size(R,3), 4, 'like', R);
% 
% % Calculate all elements of symmetric K matrix
% K11 = R(1,1,:) - R(2,2,:) - R(3,3,:);
% K12 = R(1,2,:) + R(2,1,:);
% K13 = R(1,3,:) + R(3,1,:);
% K14 = R(3,2,:) - R(2,3,:);
% 
% K22 = R(2,2,:) - R(1,1,:) - R(3,3,:);
% K23 = R(2,3,:) + R(3,2,:);
% K24 = R(1,3,:) - R(3,1,:);
% 
% K33 = R(3,3,:) - R(1,1,:) - R(2,2,:);
% K34 = R(2,1,:) - R(1,2,:);
% 
% K44 = R(1,1,:) + R(2,2,:) + R(3,3,:);
% 
% % Construct K matrix according to paper
% K = [...
%     K11,    K12,    K13,    K14;
%     K12,    K22,    K23,    K24;
%     K13,    K23,    K33,    K34;
%     K14,    K24,    K34,    K44];
% 
% K = K ./ 3;
% 
% % For each input rotation matrix, calculate the corresponding eigenvalues
% % and eigenvectors. The eigenvector corresponding to the largest eigenvalue
% % is the unit quaternion representing the same rotation.
% for i = 1:size(R,3)
%     [eigVec,eigVal] = eig(K(:,:,i),'vector');
%     [~,maxIdx] = max(real(eigVal));
%     quat(i,:) = real([eigVec(1,maxIdx) eigVec(2,maxIdx) eigVec(3,maxIdx) eigVec(4,maxIdx)]);
%     
%     % By convention, always keep scalar quaternion element positive. 
%     % Note that this does not change the rotation that is represented
%     % by the unit quaternion, since q and -q denote the same rotation.
%     if quat(i,1) < 0
%         quat(i,:) = -quat(i,:);
%     end
% end
% q = quat';

end