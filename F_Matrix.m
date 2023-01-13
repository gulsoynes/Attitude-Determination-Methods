function [F] = F_Matrix(X,delt,Jx,Jy,Jz)

q1 = X(1); q2 = X(2); q3 = X(3); q0 = X(4); %Scalar part of quaternion
wx = X(5); wy = X(6); wz = X(7);

F  = [          1,  (delt*wz)/2, -(delt*wy)/2, (delt*wx)/2,             (delt*q0)/2,           -(delt*q3)/2,             (delt*q2)/2;
    -(delt*wz)/2,            1,  (delt*wx)/2, (delt*wy)/2,             (delt*q3)/2,            (delt*q0)/2,            -(delt*q1)/2;
    (delt*wy)/2, -(delt*wx)/2,            1, (delt*wz)/2,            -(delt*q2)/2,            (delt*q1)/2,             (delt*q0)/2;
    -(delt*wx)/2, -(delt*wy)/2, -(delt*wz)/2,           1,            -(delt*q1)/2,           -(delt*q2)/2,            -(delt*q3)/2;
    0,            0,            0,           0,                       1, (delt*wz*(Jy - Jz))/Jx,  (delt*wy*(Jy - Jz))/Jx;
    0,            0,            0,           0, -(delt*wz*(Jx - Jz))/Jy,                      1, -(delt*wx*(Jx - Jz))/Jy;
    0,            0,            0,           0,  (delt*wy*(Jx - Jy))/Jz, (delt*wx*(Jx - Jy))/Jz,     1];


end

