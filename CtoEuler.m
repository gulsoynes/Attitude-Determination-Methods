function [roll, pitch, yaw] = CtoEuler(C)

yaw = atan2(C(1,2) , C(1,1));

pitch = - acos(C(3,3));

roll = atan2(C(3,1) , -C(3,2));

end

