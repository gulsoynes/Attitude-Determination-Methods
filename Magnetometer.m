function [H_o, H_b] = Magnetometer(Ro,inc,C,sigma_h,time,N)
Me = 7.943e15;          %Magnetic Dipole Moment of Earth (Wb.m)
We = 7.29e-5;           %Spin Rate of Earth (rad/s)
e = deg2rad (11.7);     %Magnetic Dipole Tilt (rad)
mu = 3.98601e14;        %Earth Gravitational Constant (m^3/s^2)
Wo = sqrt( mu / ( Ro ^ 3 ) );   %Angular Velocity of Orbit (rad/s)
%b_c = [.04; .06; .08];      %magnetometer bias vector
for i=1:N+1
    %Magnetic Field Vector in Orbit Frame
    %X Component of Magnetic Field Vector (T)
    Hx = ( Me / Ro^3 ) * ( cos( Wo*time(i) ) * ( cos(e) * sind(inc) ...
        - sin(e) * cosd(inc) * cos ( We*time(i) ) ) ...
        - sin ( Wo*time(i)) * sin(e) * sin ( We*time(i)) );
    
    %Y Component of Magnetic Field Vector (T)
    Hy = ( -Me / Ro^3 ) * ( cos(e) * cosd(inc) ...
        + sin(e) * sind(inc) * cos ( We*time(i) ) );
    
    %Z Component of Magnetic Field Vector (T)
    Hz = ( 2 * Me / Ro^3 ) * ( sin (Wo * time(i) ) * (cos(e) * sind(inc) ...
        -sin(e)*cosd(inc)*cos(We*time(i))) ...
        - 2*sin(Wo*time(i))*sin(e)*sin(We*time(i)));
    
    H = [Hx; Hy; Hz]; %Magnetic Field Vector
    
    %Direction Cosine Components of Ho
    Hxo(i,1) = Hx / norm(H) ;
    Hyo(i,1) = Hy / norm(H);
    Hzo(i,1) = Hz / norm(H);
    
    %Direction Cosine of Magnetic Field Vector in Orbit Reference
    H_o(:,i) = [Hxo(i,1) ; Hyo(i,1); Hzo(i,1)];
    
    %Measured Direction Cosine of Magnetic Field Vector in Body Frame
    H_b0 = C(i).a * H_o(:,i) + sigma_h * randn(3,1);
    H_b(:,i) = H_b0 ./ norm(H_b0);
    
    Hxb(i,1) = H_b(1,i);
    Hyb(i,1) = H_b(2,i);
    Hzb(i,1) = H_b(3,i);
end

end

