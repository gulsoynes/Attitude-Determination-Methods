function [S_o, S_b] = SunSensor(omega,Omega,inc,T,C,sigma_s,time,N)
for i = 1:N+1
    JD = JDate(2022,1,1,0,0,i-1);
    T_TDB = (JD - 2451545.0) / 36525;
    
    Lamdba_Msun = 280.460 + 36000.770 * T_TDB; %mean longitude of the sun (deg)
    M_sun = 357.5277233 + 35999.05034 * T_TDB; %mean anomaly of the sun (deg)
    Lamdba_ec = Lamdba_Msun + 1.914666471 * sind(M_sun) ...
        + 0.019994643 * sind(2 * M_sun); %ecliptic longitude of the sun (deg)
    
    %linear model of the ecliptic of the sun (deg)
    e_linear = 23.439291 - 0.0130042 * T_TDB;
    
    S_ECI = [cosd(Lamdba_ec);...
        sind(Lamdba_ec) * cosd(e_linear);...
        sind(Lamdba_ec) * sind(e_linear)];
    
    %Orbital Parameters
    v = 2 * pi * (time(i)/ T);  %true anomaly (rad)
    u = omega + rad2deg(v);     %Argument of latitude (deg)
    
    %Transformation Matrix ECI to Orbit Frame
    Tr = [-cosd(u)*cosd(inc)*sind(Omega)-sind(u)*cosd(Omega),...
        cosd(u)*cosd(inc)*cosd(Omega)-sind(u)*sind(Omega),...
        cosd(u)*sind(inc);
        ...
        -sind(inc)*sind(Omega),...
        sind(inc)*cosd(Omega),...
        -cosd(inc);
        ...
        sind(u)*cosd(inc)*sind(Omega)-cosd(u)*cosd(Omega),...
        -sind(u)*cosd(inc)*cosd(Omega)-cosd(u)*sind(Omega),...
        -sind(u)*sind(inc)];
    %Sun direction vectors in Orbit frame
    S_o0 = Tr * S_ECI;
    S_o(:,i) = S_o0 ./ norm(S_o0);
    
    Sxo(i,1) = S_o(1,i);
    Syo(i,1) = S_o(2,i);
    Szo(i,1) = S_o(3,i);
    
    %Sun direction vectors in Body frame
    S_b0 = C(i).a * S_o(:,i) + sigma_s * randn(3,1);
    S_b(:,i) = S_b0 ./ norm(S_b0);
end
end

