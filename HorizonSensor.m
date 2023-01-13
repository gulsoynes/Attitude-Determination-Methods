function [N_o, N_b] = HorizonSensor(Re,Ro,omega,Omega,T,C,sigma_n,time,N)
f  = 1/298.257;         %Earth Flattening Factor
%Nadir Unit Vectors Calculations
for i = 1:N+1
    
    Lambda_geo = rad2deg(omega + 2 * pi * (time(i)/ T)); %Geocentric Latitude (deg) true anomaly'e eşit?
    R = Re * (1-f) / (sqrt(1 - (2 - f) * f * (cosd(Lambda_geo))^2));
    azim = Omega;   %Azimuth angle (deg)
    
    elev = pi/2 - acot(((Ro^2 - R^2)/Re^2 * ( 1 + ...
        ((2-f) * f * R^2 * (cosd(Lambda_geo))^2 / (1-f)^2 * Re^2 ) * (sind(azim))^2 ))^(1/2)...
        + ((2-f) * f * R^2 * sind(2*Lambda_geo) / (2*(1-f)^2*Re^2) * sind(azim)) );  %rad
    elev = rad2deg(elev); %Elevation angle (deg)
    
    %Modelled Nadir Unit Vector in orbit coordinates
    N_o0 = [cosd(elev) * cosd(azim);
        cosd(elev) * sind(azim);
        sind(elev)];
    
    N_o(:,i) = N_o0 ./ norm(N_o0);
    
    Nxo(i,1) = N_o(1,i);
    Nyo(i,1) = N_o(2,i);
    Nzo(i,1) = N_o(3,i);
    
    N_o(:,i) = [0;0;1];
    %Measured Direction Cosine of Nadir Vector in Body Frame
    N_b0 = C(i).a * N_o(:,i) + sigma_n * randn(3,1);
    N_b(:,i) = N_b0 ./ norm(N_b0);
    
    Nxb(i,1) = N_b(1,i);
    Nyb(i,1) = N_b(2,i);
    Nzb(i,1) = N_b(3,i);
    
end
end

