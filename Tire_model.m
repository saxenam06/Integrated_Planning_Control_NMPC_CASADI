function [Fyf,Fyr] = Tire_model(vp_x,vp_y,om_z,str_f)

%%%% Vehicle Parameters
mass = 1723;  % in kgs
lf=1.232; lr=1.468;  % front, rear axle to c.g. location in m
w=1.35; % Track width
H=0.6; % C.G. Height
Iz=4175; %Vehicle inertia about Z axis in kg-m^2
L=lf+lr; % Wheel Base
Cf=66900; % N/rad Front tire linear cornering stiffness
Cr=62700; %NN/rad Rear Tire Linear Cornering Stiffness


sf=str_f-atan((vp_y+(lf*om_z))/vp_x);
sr=-atan((vp_y-(lr*om_z))/vp_x);

Fyf=Cf*sf;
Fyr=Cr*sr;

end
