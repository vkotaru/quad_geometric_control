function [flats] = setpoint(t,varargin)
        
x0 = 0;
y0 = 0;
z0 = 0;


flats.x = [x0; y0; z0];

flats.dx = zeros(3,1);

flats.d2x = zeros(3,1);

flats.d3x = zeros(3,1);

flats.d4x = zeros(3,1);

flats.d5x = zeros(3,1);

flats.d6x = zeros(3,1);
        
end