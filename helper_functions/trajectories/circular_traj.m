function [flats] = circular_traj(t)
        
x0 = 0;
y0 = 0;
z0 = 0;

phix = 0;
phiy = 0;
phiz = 0;

rx = 2;
ry = 2;
rz = 0;

f = 0.1;
omegax = 2*pi*f;
omegay = 2*pi*f;
omegaz = 2*pi*f;

flats.x = [x0+rx.*cos(phix+omegax.*t);y0+ry.*sin(phiy+omegay.*t);z0+rz.*sin( ...
  phiz+omegaz.*t)];

flats.dx = [(-1).*omegax.*rx.*sin(phix+omegax.*t);omegay.*ry.*cos(phiy+ ...
omegay.*t);omegaz.*rz.*cos(phiz+omegaz.*t)];

flats.d2x = [(-1).*omegax.^2.*rx.*cos(phix+omegax.*t);(-1).*omegay.^2.*ry.* ...
sin(phiy+omegay.*t);(-1).*omegaz.^2.*rz.*sin(phiz+omegaz.*t)];

flats.d3x = [omegax.^3.*rx.*sin(phix+omegax.*t);(-1).*omegay.^3.*ry.*cos(phiy+ ...
  omegay.*t);(-1).*omegaz.^3.*rz.*cos(phiz+omegaz.*t)];

flats.d4x = [omegax.^4.*rx.*cos(phix+omegax.*t);omegay.^4.*ry.*sin(phiy+ ...
omegay.*t);omegaz.^4.*rz.*sin(phiz+omegaz.*t)];

flats.d5x = [(-1).*omegax.^5.*rx.*sin(phix+omegax.*t);omegay.^5.*ry.*cos(phiy+ ...
  omegay.*t);omegaz.^5.*rz.*cos(phiz+omegaz.*t)];

flats.d6x = [(-1).*omegax.^6.*rx.*cos(phix+omegax.*t);(-1).*omegay.^6.*ry.* ...
sin(phiy+omegay.*t);(-1).*omegaz.^6.*rz.*sin(phiz+omegaz.*t)];
        
end