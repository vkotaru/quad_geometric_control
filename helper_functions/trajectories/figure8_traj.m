function [x, dx, d2x, d3x, d4x, d5x, d6x] = figure8_traj(t)
        
x0 = 0;
y0 = 0;
z0 = 0;

phix = 0;
phiy = 0;
phiz = 0;

rx = 5;
ry = 5;
rz = 0;

f = 0.1;
omegax = 2*pi*f;
omegay = 2*pi*f;
omegaz = 2*pi*f;

x = [x0+rx.*sin(phix+omegax.*t);y0+ry.*cos(phiy+omegay.*t).*sin(phiy+ ...
  omegay.*t);z0+rz.*sin(phiz+omegaz.*t)];

dx = [omegax.*rx.*cos(phix+omegax.*t);omegay.*ry.*cos(phiy+omegay.*t) ...
 .^2+(-1).*omegay.*ry.*sin(phiy+omegay.*t).^2;omegaz.*rz.*cos(phiz+ ...
  omegaz.*t)];

d2x = [(-1).*omegax.^2.*rx.*sin(phix+omegax.*t);(-4).*omegay.^2.*ry.* ...
cos(phiy+omegay.*t).*sin(phiy+omegay.*t);(-1).*omegaz.^2.*rz.*sin( ...
  phiz+omegaz.*t)];

d3x = [(-1).*omegax.^3.*rx.*cos(phix+omegax.*t);(-4).*omegay.^3.*ry.* ...
cos(phiy+omegay.*t).^2+4.*omegay.^3.*ry.*sin(phiy+omegay.*t).^2;( ...
  -1).*omegaz.^3.*rz.*cos(phiz+omegaz.*t)];

d4x = [omegax.^4.*rx.*sin(phix+omegax.*t);16.*omegay.^4.*ry.*cos(phiy+ ...
 omegay.*t).*sin(phiy+omegay.*t);omegaz.^4.*rz.*sin(phiz+omegaz.*t) ...
  ];

d5x = [omegax.^5.*rx.*cos(phix+omegax.*t);16.*omegay.^5.*ry.*cos(phiy+ ...
 omegay.*t).^2+(-16).*omegay.^5.*ry.*sin(phiy+omegay.*t).^2; ...
omegaz.^5.*rz.*cos(phiz+omegaz.*t)];

d6x = [(-1).*omegax.^6.*rx.*sin(phix+omegax.*t);(-64).*omegay.^6.*ry.* ...
 cos(phiy+omegay.*t).*sin(phiy+omegay.*t);(-1).*omegaz.^6.*rz.*sin( ...
  phiz+omegaz.*t)];
        
end