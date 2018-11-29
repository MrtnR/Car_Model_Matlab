function Car_Model_Plot (tspan, x0, Pd)

global Kx Ky Ko Kphi L

Kx = Pd(1); Ky = Pd(2); Ko = Pd(3); Kphi = Pd(4);

L = 0.4;


[t, X] = ode23t(@Car_Model_Sys, tspan, x0);

end

function dX = Car_Model_Sys(t, X)

global Kx Ky Ko Kphi L


xref = cos(t);
dxref = -sin(t);
yref = sin(t);
dyref = cos(t);

ex = xref - X(1);
ey = yref - X(2);

Oref = atan2(dyref -Ky*ey, dxref - Kx*ex);

W = (dxref - Kx*ey)/(cos(Oref));

Eo = Oref - X(3);

dOref = diff(Oref);

Phi_ref = atan2(dOref - Ko*Eo,W)*L;

Ephi = Phi_ref - X(4);

dPhi_ref = diff(Phi_ref);

Wd = dPhi_ref - Kphi*Ephi;

dX = [cos(X(3)); sin(X(3)); tan((X(4)/L); 0]*W + [0; 0; 0; 1]*Wd;

end