function Car_Model_Plot (tspan, x0, Pd)

global Kx Ky Ko Kphi

Kx = Pd(1); Ky = Pd(2); Ko = Pd(3); Kphi = Pd(4);



[t, X] = ode23t()

end

function dX = Car_Model_Sys(t, X)

global Kx Ky Ko Kphi


xref = cos(t);
dxref = -sin(t);
yref = sin(t);
dyref = cos(t);

ex = xref - X(1);
ey = yref - X(2);

Oref = atan2(dyref -Ky*ey, dxref - Kx*ex);

W = (dxref - Kx*ey)/(cos(Oref));

Eo = Oref - O;

for i=1:length(Oref)
    dOref
end

Phi_ref = atan2()





end