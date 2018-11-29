function Car_Model_Plot (tspan, x0, Pd)

global Kx Ky Ko Kphi L

Kx = Pd(1); Ky = Pd(2); Ko = Pd(3); Kphi = Pd(4);

L = 0.4;


[t, X] = ode23t(@Car_Model_Sys, tspan, x0);

figure;
subplot(4,1,1); plot(t, X(1,:)); title('Estado 1, Posicion en X');
subplot(4,1,2); plot(t, X(2,:)); title('Estado 2, Posicion en Y');
subplot(4,1,3); plot(t, X(3,:)); title('Estado 3, Angulo de posicion');
subplot(4,1,4); plot(t, X(4,:)); title('Estado 3, Anuglo de viraje');

figure;
plot(X(:,1),X(:,2));

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

%dOref = diff(Oref);

dOref = zeros(1,length(Oref)-1);

for i=1:length(Oref)-1
    
    dOref(1,i) = (Oref(1,i+1) - Oref(1,i))/(t(1,i+1) - t(1,i));
    
end

Phi_ref = atan2(dOref - Ko*Eo,W)*L;

Ephi = Phi_ref - X(4);

%dPhi_ref = diff(Phi_ref);
dPhi_ref = zeros(1,length(Phi_ref)-1);

for i=1:length(Phi_ref)-1
    
    dPhi_ref(1,i) = (Phi_ref(1,i+1) - Phi_ref(1,i))/((t(1,i+1) - t(1,i))); 
    
end

Wd = dPhi_ref - Kphi*Ephi;

dX = [cos(X(3)); sin(X(3)); tan((X(4))/L); 0]*W + [0; 0; 0; 1]*Wd;

end