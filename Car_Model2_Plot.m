%This function simulates a car where the rear wheels have traction
%and the steering is done by the front axel
%tspan: tiempo de simulaicon
%x0: condiciones iniciales del sistema
%Pd: polos deseados para la dinamica del error.
function Car_Model2_Plot (tspan, x0, Pd)

%Variables globales compartidas entre el sistema y la ley de control
global Kx Ky Ko L 

Kx = Pd(1); Ky = Pd(2); Ko = Pd(3); %Asignacion de los polos deseados

L = 0.3; %Longitud del carro medido del centro de las ruedas traseras al centro de las delanteras

[t, X] = ode23tb(@Car_Model2_Sys, tspan, x0); %Resolver la ecuaciones diferenciales mediante ode23tb
%Regresa el comportamiento de los estados de X Y y Ø
%X: posicion en X
%Y: posicion en Y
%Ø: angulo del carrito con respecto del eje X del sistema de referencia

%Grafica el resultado del controlador
figure;
subplot(3,1,1); plot(t, X(:,1)); title('Estado 1, Posicion en X'); %Grafica de la posicion en X
subplot(3,1,2); plot(t, X(:,2)); title('Estado 2, Posicion en Y'); %Grafica de la posicion en Y
subplot(3,1,3); plot(t, X(:,3)); title('Estado 3, Angulo de posicion'); %Grafica del angulo del carro respecto al sistema de coordenadas

%Grafica la trayectoria controlada contra la trayectoria deseada
%La trayectoria deseada es la linea punteada roja 
%La trayectoria controlada es la linea continua azul
figure;
plot(X(:,1),X(:,2), 2+ cos(t), 2+ sin(t), '--r'); title('Trayectoria del carrito');

%Se introducen las soluciones al sistema de ecuaciones diferenciales para
%obtener el comportamiento de las entradas al sistema del carrito
Ux = Car_Model2_Entrada(t' , X');

%Grafica el comportamiento de las entradas al sistema
figure;
subplot(2,1,1); plot(t,Ux(1,:)); title('Entrada 1 velocidad del carrito');
subplot(2,1,2); plot(t,Ux(2,:)); title('Entrada 2 angulo de viraje para el eje delantero de las llantas');

end

%Funcion que genera el espacio de estados con las ecuaciones diferenciales
%a resolver
%t: Tiempo de simulacion del sistema
%X: Estados del sistema
function dX = Car_Model2_Sys(t, X)

global Kx Ky Ko L %Variables globales para el sistema del carrito

%La tecnica de control para hacer que X y Y convergan a la referencia fue
%control por bloques, ya que las dos entradas al sistema influian en las
%salidas del mismo, por lo que se diseño un control por bloques para
%obtener que entrada de W y que angulo de giro en las llantas Phi son las
%que hacen que el error sea 0.

xref = 2 + cos(t); %Referencia deseada en x
dxref = -sin(t); %Primer derivada de referencia en x
ddxref = -cos(t); %Segunda derivdada de referencia en x
yref = 2 + sin(t); %Referencia deseada en y
dyref = cos(t); %Primera referencia deseada en y
ddyref = -sin(t); %Segunda referencia deseada en y

ex = xref - X(1); %Error de X
ey = yref - X(2); %Error de Y

Oref = atan2((dyref -Ky*ey) , (dxref - Kx*ex)); %Referencia para el control por bloques de Ø

W = (dxref - Kx*ex)/(cos(Oref)); %Primer entrada al sistema

dex = dxref - cos(Oref)*W; %Dinamica de error de Ex
dey = dyref - sin(Oref)*W; %Dinamica de error de Ey

Eo = Oref - X(3); %Error de Ø

%Derivada de la referencia para Ø, necesaria en el siguiente bloque de
%control.
dOref = ((ddyref - dey)*(dxref - Kx*ex) - (ddxref - dex)*(dyref - Ky*ey)/(dxref - Kx*ex)^2)/(1 + ((dyref - Ky*ey)/(dxref - Kx*ex))^2);

Phi = atan2((dOref - Ko*Eo),W)*L; %Segunda entrada del sistema 

%Sistema de ecuaciones diferenciales no lineales a resolver. 
dX = [cos(Oref); sin(Oref); tan((Phi)/L)]*W;

end

%funcion de regresa la entrada generada para controlar las salidas del
%sistema
%t: tiempo de simulacion
%X: soluciones del sistema de ecuaciones
function U = Car_Model2_Entrada(t, X)

global Kx Ky Ko L %Variables globales para el sistema a controla

j = size(t); %Numero de elementos a simular

U = zeros(2,j(2)); %Pre-alocacion para los valores de U

%Ciclo para obtener elemento a elemento las entradas calculadas para el
%sistema
for i = 1: j(2)
    
    xref = 2 + cos(t(1,i)); %xref en el elemento i
    dxref = -sin(t(1,i)); %dxref en el elemento i
    ddxref = -cos(t(1,i)); %ddxref en el elemento i
    yref = 2 + sin(t(1,i)); %yref en el elemento i
    dyref = cos(t(1,i)); %dyref en elemento i
    ddyref = -sin(t(1,i)); %ddyref en elemento i
    
    ex = xref - X(1,i); %error de x en el elemento i
    ey = yref - X(2,i); %error de y en el elemento i
    
    Oref = atan2((dyref -Ky*ey) , (dxref - Kx*ex)); %Calculo de la referencia para Ø
    
    W = (dxref - Kx*ex)/(cos(Oref)); %Calculo de la entrada W para converger los errores a 0
    
    dex = dxref - cos(Oref)*W; %Derivadas de los errores de X
    dey = dyref - sin(Oref)*W; %Derivadas de los errores de Y
    
    Eo = Oref - X(3,i); %Error de Ø en el elemento i
    
    %Derivada de la referencia de Ø
    dOref = ((dxref - Kx*ex)*(ddyref - dey) - (dyref - Ky*ey)*(ddxref - dex)/(dxref - Kx*ex).^2)/(1 + ((dyref - Ky*ey)/(dxref - Kx*ex)).^2);
    
    %Calculo de la entrada Phi para converger lso errores a 0
    Phi = atan2((dOref - Ko*Eo),W)*L;
    
    %Asignal a los correspondientes renglones de U los valores calculadores
    %de W y Phi
    U(:,i) = [W ; Phi];
    
end

end