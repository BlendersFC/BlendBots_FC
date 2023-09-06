clear all
close all
clc

%Dimensiones de robot (cm)
l1 = 6.475;         %Planta de pies
l2 = 12.5325;       %Tobillo-rodilla
l3 = 12.5325;       %Rodilla-cadera
offset = 7;       %Cadera-torso

%Objetivo
%limite_sentado = 14.75;
Oy = linspace(14.75,(l2+l3)+offset-4,40)
%Oy = (14.75)*ones(1,40);
%for i = 1:3
%     Oy_aux = linspace(27.5,25,20);
%     Oy = [Oy Oy_aux];
%     Oy_aux = linspace(25,27.5,20);
%     Oy = [Oy Oy_aux];
% end
% Oy = [Oy linspace(27.5,(l2+l3)+offset-0.5,40)];
sz = size(Oy);
Ox = 0*ones(1,sz(1,2));
%Ox = linspace(0,2,40);
%Objetivo
% % limite_sentado = 14.75;
% pose_parado = (l2+l3+offset)-4;       %En cm
% Ox = linspace(0,0.5,10);
% sz = size(Ox);
% Oy = pose_parado*ones(1,sz(1,2));

% %Circulo
% r = 30;
% h = 0;
% k = 0;
% 
% th = pi/2:pi/100:pi*(19/32);
% Ox = r * cos(th) + h;
% Oy = r * sin(th) + k;
% sz = size(Ox);


%Para graficación
X = Ox;
Y = Oy - offset;

%Brazo derecho
lado = 1;
q1r = [];
q2r = [];
q3r = [];

figure;
for i = 1:sz(1,2)
    Qr = calc_cinem_inv_3eslabones(X(1,i),Y(1,i),l2,l3,offset,lado);
    q1r = [q1r Qr(1,1)];
    q2r = [q2r Qr(1,2)];
    q3r = [q3r Qr(1,3)];
    
    %Tabla Denavith-Hartenberg
    a1r=l2; alpha1r=0; d1r=0; th1r = 0 + q1r(1,i);
    a2r=l3; alpha2r=0; d2r=0; th2r = 0 + q2r(1,i);
    
    %Transformaciones
    T1r=calc_A(a1r,alpha1r,d1r,th1r);
    T2r=calc_A(a2r,alpha2r,d2r,th2r);
    
    %Cinemática directa (multiplicación de matrices)
    Tnr=T1r*T2r;
    
    %Punto del brazo final
    Xrobot = Tnr(1,4);
    Yrobot = Tnr(2,4);
    
    %Graficar
    hold on
    plot(Ox(1,i),Oy(1,i),'sg',Xrobot,Yrobot,'or',T1r(1,4),T1r(2,4),'ob',T1r(1,4),T1r(2,4),'or');
    xlim([-15,15]);
    ylim([-1,29]);
    grid on
%     line([0,T1r(1,4)],[0,T1r(2,4)],'color','red');
%     line([T1r(1,4),Tnr(1,4)],[T1r(2,4),Tnr(2,4)],'color','blue');
%     line([X(1,i),Ox(1,i)],[Y(1,i),Oy(1,i)],'color','magenta');
%     line([l1,0],[0,0],'color','magenta');
    line([0,T1r(1,4)],[0,T1r(2,4)],'color','red');
    line([T1r(1,4),Tnr(1,4)],[T1r(2,4),Tnr(2,4)],'color','blue');
    line([X(1,i),Ox(1,i)],[Y(1,i),Oy(1,i)],'color','magenta');
    line([l1,0],[0,0],'color','magenta');
    hold off
    pause(0.05);
    if i == sz(1,2)
        break
    end
    clf;
end

%Ángulos para motores en grados
ang_rad = [transpose(q1r) (-1)*transpose(q2r) transpose(q3r)];
ang_deg = rad2deg(ang_rad);
ang_deg(:,1) = ang_deg(:,1)-90;
%A = abs(round(ang_deg))
rads = deg2rad(ang_deg);

rads2 = [rads (-1)*rads]

%dlmwrite('Pararse.txt',rads2,'delimiter',' ');

function Ai = calc_A(ai,alphai,di,thi)
Rz=[cos(thi) -sin(thi) 0 0;
    sin(thi) cos(thi) 0 0;
    0 0 1 0;
    0 0 0 1];

Tz=[1 0 0 0;
    0 1 0 0;
    0 0 1 di;
    0 0 0 1];

Tx=[1 0 0 ai;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];

Rx=[1 0 0 0;
    0 cos(alphai) -sin(alphai) 0;
    0 sin(alphai) cos(alphai) 0;
    0 0 0 1];

Ai = Rz*Tz*Tx*Rx;

end