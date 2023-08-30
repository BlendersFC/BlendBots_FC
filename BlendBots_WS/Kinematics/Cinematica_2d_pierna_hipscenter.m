%Authors: Pedro Deniz
%         Marlene Cobian

clear all
clc

%Dimensiones de robot (cm)
l1 = 6.475;         %Planta de pies
l2 = 12.5325;       %Tobillo-rodilla
l3 = 12.5325;       %Rodilla-cadera
offset = 7;         %Cadera-torso

%Número de pasos para interpolaciones
no_interp = 10;

%Número de pasos para la simulación
no_steps = 3;

%Puntos a seguir por paso
init_pose = [0 -(l2+l3) + offset + 4];

foot_up = [0 -(l2+l3) + offset + 7];

foot_front = [3 -(l2+l3) + offset + 4];

foot_back = [-3 -(l2+l3) + offset + 4];

%Interpolaciones pierna derecha
Oyr0 = (init_pose(2))*ones(1, no_interp);
Oyr1 = linspace(init_pose(2), foot_up(2), no_interp);
Oyr = [Oyr0 Oyr1];

Oxr0 = (init_pose(1))*ones(1, no_interp);
Oxr1 = linspace(init_pose(1), foot_up(1), no_interp);
Oxr = [Oxr0 Oxr1];

t = -360:40:360;

for i = 1:no_steps
    Oxr2 = linspace(foot_up(1), foot_front(1), no_interp);
    Oxr3 = linspace(foot_front(1), init_pose(1), no_interp);
    Oxr4 = linspace(init_pose(1), foot_back(1), no_interp);
    Oxr5 = linspace(foot_back(1), foot_up(1), no_interp);

    Oyr2 = linspace(foot_up(2), foot_front(2), no_interp);
    Oyr3 = linspace(foot_front(2), init_pose(2), no_interp);
    Oyr4 = linspace(init_pose(2), foot_back(2), no_interp);
    Oyr5 = linspace(foot_back(2), foot_up(2), no_interp);
    
    Oxr = [Oxr Oxr2 Oxr3 Oxr4 Oxr5];
    Oyr = [Oyr Oyr2 Oyr3 Oyr4 Oyr5];
end

%Interpolaciones pierna izquierda
Oyl0 = (init_pose(2))*ones(1, no_interp*2);
Oyl = [Oyl0];

Oxl0 = (init_pose(1))*ones(1, no_interp*2);
Oxl = [Oxl0];

for i = 1:no_steps
    Oxl1 = linspace(init_pose(1), foot_back(1), no_interp);
    Oxl2 = linspace(foot_back(1), foot_up(1), no_interp);
    Oxl3 = linspace(foot_up(1), foot_front(1), no_interp);
    Oxl4 = linspace(foot_front(1), init_pose(1), no_interp);

    Oyl1 = linspace(init_pose(2), foot_back(2), no_interp);
    Oyl2 = linspace(foot_back(2), foot_up(2), no_interp);
    Oyl3 = linspace(foot_up(2), foot_front(2), no_interp);
    Oyl4 = linspace(foot_front(2), init_pose(2), no_interp);
    
    Oxl = [Oxl Oxl1 Oxl2 Oxl3 Oxl4];
    Oyl = [Oyl Oyl1 Oyl2 Oyl3 Oyl4];
end

sz = size(Oxr);

%Para graficación
Xr = Oxr;
Yr = Oyr - offset;
Xl = Oxl;
Yl = Oyl - offset;

%Orientación
lado = -1;
q1r = [];
q2r = [];
q3r = [];
q1l = [];
q2l = [];
q3l = [];

figure;
for i = 1:sz(1,2)
    %Cálculo de cinemática inversa
    Qr = calc_cinem_inv_3eslabones(Xr(1,i),Yr(1,i),l2,l3,offset,lado);
    q1r = [q1r Qr(1,1)];
    q2r = [q2r Qr(1,2)];
    q3r = [q3r Qr(1,3)];

    Ql = calc_cinem_inv_3eslabones(Xl(1,i),Yl(1,i),l2,l3,offset,lado);
    q1l = [q1l Ql(1,1)];
    q2l = [q2l Ql(1,2)];
    q3l = [q3l Ql(1,3)];
    
    %Tablas Denavith-Hartenberg
    a1r=l2; alpha1r=0; d1r=0; th1r = 0 + q1r(1,i);
    a2r=l3; alpha2r=0; d2r=0; th2r = 0 + q2r(1,i);

    a1l=l2; alpha1l=0; d1l=0; th1l = 0 + q1l(1,i);
    a2l=l3; alpha2l=0; d2l=0; th2l = 0 + q2l(1,i);
    
    %Transformaciones
    T1r=calc_A(a1r,alpha1r,d1r,th1r);
    T2r=calc_A(a2r,alpha2r,d2r,th2r);

    T1l=calc_A(a1l,alpha1l,d1l,th1l);
    T2l=calc_A(a2l,alpha2l,d2l,th2l);
    
    %Cinemática directa (multiplicación de matrices)
    Tnr=T1r*T2r;

    Tnl=T1l*T2l;

    %Puntos finales de las piernas
    Xrobotr = Tnr(1,4);
    Yrobotr = Tnr(2,4);

    Xrobotl = Tnl(1,4);
    Yrobotl = Tnl(2,4);
    
    %Graficar
    hold on

    plot(0,offset,'sg',Xrobotl,Yrobotl,'ob',T1l(1,4),T1l(2,4),'or',T1l(1,4),T1l(2,4),'ob');
    plot(0,offset,'sg',Xrobotr,Yrobotr,'or',T1r(1,4),T1r(2,4),'ob',T1r(1,4),T1r(2,4),'or');

    xlim([-17.5,17.5]);
    ylim([-25.5,10.5]);
    
    grid on
    
    line([0,T1l(1,4)],[0,T1l(2,4)],'color','blue');
    line([T1l(1,4),Tnl(1,4)],[T1l(2,4),Tnl(2,4)],'color','blue');
    line([0,0],[0,offset],'color','magenta');
    line([Tnl(1,4)-(l1*lado),Tnl(1,4)],[Tnl(2,4),Tnl(2,4)],'color','blue');

    line([0,T1r(1,4)],[0,T1r(2,4)],'color','red');
    line([T1r(1,4),Tnr(1,4)],[T1r(2,4),Tnr(2,4)],'color','red');
    line([0,0],[0,offset],'color','magenta');
    line([Tnr(1,4)-(l1*lado),Tnr(1,4)],[Tnr(2,4),Tnr(2,4)],'color','red');
    
    hold off
    
    %disp(Oy(i))
    pause(0.05);
    
    if i == sz(1,2)
        break
    end
    
    clf;

end

%Ángulos para motores en grados
ang_rad = [(-1)*transpose(q1r) transpose(q2r) transpose(q3r)+pi];
ang_deg = rad2deg(ang_rad);
ang_deg(:,1) = ang_deg(:,1)-90;
%A = abs(round(ang_deg))
rads = deg2rad(ang_deg)

%Radianes en orden: tobillo, rodilla, cadera, cadera, rodilla, tobillo
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