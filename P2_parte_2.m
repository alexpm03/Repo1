
%% INICIALIZACIÓN DE ROS

setenv('ROS_MASTER_URI','http://192.168.104.87:11311');  % IP del maestro ROS
setenv('ROS_IP','192.168.104.80');                  %  IP local
rosshutdown();  % Cerramos cualquier conexión previa
rosinit();      % Iniciamos la conexión con ROS

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
MAX_TIME = 1000;                     % Número máximo de iteraciones
medidas = zeros(5, MAX_TIME);        % Para almacenar distancias, errores, etc.
                                     %   fila 1: lectura actual del sonar (dist)
                                     %   fila 2: lectura anterior (lastdist)
                                     %   fila 3: distancia avanzada (distav)
                                     %   fila 4: error de orientación (Eori)
                                     %   fila 5: error lateral (Edist)
% Parámetros del controlador
D_des = 1;     % Distancia deseada a la pared (m)
offset = 0.105;%-0.105;  % Offset del sensor sonar_0 respecto al centro del robot (m)
K_d = 0.4; %0.5; %1.0;       % Ganancia para el error de distancia (d_e) lateral
K_o = 0.4; %0.5; %1.0;       % Ganancia para el error de orientación (o_e)
v_lin = 0.2; % Velocidad lineal
% Saturamos la velocidad angular para que no supere 0.5 rad/s (según la guía)
MAX_ANG_VEL = 0.5;

%% VARIABLES PARA ALMACENAR DATOS DE GRÁFICAS
tiempo = [];         % Vector de tiempo
vel_linear_vec = []; % Vector para velocidad lineal comandada
vel_angular_vec = [];% Vector para velocidad angular comandada

%% DECLARACIÓN DE SUBSCRIBERS
odom = rossubscriber('/robot0/odom');                % Subscripción a la odometría
sonar0 = rossubscriber('/robot0/sonar_0','sensor_msgs/Range');   % Subscripción al sensor de ultrasonidos

%% DECLARACIÓN DE PUBLISHERS
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);  % Mensaje del tipo "geometry_msgs/Twist"
msg_sonar0 = rosmessage(sonar0);

%% Definimos la periodicidad del bucle (10 Hz)
r = robotics.Rate(10);
waitfor(r);
pause(3);  % Esperamos ~3 s para asegurarnos de que llegan los primeros mensajes

%% Nos aseguramos de recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId, 'robot0') ~= 1)
    odom.LatestMessage;
end

%% Inicializamos variables para el control
i = 0;

pos = odom.LatestMessage.Pose.Pose.Position;   % Posición inicial
lastpos = pos;                                 % Guardamos como "última posición"
dist = msg_sonar0.Range_;        % Distancia medida por sonar_0
if dist > 5
   dist = 5;
end
lastdist = dist;                               % Guardamos como "última distancia"
lastdistav = 0;                                % Distancia avanzada previa (inicialmente 0)

msg_vel.Linear.X = v_lin;
send(pub, msg_vel);
pause(3);
%% Iniciamos el contador de tiempo
t_inicio = tic;

%% Bucle de control
while true
    i = i + 1;
    t_actual = toc(t_inicio);   % Tiempo actual en segundos
    %tiempo = [tiempo, t_actual];  % Almacenamos el tiempo
    tiempo(i) = t_actual;

    % 1) Obtenemos la posición y medida del sonar actuales
    pos = odom.LatestMessage.Pose.Pose.Position;
    msg_sonar0 = receive (sonar0); 
    dist = msg_sonar0.Range_;
    
    % 2) Calculamos la distancia avanzada (distav)
    distav = sqrt( (pos.X - lastpos.X)^2 + (pos.Y - lastpos.Y)^2 );
    
    % 3) Limitamos la lectura del sonar a 5 m (para descartar valores muy grandes)
    if dist > 5
        dist = 5;
    end
    disp(['distancia sonar 0: ', num2str(dist)]);
    disp(['Posición actual -> X: ', num2str(pos.X), ', Y: ', num2str(pos.Y)]);
    disp(['distancia avanzada: ', num2str(distav)]);

    % 4) Cálculo del error de orientación (o_e)
    %    Según la práctica: o_e = atan2( dist(t) - dist(t-1), distav )
    
    delta_dist = (dist - lastdist)/distav; %DEPENDIENDO DE SI LA PARED ESTÁ A LA DERECHA O IZQUIERDA PODRÍA SER AL REVÉS, Y -OFFSET 
    
    
    Eori = atan(delta_dist);
    
   disp(['error angular: ', num2str(Eori)]);

    disp(['Velocidad lineal: ', num2str(odom.LatestMessage.Twist.Twist.Linear.X), ' m/s']);

    % 5) Cálculo del error de distancia (d_e)
    %    Según la práctica: d_e = [ (dist + offset) * cos(o_e) ] - D_des
    Edist = ( (dist + offset) * cos(Eori) ) - D_des;
    
    % 6) Almacenamos en la matriz "medidas"
    %    medidas(1,i) = dist actual
    %    medidas(2,i) = lastdist (valor anterior)
    %    medidas(3,i) = distav (distancia avanzada)
    %    medidas(4,i) = Eori (error de orientación)
    %    medidas(5,i) = Edist (error lateral)
    medidas(1,i) = dist;
    medidas(2,i) = lastdist;
    medidas(3,i) = distav;
    medidas(4,i) = Eori;
    medidas(5,i) = Edist;
    
    % 7) Calculamos las consignas de velocidades
    
   
    
    if abs(Eori) < 0.05  % Puedes ajustar 0.05 rad (~3°) según necesites
        consigna_vel_ang = 0; % No girar más
    else
        consigna_vel_ang = K_d * Edist + K_o * Eori;
        consigna_vel_ang = max(-MAX_ANG_VEL, min(MAX_ANG_VEL, consigna_vel_ang));
    end
           
    %ACTUALIZAR SOLO LA VELOCIDAD ANGULAR
    msg_vel.Linear.X = v_lin;
    msg_vel.Angular.Z = consigna_vel_ang;

    %ENVIAMOS LA CONSIGNA (LA VELOCIDAD ANGULAR PERMANECE CONSTANTE)
    send(pub, msg_vel);

    % Almacenamos las velocidades comandadas para graficar
    vel_linear_vec(i) = v_lin;
    vel_angular_vec(i) = consigna_vel_ang;
    %vel_linear_vec = [vel_linear_vec, consigna_vel_linear];
    %vel_angular_vec = [vel_angular_vec, consigna_vel_ang];

    %ACTUALIZAMOS VARIABLES 
    lastpos = pos;
    lastdist = dist;
    lastdistav = distav;

    % 8) Condición de parada:
    waitfor(r);
    %    Se para si el robot está muy cerca de la distancia deseada
    %    y casi paralelo a la pared
    if (abs(Edist) < 0.05) && (abs(Eori) < 0.05)
        % Detenemos el robot
        msg_vel.Linear.X = 0;
        msg_vel.Angular.Z = 0;
        send(pub, msg_vel);
        break;
    end
    if i == MAX_TIME
        break;
    end
end

%% Guardamos la matriz de medidas
save('medidas.mat', 'medidas');

%% DESCONEXIÓN DE ROS
rosshutdown;

%% Apartado de GRÁFICAS
% Se trazarán:
%   - Evolución del error de orientación (Eori)
%   - Evolución del error lateral (Edist)
%   - Velocidad lineal y angular comandadas
figure;

subplot(2,2,1);
plot(tiempo, medidas(4,1:i), 'b','LineWidth',1.5);
xlabel('Tiempo (s)');
ylabel('Error de orientación (rad)');
title('Evolución del error de orientación');
grid on;

subplot(2,2,2);
plot(tiempo, medidas(5,1:i), 'r','LineWidth',1.5);
xlabel('Tiempo (s)');
ylabel('Error lateral (m)');
title('Evolución del error lateral');
grid on;

subplot(2,2,3);
plot(tiempo, vel_linear_vec, 'm','LineWidth',1.5);
xlabel('Tiempo (s)');
ylabel('Velocidad lineal (m/s)');
title('Velocidad lineal comandada');
grid on;

subplot(2,2,4);
plot(tiempo, vel_angular_vec, 'c','LineWidth',1.5);
xlabel('Tiempo (s)');
ylabel('Velocidad angular (rad/s)');
title('Velocidad angular comandada');
grid on;
