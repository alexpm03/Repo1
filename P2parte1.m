%% INICIALIZACIÓN DE ROS (COMPLETAR ESPACIOS CON LAS DIRECCIONES IP)
setenv('ROS_MASTER_URI','http://172.29.30.178:11311');  % Reemplazar con la IP del maestro ROS
setenv('ROS_IP','172.29.29.51');                        % Reemplazar con la IP local
rosshutdown;
rosinit(); % Inicialización de ROS en la IP correspondiente

%% Ganancias del controlador P
Kp_linear = 0.5;      % Ganancia para el control de velocidad lineal
Kp_angular = 5;     % Ganancia para el control de velocidad angular
Kpi_angular = 0.1;

%% DECLARACIÓN DE SUBSCRIBERS
%% PUBLICADORES
% Publicador para enviar velocidades
pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist');
msg_vel = rosmessage(pub);

% Publicador para habilitar los motores
pub_enable = rospublisher('/cmd_motor_state','std_msgs/Int32');
msg_enable_motor = rosmessage(pub_enable);

%% HABILITAR LOS MOTORES
msg_enable_motor.Data = 1;  % 1 = activar motores
send(pub_enable, msg_enable_motor);


%% Definimos la periodicidad del bucle (10 Hz)
r = robotics.Rate(10);
waitfor(r);
pause(3);

%% Nos aseguramos de recibir un mensaje relacionado con el robot
while (strcmp(odom.LatestMessage.ChildFrameId, 'base_link') ~= 1)
    odom.LatestMessage;
end


%% Umbrales para condiciones de parada del robot
umbral_distancia = 0.5;  % Error de distancia en metros
umbral_angulo = 0.5;     % Error de orientación en radianes
error_integral = 0.1;

%% DECLARACIÓN DE VARIABLES NECESARIAS PARA EL CONTROL
goal = input('Ingrese la referencia de posición [x, y]: ');  
dt = 0.1;

%% Inicialización de vectores para almacenar datos
tiempo = [];
error_distancia = [];
error_orientacion = [];
error_integral_vec = [];
vel_linear_vec = [];
vel_angular_vec = [];
pos_x = [];
pos_y = [];

t_inicio = tic; % Iniciar tiempo

%% Bucle de control infinito
while (1)
    t_actual = toc(t_inicio); % Obtener tiempo actual
    
    %% Obtenemos la posición y orientación actuales
    pos = odom.LatestMessage.Pose.Pose.Position;
    ori = odom.LatestMessage.Pose.Pose.Orientation;
    yaw = quat2eul([ori.W, ori.X, ori.Y, ori.Z]);
    yaw = yaw(1);
    
    %% Calculamos el error de distancia
    Edist = sqrt((goal(1) - pos.X)^2 + (goal(2) - pos.Y)^2);
    
    %% Calculamos el error de orientación
    desired_yaw = atan2(goal(2) - pos.Y, goal(1) - pos.X);
    Eori = desired_yaw - yaw;
    Eori = atan2(sin(Eori), cos(Eori));
    
    %% Error integral
    error_integral = error_integral + Eori * dt;
    
    %% Calculamos las consignas de velocidades
    consigna_vel_linear = Kp_linear * Edist;
    consigna_vel_ang = Kp_angular * Eori + Kpi_angular * error_integral;
    
    %% Limitar la velocidad angular máxima
    MAX_VELOCIDAD_ANGULAR = 1.5;
    consigna_vel_ang = max(-MAX_VELOCIDAD_ANGULAR, min(MAX_VELOCIDAD_ANGULAR, consigna_vel_ang));
    
    %% Almacenar datos en vectores
    tiempo = [tiempo, t_actual];
    error_distancia = [error_distancia, Edist];
    error_orientacion = [error_orientacion, Eori];
    error_integral_vec = [error_integral_vec, error_integral];
    vel_linear_vec = [vel_linear_vec, consigna_vel_linear];
    vel_angular_vec = [vel_angular_vec, consigna_vel_ang];
    pos_x = [pos_x, pos.X];
    pos_y = [pos_y, pos.Y];
    
    %% Condición de parada
    if (Edist < umbral_distancia) && (abs(Eori) < umbral_angulo)
        msg_vel.Linear.X = 0;
        msg_vel.Angular.Z = 0;
        disp(pos);
        send(pub, msg_vel);
        break;
    end
    
    %% Aplicamos consignas de control
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Angular.Z = consigna_vel_ang;
    send(pub, msg_vel);
    waitfor(r);
end

rosshutdown;

%% Graficar los resultados
figure;
subplot(2,2,1);
plot(tiempo, error_distancia, 'r');
xlabel('Tiempo (s)'); ylabel('Error de distancia (m)');
title('Evolución del error de distancia'); grid on;

subplot(2,2,2);
plot(tiempo, error_orientacion, 'b');
xlabel('Tiempo (s)'); ylabel('Error de orientación (rad)');
title('Evolución del error de orientación'); grid on;

subplot(2,2,3);
plot(tiempo, error_integral_vec, 'g');
xlabel('Tiempo (s)'); ylabel('Error integral (rad·s)');
title('Evolución del error integral'); grid on;

subplot(2,2,4);
plot(tiempo, vel_linear_vec, 'm', tiempo, vel_angular_vec, 'c');
xlabel('Tiempo (s)'); ylabel('Velocidad');
title('Velocidades Lineal y Angular'); grid on;
legend('Velocidad Lineal', 'Velocidad Angular');

%% Graficar la trayectoria del robot
figure;
plot(pos_x, pos_y, 'k', 'LineWidth', 2);
hold on;
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Posición X (m)'); ylabel('Posición Y (m)');
title('Trayectoria del robot en 2D'); grid on;
legend('Recorrido', 'Objetivo');
axis([0 20 0 20]); % Fijar el eje de la gráfica de 0 a 20 en X y Y