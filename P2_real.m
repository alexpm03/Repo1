%% LIMPIEZA E INICIALIZACIÓN DE ROS
rosshutdown();  % Cierra cualquier sesión anterior de ROS

% Establecer las IP adecuadas para tu robot y tu PC
setenv('ROS_MASTER_URI','http://172.29.30.175:11311');  % IP del robot real
setenv('ROS_IP','172.29.29.51');                      % IP de tu PC (IP local)
rosinit();                                            % Iniciar conexión con ROS

%% GANANCIAS DEL CONTROLADOR
Kp_linear = 0.5;       % Ganancia para el control de velocidad lineal
Kp_angular = 5;      % Ganancia para el control de velocidad angular
Kpi_angular = 0.1;     % Ganancia integral para el control angular

%% SUSCRIPTOR A LA ODOMETRÍA (ROBOT REAL)
odom = rossubscriber('/pose');  % Para el robot real, el topic de odometría es /pose

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

%% PAUSA INICIAL Y CONTROL DE FRECUENCIA
r = robotics.Rate(10); 
waitfor(r);
pause(5);  % Esperamos unos segundos para asegurarnos de que lleguen datos del odómetro

%% ESPERAR A RECIBIR UN MENSAJE VÁLIDO (FrameId = 'base_link')
while (strcmp(odom.LatestMessage.ChildFrameId, 'base_link') ~= 1)
    odom.LatestMessage;
end

%% UMBRALES PARA CONDICIÓN DE PARADA
umbral_distancia = 0.5;  % Error de distancia en metros
umbral_angulo = 0.5;     % Error de orientación en radianes

%% INICIALIZACIÓN DE VARIABLES DE CONTROL
goal = input('Ingrese la referencia de posición [x, y]: ');
dt = 0.1;
error_integral = 0.0;   % Iniciamos la integral en cero; 0.1

%% VECTORES PARA ALMACENAR DATOS (OPCIONAL)
tiempo = [];
error_distancia = [];
error_orientacion = [];
error_integral_vec = [];
vel_linear_vec = [];
vel_angular_vec = [];
pos_x = [];
pos_y = [];

t_inicio = tic;  % Iniciar conteo de tiempo

%% BUCLE DE CONTROL
while true
    t_actual = toc(t_inicio);  % Tiempo actual en segundos
    
    % 1. OBTENER POSICIÓN Y ORIENTACIÓN
    pos = odom.LatestMessage.Pose.Pose.Position;
    ori = odom.LatestMessage.Pose.Pose.Orientation;
    
    % Convertir el cuaternión a ángulos (yaw, pitch, roll)
    % Asegurarse de que quat2eul interprete el orden correcto: [W, X, Y, Z]
    yaw = quat2eul([ori.W, ori.X, ori.Y, ori.Z]);
    yaw = yaw(1);  % El primer valor es yaw
    
    % 2. CÁLCULO DE ERRORES
    % Error de distancia (euclidiana)
    Edist = sqrt((goal(1) - pos.X)^2 + (goal(2) - pos.Y)^2);
    
    % Error de orientación
    desired_yaw = atan2(goal(2) - pos.Y, goal(1) - pos.X);
    Eori = desired_yaw - yaw;
    % Normalizar el error entre -pi y pi
    Eori = atan2(sin(Eori), cos(Eori));
    
    % 3. ACTUALIZACIÓN DEL ERROR INTEGRAL
    error_integral = error_integral + Eori * dt;
    
    % 4. LEYES DE CONTROL (P + I en el lazo angular)
    consigna_vel_linear = Kp_linear * Edist;
    consigna_vel_ang = Kp_angular * Eori + Kpi_angular * error_integral;
    
    % Limitar velocidad angular máxima
    MAX_VELOCIDAD_ANGULAR = 1.5;  % rad/s 
    consigna_vel_ang = max(-MAX_VELOCIDAD_ANGULAR, min(MAX_VELOCIDAD_ANGULAR, consigna_vel_ang));
    
    % 5. ALMACENAR DATOS PARA GRÁFICAS (OPCIONAL)
    tiempo              = [tiempo, t_actual];
    error_distancia     = [error_distancia, Edist];
    error_orientacion   = [error_orientacion, Eori];
    error_integral_vec  = [error_integral_vec, error_integral];
    vel_linear_vec      = [vel_linear_vec, consigna_vel_linear];
    vel_angular_vec     = [vel_angular_vec, consigna_vel_ang];
    pos_x = [pos_x, pos.X];
    pos_y = [pos_y, pos.Y];
    % 6. CONDICIÓN DE PARADA
    if (Edist < umbral_distancia) && (abs(Eori) < umbral_angulo)
        % Detener el robot
        msg_vel.Linear.X = 0;
        msg_vel.Angular.Z = 0;
        disp(pos); 
        send(pub, msg_vel);
        send(pub_enable, msg_enable_motor);
        break;
    end
    
    % 7. ENVIAR CONSIGNAS DE CONTROL
    msg_vel.Linear.X = consigna_vel_linear;
    msg_vel.Linear.Y = 0;
    msg_vel.Linear.Z = 0;
    msg_vel.Angular.X = 0;
    msg_vel.Angular.Y = 0;
    msg_vel.Angular.Z = consigna_vel_ang;
    send(pub, msg_vel);
    
    % 8. RESPETAR LA FRECUENCIA
    waitfor(r);
end

%% FINALIZAR SESIÓN ROS
rosshutdown();

%% GRÁFICAS DE RESULTADOS 
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
legend('Velocidad Lineal', 'Velocidad Angular');

%% Graficar la trayectoria del robot
figure;
plot(pos_x, pos_y, 'k', 'LineWidth', 2);
hold on;
plot(goal(1), goal(2), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
xlabel('Posición X (m)'); ylabel('Posición Y (m)');
title('Trayectoria del robot en 2D'); grid on;
legend('Recorrido', 'Objetivo');
axis([0 20 0 20]); % Fijar el eje de la gráfica de 0 a 20 en X y Y
