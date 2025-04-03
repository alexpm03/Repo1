%% Código para Simulador STDR

% Inicialización de ROS para el simulador
setenv('ROS_MASTER_URI','http://192.168.1.117:11311') % IP del entorno del robot (simulador)
setenv('ROS_IP','192.168.1.119') % IP donde se ejecuta Matlab
rosshutdown;
rosinit;

% Declaración de Subscribers y Publishers
odom = rossubscriber('/robot0/odom');           % Suscriptor de odometría (simulador)
pub  = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist');  % Publicador para cmd_vel (simulador)

% Esperar a recibir mensajes válidos
pause(2);
while (strcmp(odom.LatestMessage.ChildFrameId, 'robot0') ~= 1)
    odom.LatestMessage;
end
%% Estructuras de datos
vect_distancias = [];
vect_angular = [];
%% Secuencia de movimientos para el simulador STDR
disp('Iniciando secuencia en Simulador STDR...');
    %robotReal(pub,odom);
    salirLaberinto(pub,odom);
disp('Secuencia completada en Simulador STDR.');

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Funciones locales para el control de movimiento en Simulador STDR %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function recorrido = avanzar(distancia, pub, odom)
    % Función para avanzar una distancia determinada
    velocidad = 0.2;  % Velocidad lineal (m/s)
    initPos = odom.LatestMessage.Pose.Pose.Position;
    vect_distancias = [];
    msg = rosmessage(pub);
    msg.Linear.X = velocidad;
    msg.Angular.Z = 0;
    r = robotics.Rate(10); % Frecuencia de 10 Hz
    while true
        send(pub, msg);
        currentPos = odom.LatestMessage.Pose.Pose.Position;
        d = sqrt((currentPos.X - initPos.X)^2 + (currentPos.Y - initPos.Y)^2);
        % Evitamos que se metan valores nulos al vector
        if(d==0)
            else
            vect_distancias = [vect_distancias, d];
            end
        if d >= distancia
            break;
        end
        waitfor(r);
    end
    recorrido = sumarVector(vect_distancias);
    % Detener el robot
    msg.Linear.X = 0;
    send(pub, msg);
end

function giro = girar(angulo, pub, odom)
    % Función para girar un ángulo determinado y almacenar mediciones
    targetAngle = deg2rad(angulo); % Conversión a radianes
    velocidadAngular = 0.5 * sign(targetAngle); % Velocidad angular con signo
    tolerancia = deg2rad(2); % Margen de error de 2 grados
    vect_angular = [];  % Inicializar el vector para almacenar los ángulos

    % Esperar hasta que odom tenga datos válidos
    while isempty(odom.LatestMessage)
        pause(0.1);
    end
    
    % Obtener la orientación inicial
    initQuat = odom.LatestMessage.Pose.Pose.Orientation;
    initEul = quat2eul([initQuat.W, initQuat.X, initQuat.Y, initQuat.Z], 'ZYX');
    initYaw = initEul(1);  % Yaw es el primer elemento en 'ZYX'

    % Preparar el mensaje de velocidad
    msg = rosmessage(pub);
    msg.Linear.X = 0;
    msg.Angular.Z = velocidadAngular;

    % Control de tiempo de ejecución
    r = robotics.Rate(10); % 10 Hz

    while true
        send(pub, msg);
        
        % Obtener la orientación actual
        currentQuat = odom.LatestMessage.Pose.Pose.Orientation;
        currentEul = quat2eul([currentQuat.W, currentQuat.X, currentQuat.Y, currentQuat.Z], 'ZYX');
        currentYaw = currentEul(1);

        if(currentYaw==0)
        else
        vect_angular = [vect_angular, currentYaw];
        end

        % Calcular el cambio de ángulo usando la función personalizada
        deltaYaw = normalizarAngulo(currentYaw - initYaw);
        
        % Verificar si se alcanzó el ángulo deseado dentro de la tolerancia
        if abs(deltaYaw) >= abs(targetAngle) - tolerancia
            break;
        end
        waitfor(r);
    end
    giro = sumarVector(vect_angular);
    % Detener el giro
    msg.Angular.Z = 0;
    send(pub, msg);
end

% Función personalizada para normalizar el ángulo en [-pi, pi]
function angulo_normalizado = normalizarAngulo(angulo)
    angulo_normalizado = mod(angulo + pi, 2*pi) - pi;
end



function salirLaberinto(pub,odom)
    girar(180, pub, odom); 
    avanzar(2, pub, odom);   
    girar(-90, pub, odom); 
    avanzar(2, pub, odom);   
    girar(90, pub, odom);
    avanzar(4, pub, odom); 
    girar(90, pub, odom);
    avanzar(3, pub, odom);
end

function robotReal(pub, odom)
    Distancia = 0;
    giro = 0;
    Distancia = Distancia + avanzar(2, pub, odom);   
    giro = giro +girar(90, pub, odom); 
    Distancia = Distancia + avanzar(1, pub, odom);   
    giro = giro +girar(-90, pub, odom);
    Distancia = Distancia + avanzar(1, pub, odom); 
    fprintf('Distancia total recorrida: %.2f metros\n', Distancia);
    fprintf('Ángulo total girado: %.2f grados\n', rad2deg(giro)); % Convertimos a grados
    

end


function resultado = sumarVector(vector)
    % Verificar que el vector no esté vacío
    if isempty(vector)
        error('El vector está vacío.');
    end

    % Restar el último valor menos el primero
    resultado = vector(end) - vector(1);
end


