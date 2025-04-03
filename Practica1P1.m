%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP

%rosinit('http://192.168.103.181:11311', 'NodeHost', ' 192.168.103.227')
%setenv('ROS_MASTER_URI','http://172.29.30.48:11311') % IP del entorno del robot
setenv('ROS_MASTER_URI','http://192.168.100.116:11311')
setenv('ROS_IP','192.168.100.7')

rosshutdown; % Desconectamos por si hay algún ROS ejecutándose
rosinit % Inicialización de ROS
%% DECLARACIÓN DE SUBSCRIBERS 
odom=rossubscriber('/robot0/odom'); % Subscripción a la odometría 
sonar0 = rossubscriber('/robot0/sonar_0','sensor_msgs/Range');
sonar1 = rossubscriber('/robot0/sonar_1','sensor_msgs/Range');
sonar2 = rossubscriber('/robot0/sonar_2','sensor_msgs/Range');
sonar3 = rossubscriber('/robot0/sonar_3','sensor_msgs/Range');
sonar4 = rossubscriber('/robot0/sonar_4','sensor_msgs/Range');
sonar5 = rossubscriber('/robot0/sonar_5','sensor_msgs/Range');
sonar6 = rossubscriber('/robot0/sonar_6','sensor_msgs/Range');
sonar7 = rossubscriber('/robot0/sonar_7','sensor_msgs/Range');
pause (2) 


%% DECLARACIÓN DE PUBLISHERS 
pub = rospublisher('/robot0/cmd_vel', 'geometry_msgs/Twist'); 
%% Inicializamos la primera posición (coordenadas x,y,z)
initpos=odom.LatestMessage.Pose.Pose.Position;

%% GENERACIÓN DE MENSAJE 
msg=rosmessage(pub); %% Creamos un mensaje del tipo declarado en "pub" (geometry_msgs/Twist) 
% Rellenamos los campos del mensaje para que el robot avance a 0.2 m/s 
% Velocidades lineales en x,y y z (velocidades en y o z no se usan en robots diferenciales y entornos 2D) 
msg.Linear.X=0; 
msg.Linear.Y=0; 
msg.Linear.Z=0; 
% Velocidades angulares (en robots diferenciales y entornos 2D solo se utilizará el valor Z) 
msg.Angular.X=0; 
msg.Angular.Y=0; 
msg.Angular.Z=0; 
%% Definimos la perodicidad del bucle (10 hz) 
r = robotics.Rate(10); 
%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0" 
pause(1);
while (strcmp(odom.LatestMessage.ChildFrameId,'robot0')~=1) 
odom.LatestMessage 
end 
%% Estructuras de datos
vect_distancias = [];
vect_angular = [];
paredes = 0;
umbral = 1;
%% Bucle de control infinito 
while (1) 
send(pub,msg); 
% Temporización del bucle según el parámetro establecido en r 
%% Obtenemos la posición actual 
pos=odom.LatestMessage.Pose.Pose.Position; 
%% Obtenemos la orientación actual q_angular
orientationQuat = [odom.LatestMessage.Pose.Pose.Orientation.W, ...
                   odom.LatestMessage.Pose.Pose.Orientation.X, ...
                   odom.LatestMessage.Pose.Pose.Orientation.Y, ...
                   odom.LatestMessage.Pose.Pose.Orientation.Z];

orientationRadianes = quat2eul(orientationQuat);


%% Calculamos la distancia euclÃdea que se ha desplazado q_lineal
dist = sqrt((initpos.X-pos.X)^2+(initpos.Y-pos.Y)^2);
% Evitamos que se metan valores nulos al vector
if(dist==0)
else
vect_distancias = [vect_distancias, dist];
end

% Evitamos que se metan valores nulos al vector
if(orientationRadianes(1)==0)
else
vect_angular = [vect_angular, orientationRadianes(1)];
end

% Calculamos las respectivas resoluciones máximas (q_lineal y q_angular)
minimoDist = min(diff((unique(vect_distancias))));
minimoAng = min(diff((unique(vect_angular))));

plot(diff((unique(vect_distancias))));
plot(diff((unique(vect_angular))));

%% Si el robot se ha desplazado más de un metro detenemos el robot (velocidad lineal 0) y salimos del bucle
if (dist>4)
msg.Linear.X = 0;
msg.Angular.Z = 0;
fprintf('Detencion por distancia maxima');
% Comando de velocidad
send(pub,msg);
% Salimos del bucle de control
break;
else
% Comando de velocidad
send(pub,msg);
end

%% calcular posicion actual del robot
pos_sensor = odom.LatestMessage.Pose.Pose.Position;
disp(['Posición del robot: X=', num2str(pos_sensor.X), ', Y=', num2str(pos_sensor.Y)]);
%% calcular distancia del sonar 1
distSonar_2 = sonar2.LatestMessage.Range_;
%% Si la distancia de sonar 2 es igual a 2 metros el robot se para
disp(['Distancia medida por el sonar 2: ', num2str(distSonar_2)]);
if (distSonar_2 >= 1.8 && distSonar_2 <= 2.2)
msg.Linear.X = 0;
msg.Angular.Z = 0;
fprintf('Detencion por distancia de sonar');
% Comando de velocidad
send(pub,msg);
else
% Comando de velocidad
send(pub,msg);
end

%% Obtener medidas de todods los sensores sonar
sonar0Dist = sonar0.LatestMessage.Range_;
disp(['Distancia medida por el sonar 0: ', num2str(sonar0Dist)]);
sonar1Dist = sonar1.LatestMessage.Range_;
disp(['Distancia medida por el sonar 1: ', num2str(sonar1Dist)]);
sonar2Dist = sonar2.LatestMessage.Range_;
disp(['Distancia medida por el sonar 2: ', num2str(sonar2Dist)]);
sonar3Dist = sonar3.LatestMessage.Range_;
disp(['Distancia medida por el sonar 3: ', num2str(sonar3Dist)]);
sonar4Dist = sonar4.LatestMessage.Range_;
disp(['Distancia medida por el sonar 4: ', num2str(sonar4Dist)]);
sonar5Dist = sonar5.LatestMessage.Range_;
disp(['Distancia medida por el sonar 5: ', num2str(sonar5Dist)]);
sonar6Dist = sonar6.LatestMessage.Range_;
disp(['Distancia medida por el sonar 6: ', num2str(sonar6Dist)]);
sonar7Dist = sonar7.LatestMessage.Range_;
disp(['Distancia medida por el sonar 7: ', num2str(sonar7Dist)]);
%% comrpobar distancia trasera
d_trasera = calcular_distancia_trasera(sonar6Dist, sonar7Dist);
disp(['La distancia real trasera es: ', num2str(d_trasera)]);

%% comprobar paredes
numParedes = n_paredes(sonar0Dist, sonar2Dist, sonar5Dist, d_trasera, umbral);

%% confianza paredes
theta = orientationRadianes(1); % Yaw del robot en radianes
ideal = 1;   % distancia ideal a la pared (en metros)
maxDist = 2; % umbral máximo (en metros)

funcion_calidad(sonar0Dist, sonar1Dist, sonar2Dist, sonar3Dist, sonar4Dist, sonar5Dist, sonar7Dist, sonar6Dist, ideal, maxDist);

%% RECOGER 1,000 MEDICIONES

num_mediciones = 1000; % Número total de muestras
distanciasSonar2 = zeros(1, num_mediciones); % Vector para almacenar las mediciones
distanciasFiltradas = zeros(1, num_mediciones); % Vector para la media móvil
N = 5; % Tamaño de la ventana para la media móvil

disp('Iniciando captura de datos del sonar...');

for i = 1:num_mediciones
    % Obtener la medición actual del sonar
    msg = sonar2.LatestMessage;
    
    if isempty(msg) % Verificar si el mensaje está vacío
        distanciasSonar2(i) = NaN; % Guardar NaN si no hay medición válida
    else
        distanciasSonar2(i) = msg.Range_; % Guardar la distancia medida
    end
    
    % Aplicar filtro de media móvil cuando haya suficientes datos
    if i >= N
        distanciasFiltradas(i) = mean(distanciasSonar2(i-N+1:i), 'omitnan');
    else
        distanciasFiltradas(i) = distanciasSonar2(i); % Usar valores originales al inicio
    end
    
    pause(0.01); % Simular frecuencia de muestreo (100 Hz)
end

disp('Captura de datos finalizada.');

%%  PRIMERA GRÁFICA: TODAS LAS MEDICIONES
figure;
plot(distanciasSonar2, 'r', 'DisplayName', 'Medición Original'); % Línea roja (sin filtrar)
hold on;
plot(distanciasFiltradas, 'b', 'LineWidth', 2, 'DisplayName', 'Medición Filtrada'); % Línea azul (filtrada)
xlabel('Número de medición');
ylabel('Distancia medida (m)');
title('Mediciones completas del sensor sonar 2');
legend;
grid on;
hold off;

%% SEGUNDA GRÁFICA: ÚLTIMAS 5 MEDICIONES
figure;
ultimas_5_medidas = num_mediciones - N + 1 : num_mediciones; % Índices de las últimas 5 medidas
plot(ultimas_5_medidas, distanciasSonar2(ultimas_5_medidas), 'ro-', 'DisplayName', 'Medición Original');
hold on;
plot(ultimas_5_medidas, distanciasFiltradas(ultimas_5_medidas), 'bo-', 'LineWidth', 2, 'DisplayName', 'Medición Filtrada');
xlabel('Número de medición');
ylabel('Distancia medida (m)');
title('Últimas 5 mediciones del sensor sonar 2');
legend;
grid on;
hold off;


%% ANÁLISIS ESTADÍSTICO DEL RUIDO
maximo = max(distanciasSonar2);
media = mean(distanciasSonar2);
varianza = var(distanciasSonar2);

%% MOSTRAR RESULTADOS
disp(['Máximo: ', num2str(maximo), ' m']);
disp(['Media: ', num2str(media), ' m']);
disp(['Varianza: ', num2str(varianza), ' m²']);
%}


%% final bucle

waitfor(r) 
end 

%% funciones 
function d_trasera = calcular_distancia_trasera(d1, d2)
    % d1: distancia medida por el sensor izquierdo
    % d2: distancia medida por el sensor derecho
    
    % Ángulo entre los sensores (en radianes)
    angulo = 45; % 45 grados
    angulo_rad = deg2rad(angulo);
    
    % Aplicamos la ley de los cosenos para calcular la distancia real
    d_trasera = sqrt(d1^2 + d2^2 - 2 * d1 * d2 * cos(angulo_rad));
end

function paredes = n_paredes(dist0, dist1, dist2, dist3, dist_minima)

    if (((dist0 < dist_minima) && (dist1 > dist_minima) && (dist2 > dist_minima) && (dist3 > dist_minima)) || ...
        ((dist0 > dist_minima) && (dist1 < dist_minima) && (dist2 > dist_minima) && (dist3 > dist_minima)) || ...
        ((dist0 > dist_minima) && (dist1 > dist_minima) && (dist2 < dist_minima) && (dist3 > dist_minima)) || ...
        ((dist0 > dist_minima) && (dist1 > dist_minima) && (dist2 > dist_minima) && (dist3 < dist_minima)))
        paredes = 1;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    
    elseif (((dist0 < dist_minima) && (dist1 < dist_minima) && (dist2 > dist_minima) && (dist3 > dist_minima)) || ...
            ((dist0 < dist_minima) && (dist1 > dist_minima) && (dist2 < dist_minima) && (dist3 > dist_minima)) || ...
            ((dist0 < dist_minima) && (dist1 > dist_minima) && (dist2 > dist_minima) && (dist3 < dist_minima)) || ...
            ((dist0 > dist_minima) && (dist1 < dist_minima) && (dist2 < dist_minima) && (dist3 > dist_minima)) || ...
            ((dist0 > dist_minima) && (dist1 < dist_minima) && (dist2 > dist_minima) && (dist3 < dist_minima)) || ...
            ((dist0 > dist_minima) && (dist1 > dist_minima) && (dist2 < dist_minima) && (dist3 < dist_minima))) 
        paredes = 2;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    elseif (((dist0 < dist_minima) && (dist1 < dist_minima) && (dist2 < dist_minima) && (dist3 > dist_minima)) || ...
            ((dist0 < dist_minima) && (dist1 < dist_minima) && (dist2 > dist_minima) && (dist3 < dist_minima)) || ...
            ((dist0 > dist_minima) && (dist1 < dist_minima) && (dist2 < dist_minima) && (dist3 < dist_minima)) || ...
            ((dist0 < dist_minima) && (dist1 > dist_minima) && (dist2 < dist_minima) && (dist3 < dist_minima)))
        paredes = 3;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);

    elseif ((dist0 < dist_minima) && (dist1 < dist_minima) && (dist2 < dist_minima) && (dist3 < dist_minima))
        paredes = 4;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    else
        paredes = 0;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    end
    
end

function funcion_calidad(izq, frente1, frente2, frente3, frente4, drch, detras1, detras2, ideal, maxDist)
    
    compute_conf = @(d) (d <= ideal).* 1 + (d > ideal & d < maxDist).* (1 - (d - ideal)/(maxDist - ideal));
    
    % Confianza para sensores individuales (izquierda y derecha)
    confianza_izq  = compute_conf(izq);
    confianza_drch = compute_conf(drch);
    
    % Para los sensores frontales, se pueden combinar los datos
    datos_delanteros = [frente1, frente2, frente3, frente4];
    % Tomamos solo las mediciones menores a la distancia máxima
    datos_validos = datos_delanteros(datos_delanteros < maxDist);
    if ~isempty(datos_validos)
        confianza_delantera = mean(compute_conf(datos_validos));
    else
        confianza_delantera = 0;
    end
    
    % Para los sensores traseros
    datos_traseros = [detras1, detras2];
    datos_validos_traseros = datos_traseros(datos_traseros < maxDist);
    if ~isempty(datos_validos_traseros)
        confianza_trasera = mean(compute_conf(datos_validos_traseros));
    else
        confianza_trasera = 0;
    end
    
    % Mostrar resultados
    fprintf('Confianza pared izquierda: %.2f\n', confianza_izq);
    fprintf('Confianza pared derecha:   %.2f\n', confianza_drch);
    fprintf('Confianza pared delantera: %.2f\n', confianza_delantera);
    fprintf('Confianza pared trasera:   %.2f\n', confianza_trasera);
end









