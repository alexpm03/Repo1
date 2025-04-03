%% INICIALIZACIÓN DE ROS
% Se definen las variables de entorno ROS_MASTER_URI (ip del Master) y ROS_IP (IP

%rosinit('http://192.168.103.181:11311', 'NodeHost', ' 192.168.103.227')
%setenv('ROS_MASTER_URI','http://172.29.30.48:11311') % IP del entorno del robot
setenv('ROS_MASTER_URI','http://172.29.30.173:11311') % IP del entorno del robot real
setenv('ROS_IP','172.29.29.72') % IP donde se ejecute Matlab
rosshutdown; % Desconectamos por si hay algún ROS ejecutándose
rosinit % Inicialización de ROS

%% DECLARACIÓN DE SUBSCRIBERS 
laser = rossubscriber('/scan');
pause(2);
%% DECLARACIÓN DE PUBLISHERS 
pub = rospublisher('/cmd_vel', 'geometry_msgs/Twist'); 

%% GENERACIÓN DEL MENSAJE (de velocidad)
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

%% Publisher real
pub_enable=rospublisher('/cmd_motor_state','std_msgs/Int32');
msg_enable_motor=rosmessage(pub_enable);
msg_enable_motor.Data=1;
send(pub_enable,msg_enable_motor);
%% Nos aseguramos recibir un mensaje relacionado con el robot "robot0" 
pause(5);
%{
while (strcmp(laser.LatestMessage.ChildFrameId,'base_link')~=1) 
laser.LatestMessage 
end 
%}
%% Estructuras de datos
vect_distancias = [];
vect_angular = [];
paredes = 0;
umbral_laser = 1.5;

%% Capturar el mensaje más reciente del sensor láser
scanMsg = laser.LatestMessage;

% Mostrar el contenido del mensaje para verificar los nombres de los campos
disp('Contenido del mensaje LaserScan (Simulador):');
disp(scanMsg);

% Verificar si el mensaje contiene datos
if isempty(scanMsg.Ranges)
    disp('No hay datos en el mensaje del sensor láser.');
else
    % Verificar que los campos existen antes de acceder a ellos
    if isprop(scanMsg, 'AngleMin') && isprop(scanMsg, 'AngleMax') && isprop(scanMsg, 'AngleIncrement')
        % Generar el vector de ángulos correctamente
        num_points = length(scanMsg.Ranges);
        angles = linspace(scanMsg.AngleMin, scanMsg.AngleMax, num_points);
        angles_deg = rad2deg(angles);
        %{
        % Graficar los datos del sensor láser
        figure;
        plot(angles, scanMsg.Ranges);
        xlabel('Ángulo (rad)');
        ylabel('Distancia (m)');
        title('Datos del sensor láser (Simulador)');
        grid on;
        %}
    else
        disp('Error: Los campos de ángulo no existen en scanMsg.');
    end
end


%% Bucle de control infinito 
while (1) 
send(pub,msg); 
%% actualizar valor del laser
scanMsg = laser.LatestMessage;

%% Adquisición de datos de un haz concreto (por ejemplo, el central)
numBeams = length(scanMsg.Ranges);
fprintf('Numero de haces: %.2f\n', numBeams);

centerIndex = round(numBeams/2);
fprintf('indice de haz central: %.2f\n', centerIndex);

centerDistance = scanMsg.Ranges(centerIndex);

%% Si la distancia de sonar 2 es igual a 2 metros el robot se para

disp(['Distancia medida por el haz central: ', num2str(centerDistance)]);
if (centerDistance >= 1.8 && centerDistance <= 2.2)
msg.Linear.X = 0;
msg.Angular.Z = 0;
fprintf('Detencion por distancia frontal de laser');
% Comando de velocidad
send(pub,msg);
else
% Comando de velocidad
send(pub,msg);
end

%% Procesamiento del sensor láser
% Obtener un primer mensaje láser para visualizar la información
scanMsg = laser.LatestMessage;
disp('Contenido del mensaje LaserScan:');
disp(scanMsg);

% Extraer el vector de ángulos y graficar las distancias del láser
angles = scanMsg.AngleMin : scanMsg.AngleIncrement : scanMsg.AngleMax;
angles_deg = rad2deg(angles);
figure;
plot(angles, scanMsg.Ranges);
xlabel('Ángulo (rad)');
ylabel('Distancia (m)');
title('Datos del sensor láser');
grid on;


%% realizamos 100 mediciones
numMediciones = 1000;  % Número total de muestras
rawLaserData = zeros(1, numMediciones);    % Datos sin filtrar
filteredLaserData = zeros(1, numMediciones); % Datos filtrados (media móvil)
ventana = 5;  % Tamaño de la ventana
dt = 0.01;  % Intervalo de tiempo entre mediciones (segundos)

disp('Iniciando captura de datos de distancia con filtro de media móvil...');

for i = 1:numMediciones
    % Obtener el último mensaje del sensor láser
    scanMsg = laser.LatestMessage;
    
    % Verificar si el mensaje contiene datos válidos
    if  isempty(scanMsg.Ranges)
        rawDistanceData(i) = 2;  % Si no hay datos, guardar NaN
    elseif isinf(scanMsg.Ranges)
        rawDistanceData(i) = 2;  % Si no hay datos, guardar NaN
    else
        % Número total de haces y obtención del haz central
        numBeams = length(scanMsg.Ranges);
        centerIndex = round(numBeams / 2);
        
        rawDistanceData(i) = scanMsg.Ranges(centerIndex);
        if  isinf(rawDistanceData(i))
            rawDistanceData(i) = 2;  % Si no hay datos, guardar NaN
        % Almacenar la distancia del haz central en crudo
        end
    end
    
    % Aplicar filtro de media móvil con los últimos 5 valores
    if i >= ventana
        filteredDistanceData(i) = mean(rawDistanceData(i-ventana+1:i), 'omitnan');
    else
        % Para las primeras mediciones, tomar el promedio de los datos disponibles
        filteredDistanceData(i) = mean(rawDistanceData(1:i), 'omitnan');
    end
    
    pause(dt); % Mantener la frecuencia de muestreo en 100 Hz
end

disp('Captura y filtrado de datos completados.');

% Gráfica de los datos en crudo
figure; % Nueva ventana de figura
plot(rawDistanceData, 'r'); % Datos sin filtrar en rojo
xlabel('Muestra');
ylabel('Distancia (m)');
title('Mediciones en crudo del sensor láser (Haz central)');
legend('Datos en crudo');
grid on;

% Gráfica de los datos filtrados
figure;
ultimas_5_medidas = numMediciones - ventana + 1 : numMediciones; % Índices de las últimas 5 medidas
plot(ultimas_5_medidas, rawDistanceData(ultimas_5_medidas), 'ro-', 'DisplayName', 'Medición Original');
hold on;
plot(ultimas_5_medidas, filteredDistanceData(ultimas_5_medidas), 'bo-', 'LineWidth', 2, 'DisplayName', 'Medición Filtrada');
xlabel('Número de medición');
ylabel('Distancia medida (m)');
title('Últimas 5 mediciones del haz laser');
legend;
grid on;
hold off;

%% ANÁLISIS ESTADÍSTICO DEL RUIDO
maximo = max(rawDistanceData);
media = mean(rawDistanceData);
varianza = var(rawDistanceData);

%% MOSTRAR RESULTADOS
disp(['Máximo: ', num2str(maximo), ' m']);
disp(['Media: ', num2str(media), ' m']);
disp(['Varianza: ', num2str(varianza), ' m²']);


%% Apartado 6: Selección de medidas útiles para definir las cuatro paredes
% Suponiendo que el robot está en una celda rectangular y alineado con las paredes,
% definimos ventanas angulares aproximadas para cada pared:
%   - Izquierda: 60° a 120°
%   - Frontal: -30° a 30° (o, para evitar problemas de wrap-around, 330° a 360° y 0° a 30°)
%   - Derecha: -120° a -60°
%   - Trasera: 150° a 210°
leftIdx  = find(angles_deg >= 60  & angles_deg <= 120);
frontIdx = find((angles_deg >= -30 & angles_deg <= 30) | (angles_deg >= 330 & angles_deg <= 360));
rightIdx = find(angles_deg >= -120 & angles_deg <= -60);
rearIdx  = find(angles_deg >= 150 & angles_deg <= 210);

% Extraer la mediana de las distancias en cada ventana
dist_left   = median(scanMsg.Ranges(leftIdx));
dist_front  = median(scanMsg.Ranges(frontIdx));
dist_right  = median(scanMsg.Ranges(rightIdx));
dist_rear   = median(scanMsg.Ranges(rearIdx));

fprintf('Medidas de pared (Simulador): Izquierda: %.2f m, Frontal: %.2f m, Derecha: %.2f m, Trasera: %.2f m\n',...
        dist_left, dist_front, dist_right, dist_rear);

%% Apartado 7: Codificación de la presencia de paredes y evaluación de calidad
paredes = n_paredes(dist_left,dist_front,dist_right,dist_rear,umbral_laser);

%% confianza paredes
fprintf('Pared iquierda ');
confianzaIzq = confianzaPared(leftIdx,scanMsg,umbral_laser);
fprintf('Pared derecha');
confianzaDrch = confianzaPared(rightIdx,scanMsg,umbral_laser);
fprintf('Pared centro');
confianzaFrente = confianzaPared(frontIdx,scanMsg,umbral_laser);
fprintf('Pared detras');
confianzaDetras = confianzaPared(rearIdx,scanMsg,umbral_laser);

end


function paredes = n_paredes(dist_left, dist_front, dist_right, dist_rear, umbral_laser)
    dist_minima = umbral_laser;
    
    if(((dist_left  < umbral_laser)&&(dist_front  > umbral_laser)&&(dist_right  > umbral_laser)&&(dist_rear  > umbral_laser))|| ...
            ((dist_left  > umbral_laser)&&(dist_front  < umbral_laser)&&(dist_right  > umbral_laser)&&(dist_rear  > umbral_laser)) ||...
            ((dist_left  > umbral_laser)&&(dist_front  > umbral_laser)&&(dist_right  < umbral_laser)&&(dist_rear  > umbral_laser)) ||...
            ((dist_left  > umbral_laser)&&(dist_front  > umbral_laser)&&(dist_right  > umbral_laser)&&(dist_rear  < umbral_laser)))
        paredes = 1;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    elseif(((dist_left < dist_minima) && (dist_front < dist_minima) && (dist_right > dist_minima) && (dist_rear > dist_minima)) || ...
            ((dist_left < dist_minima) && (dist_front > dist_minima) && (dist_right < dist_minima) && (dist_rear > dist_minima)) || ...
            ((dist_left < dist_minima) && (dist_front > dist_minima) && (dist_right > dist_minima) && (dist_rear < dist_minima)) || ...
            ((dist_left > dist_minima) && (dist_front < dist_minima) && (dist_right < dist_minima) && (dist_rear > dist_minima)) || ...
            ((dist_left > dist_minima) && (dist_front < dist_minima) && (dist_right > dist_minima) && (dist_rear < dist_minima)) || ...
            ((dist_left > dist_minima) && (dist_front > dist_minima) && (dist_right < dist_minima) && (dist_rear < dist_minima))) 
        paredes = 2;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
   elseif (((dist_left < dist_minima) && (dist_front < dist_minima) && (dist_right < dist_minima) && (dist_rear > dist_minima)) || ...
            ((dist_left < dist_minima) && (dist_front < dist_minima) && (dist_right > dist_minima) && (dist_rear < dist_minima)) || ...
            ((dist_left > dist_minima) && (dist_front < dist_minima) && (dist_right < dist_minima) && (dist_rear < dist_minima)) || ...
            ((dist_left < dist_minima) && (dist_front > dist_minima) && (dist_right < dist_minima) && (dist_rear < dist_minima)))
        paredes = 3;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);

    elseif ((dist_left < dist_minima) && (dist_front < dist_minima) && (dist_right < dist_minima) && (dist_rear < dist_minima))
        paredes = 4;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    else
        paredes = 0;
        fprintf('Paredes que rodean al robot: %.0f\n', paredes);
    end
    
end

function confianza = confianzaPared(paredIdx, scanMsg, umbral_laser)

    % Obtener las mediciones en el índice especificado
    distancias = scanMsg.Ranges(paredIdx);

    
    % Filtrar mediciones mayores que el umbral
    distancias_validas = distancias(distancias < umbral_laser);
    
    % Verificar si hay datos válidos
    if isempty(distancias_validas)
        confianza = 0;
    else
        % Calcular distancia promedio
        dist_promedio = mean(distancias_validas);

        
        confianza = max(0, min(1, (umbral_laser - dist_promedio) / umbral_laser));
    end
    
    % Imprimir la confianza calculada
    fprintf('Confianza (Simulador): %.2f\n', confianza*2);
end
