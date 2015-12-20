classdef Estimador < handle
    %UNTITLED3 Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        a_world_t;
        a_world_x;
        a_world_y;
        a_world_z;
        
        a_real_t;
        a_real_x;
        a_real_y;
        a_real_z;
        
        q_t;
        q_w;
        q_x;
        q_y;
        q_z;
        
        t_ypr;
        yaw;
        pitch;
        row;
        
        t_euler;
        euler_x;
        euler_y;
        euler_z;
        
        acc_magFilt;
        stationary;
        
        accX;
        accY;
        accZ;
        
        t;
        s;
        v;
    end
    
    methods
        function obj = Estimador(obj)
            obj.carregar_dados();
        end
        function obj = processar_dados(obj)
            obj.s = zeros(3,1);
            obj.v = zeros(3,1);
            
            obj.accX = obj.a_world_x;
            obj.accY = obj.a_world_y;
            obj.accZ = obj.a_world_z;
            
            obj.t = obj.a_world_t;
            
            samplePeriod = mean(diff(obj.t));
            % -------------------------------------------------------------------------
            % Detect stationary periods
            
            % Compute accelerometer magnitude
            acc_mag = sqrt(obj.accX.*obj.accX + obj.accY.*obj.accY + obj.accZ.*obj.accZ);
            
            % HP filter accelerometer data
            filtCutOff = 0.001;
            [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'high');
            obj.acc_magFilt = filtfilt(b, a, acc_mag);
            
            % Compute absolute value
            obj.acc_magFilt = abs(obj.acc_magFilt);
            
            % LP filter accelerometer data
            filtCutOff = 5;
            [b, a] = butter(1, (2*filtCutOff)/(1/samplePeriod), 'low');
            obj.acc_magFilt = filtfilt(b, a, obj.acc_magFilt);
            
            % Threshold detection
            obj.stationary = obj.acc_magFilt < 0.05;
            
            % -------------------------------------------------------------------------
            
            acc = [obj.accX; obj.accY; obj.accZ];
            obj.v(:,1) = acc(:,1)*samplePeriod;
            obj.s(:,1) = zeros(3,1);
            
            for i=2:length(obj.t)
                dt = obj.t(i)-obj.t(i-1);
                obj.v(:,i) = obj.v(:,i-1)+acc(:,i)*dt;
                obj.s(:,i) = obj.s(:,i-1)+obj.v(:,i)*dt;
            end
            
            f = figure('MenuBar','None','Name','Dados estimados');
            mh = uimenu(f,'Label','Janelas');
            frh = uimenu(mh,'Label','Estacionário',...
                'Callback','ans.estacionario();');
            frh = uimenu(mh,'Label','Cinemática',...
                'Callback','ans.cinematica();');
            frh = uimenu(mh,'Label','Mapa',...
                'Callback','ans.mapa();');
            
        end
        function obj = mapa(obj)
            subplot(1,3,1);
            plot(obj.s(1,:),obj.s(2,:));
            title('Mapa de deslocamento')
            xlabel('X')
            ylabel('Y')
            subplot(1,3,2);
            plot3(obj.s(1,:),obj.s(2,:),obj.v(1,:));
            title('Mapa de velocidade no espaco');
            xlabel('X')
            ylabel('Y')
            zlabel('Vx')
            subplot(1,3,3);
            plot3(obj.s(1,:),obj.s(2,:),obj.v(2,:));
            title('Mapa de velocidade no espaco');
            xlabel('X')
            ylabel('Y')
            zlabel('Vy')
        end
        function obj = estacionario(obj)
            subplot(1,2,1);
            plot(obj.acc_magFilt);
            title('Filtro de magnitude')
            subplot(1,2,2);
            plot(obj.stationary);
            title('Estacionário');
        end
        function obj = cinematica(obj)
            subplot(3,2,1);
            plot(obj.t,obj.accX);
            title('Aceleração no tempo em X')
            subplot(3,2,2);
            plot(obj.t,obj.accY);
            title('Aceleração no tempo em Y')
            subplot(3,2,3);
            plot(obj.t,obj.v(1,:));
            title('Velocidade no tempo em X')
            subplot(3,2,4);
            plot(obj.t,obj.v(2,:));
            title('Velocidade no tempo em Y')
            subplot(3,2,5);
            plot(obj.t,obj.v(1,:));
            title('Posição no tempo em X')
            subplot(3,2,6);
            plot(obj.t,obj.v(2,:));
            title('Posição no tempo em Y')
        end
        
    end
    methods (Access = private)
        function obj = carregar_dados(obj)
            % Abrir todos os arquivos para ler os dados
            fileID = fopen('saida_world_acc.txt','r');
            fileID1 = fopen('saida_real_acc.txt','r');
            fileID2 = fopen('saida_quaternion.txt','r');
            fileID3 = fopen('saida_ypr.txt','r');
            fileID4 = fopen('saida_euler.txt','r');
            
            formatSpec = '%f    %f     %f    %f';
            
            % Guardando os dados nas variáveis
            
            A = fscanf(fileID,formatSpec,[4 Inf]);
            B = fscanf(fileID1,formatSpec,[4 Inf]);
            C = fscanf(fileID2,formatSpec,[5 Inf]);
            D = fscanf(fileID3,formatSpec,[4 Inf]);
            E = fscanf(fileID4,formatSpec,[4 Inf]);
            
            % Fechando os arquivos utilizados
            
            fclose(fileID);
            fclose(fileID1);
            fclose(fileID2);
            fclose(fileID3);
            fclose(fileID4);
            
            % Obtendo os dados do acelerômetro em relação ao mundo
            
            obj.a_world_t = A(1,:)/1000;
            a_world_x = A(2,:);
            a_world_y = A(3,:);
            a_world_z = A(4,:);
            
            % Removendo ruidos
            
            obj.a_world_x = Filtro_exponencial(a_world_x);
            obj.a_world_y = Filtro_exponencial(a_world_y);
            obj.a_world_z = Filtro_exponencial(a_world_z);
            
            % Obtendo os dados do acelerômetro reais, sem orientação
            
            obj.a_real_t = B(1,:)/1000;
            obj.a_real_x = B(2,:);
            obj.a_real_y = B(3,:);
            obj.a_real_z = B(4,:);
            
            % Quaternions
            
            obj.q_t = C(1,:)/1000;
            obj.q_w = C(2,:);
            obj.q_x = C(3,:);
            obj.q_y = C(4,:);
            obj.q_z = C(5,:);
            
            % Ângulos de orientação
            
            obj.t_ypr = D(1,:)/1000;
            obj.yaw = D(2,:);
            obj.pitch = D(3,:);
            obj.row = D(4,:);
            
            %Matriz de euler
            
            obj.t_euler = E(1,:)/1000;
            obj.euler_x = E(2,:);
            obj.euler_y = E(3,:);
            obj.euler_z = E(4,:);
        end
        
    end
    
end

