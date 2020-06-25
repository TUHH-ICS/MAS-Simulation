classdef AgentFieldSensor
    %AgentFieldSensor 
    % This object defines a sensor object that measures a noisy field 
    % values near agent position and fits a quadratic function to data     
   properties            
      sensor_range      % Range of sensor
      N                 % Number of measurements
      noise_bound       % peak bound (linf) on the noise
   end
   properties(GetAccess = private, SetAccess = immutable)
        conc_field  % external concentration field
    end
   methods
        function obj = AgentFieldSensor(sensor_range,N,noise_bound,conc_field)
            % Constructor function for setting the properties
            obj.sensor_range       = sensor_range;
            obj.N                  = N;
            obj.noise_bound        = noise_bound;  
            obj.conc_field         = conc_field;  
        end
        function Data=get_measurement(obj,agent_pos)
            % This function takes N uniformly distributed samples in a square region
            % around agent_pos of side sensor_range, measures the field values there
            % and returns the input output data set
            d=size(agent_pos,1);       % spatial dimension of agent's universe
            x=agent_pos+obj.sensor_range*(-1+2*rand(d,obj.N)); % Generate sensor data points
            y=obj.conc_field.get_field_value_at(x)+obj.noise_bound.*(-1+2*rand(obj.N,1))'; % measurements
            Data=struct;
            Data.positions=x;
            Data.values=y;
        end
        function Model_est=quadratic_regression(obj,Data)
            % This function fits a quadratic function of the form f(x)=x'Qx+b'x+c 
            % to Data given a peak bound (linf) on the noise
            d=size(Data.positions,1); % input vector size            
            %% Form the lifted Data matrix
            p=d*(d+1)/2+d+1; % Size of the lifted Data matrix
            Z=zeros(obj.N,p);
            for i=1:obj.N    
                counter=1;
                %quadratic terms first
                for m=1:d
                    for n=m:d
                        Z(i,counter)=Data.positions(m,i)*Data.positions(n,i);
                        counter=counter+1;
                    end
                end
                % linear terms
                Z(i,counter:(counter+d-1))=Data.positions(:,i)';
                % constant
                Z(i,end)=1;
            end
            %% Identify model
            % Normal equations for Least squares solution
            theta=Z\Data.values'; % Gives a LS solution for non-square A
           
            % "simplest" possible function agreeing with data in linf
            %theta=obj.eps_insensitive_loss_optimal(Z,Data.y',obj.noise_bound);
            
            % Get the quadratic model matrices back
            counter=1;
            Q_id=zeros(d,d);
            for m=1:d
                    for n=m:d
                        Q_id(m,n)=theta(counter,1);
                        counter=counter+1;
                    end
            end
            Model_est=struct;
            Model_est.Q_id=0.5*(Q_id+Q_id'); % Identified Q
            Model_est.b_id=theta(counter:(counter+d-1),1);
            Model_est.c_id=theta(end,1);
        end
        function [x]=eps_insensitive_loss_optimal(obj,A,b,t)
            % function that solves min ||x||_1 s.t ||Ax-b||_inf<t
            % Gives the "simplest" function that agrees with data (upto t in l_inf)
            nx=size(A,2);
            ny=size(A,1);
            c=ones(ny,1);
            cvx_begin quiet
                variable x(nx)
                minimize norm(x,1)
                subject to
                    A*x-b<=t*c
                    A*x-b>=-t*c
            cvx_end
        end
   end
end