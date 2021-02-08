classdef DataLeech < handle
    %DATALEECH Class that helps with repeatedly sampling data.
    %   This class can be used to sample data from a collection of agents
    %   at each sampling time. This is required, as the agents only save
    %   the data of the current timestep, not their historic position and
    %   velocities.
    
    properties(GetAccess = public, SetAccess = private)
        data    % Data that was saved from the agents
        t       % List of time instances, at whcih the sampling occured
    end
    
    properties(GetAccess = public, SetAccess = immutable)
        targets % Target properties of the host
        steps   % Number of timesteps that can be saved by this instance
    end
    
    properties(GetAccess = private, SetAccess = private)
        k       % Internal counter for the number of saved datapoints
    end
    
    properties(GetAccess = private, SetAccess = immutable)
        host    % Object that the data gets saved from
    end
    
    methods
        function obj = DataLeech(host, steps, varargin)
            %DATALEECH Construct an instance of this class
            %   The host is the object that the data should be saved from,
            %   steps is the number of timesteps that can be saved and
            %   after those two, you need to provide a list of properties
            %   to save.
            
            if nargin <= 2
                error('You need to name at least one target property')
            end
            
            obj.k       = 1;
            obj.t       = zeros(steps, 1);
            
            obj.host    = host;
            obj.steps   = steps;
            obj.targets = {};
            obj.data    = struct;
            
            % Assemble a list of public properties that the host has
            clazz    = metaclass(host);
            pubmask  = strcmp({clazz.PropertyList.GetAccess}, 'public');
            pubprobs = {clazz.PropertyList(pubmask).Name};
            
            % Build a datastorage from the provided list of properties
            for field = varargin
                name = field{:};
                if any(strcmp(pubprobs, name))
                    obj.targets = [obj.targets, field];
                else
                    warning("The host does not have a property called '%s'. This variable will be ignored.", name)
                end
            end
        end
        
        function save(obj, t)
            %SAVE Saves one set of data from the host
            
            if obj.k > obj.steps
                error('The target number of steps was exceeded')
            end
            
            % Initialize all variables
            if obj.k == 1
                for field = obj.targets
                    name = field{:};
                    val  = [obj.host.(name)];
                    obj.data.(name) = zeros([obj.steps, size(val(:))]);
                end
            end
            
            % Save data for each desired property
            for field = obj.targets
                name = field{:};
                val  = [obj.host.(name)];
                obj.data.(name)(obj.k,:) = val(:);
            end
            
            % Update time instance
            obj.t(obj.k) = t;
            obj.k = obj.k + 1;
        end
        
        function [t, data] = get(obj)
            %GET Export saved data
            
            t    = obj.t;
            data = obj.data;
        end
        
        function [t, sampled] = resample(obj, dT, method)
            %RESAMPLE Resamples the saved data onto a time grid with
            %constant stepsize dT.
            %   Depending on the simulation, the number of timesteps can be
            %   very large and the difference between two timesteps can be
            %   non uniform. In these cases, it is benifical to resample
            %   the data onto a uniform grid.
            
            if nargin <= 2
                method = 'zoh';
            end
            
            % Build timeseries object for each property
            tsin  = tscollection(obj.t); 
            for field = obj.targets
                name  = field{:};
                list  = obj.data.(name);
                list  = permute(list, [2:length(size(list)), 1]);
                ts    = timeseries(list, obj.t, 'Name', name);
                tsin  = addts(tsin, ts);
            end
            
            % Resample with zero order hold behaviour onto uniform grid
            tsout = resample(tsin, 0:dT:max(obj.t), method);

            % Export resampled data as a structure
            t = tsout.Time;
            sampled = struct;
            for field = obj.targets
                name  = field{:};
                val   = tsout.(name).Data;
                dims  = length(size(val));
                sampled.(name) = permute(val, [dims, 1:(dims-1)]);
            end
        end
    end
end
