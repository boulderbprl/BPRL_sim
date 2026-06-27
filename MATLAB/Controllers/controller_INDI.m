classdef controller_INDI < handle
    % PID controller based on Ardupilot

    properties
        Kp = 1.0; % proportional gain

    end

    properties(Access=private)
        
        g_fcn = @(x) NaN;
    end

    methods
        function obj = controller_INDI(Kp,g_fcn)
            
            obj.Kp = Kp;
            obj.g_fcn = g_fcn;

        end

        function reset(~)
            pass;

        end
        
        function g_inv = compute_ginv(obj,state)
                
            ctrl_alloc = obj.g_fcn(state);
            g_inv = inv(ctrl_alloc);
            
        end
        
        function output = update(obj,target,measurement,state,current_input)
            
            % compute the tracking error 

            error = target - measurement; 

            %1. compute the delta input term

            delta_v= obj.Kp*error; 

            %2. compute the g(x) term
            
            g_inv = obj.compute_ginv(state);
            
            %3. delta_u = output
            delta_u = g_inv*delta_v;

            
            
            % Compute the final output
            output = current_input + delta_u ;
              

        end


    end
end