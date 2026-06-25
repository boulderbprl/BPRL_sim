classdef controller_PID < handle
    % PID controller based on Ardupilot

    properties
        Kp = 1.0; % proportional gain
        Ki = 0.1; % integrator gain
        Kd = 0.02; % derivative gain
        Ff = 0.0; % feedforward gain

        % Integrator anti-windup limits 

        imax = 6.0; 

        % Filter Cutoff Frequencies 
        fCutDeriv = 20.0; % cutoff frequency for derivative low-pass filter
        fCutError = 0.0; % 0 disbales error filtering 
    end

    properties(Access=private)
        
        integrator = 0.0; 
        last_error = 0.0; 
        last_derivative = NaN; % for initialization purposes
    end

    methods
        function obj = controller_PID(Kp, Ki,Kd,Ff,imax, fCutDeriv)
            
            obj.Kp = Kp;
            obj.Ki = Ki;
            obj.Kd = Kd;
            
            if nargin>3, obj.Ff = Ff; end
            if nargin > 4, obj.imax = imax; end
            if nargin > 5, obj.fCutDeriv = fCutDeriv; end

        end

        function reset(obj)
            
            obj.integrator = 0.0; 
            obj.last_error = 0.0; 
            obj.last_derivative = NaN;
        
        end
        
        
        function output = update(obj,target,measurement,dt)
            
            % compute the tracking error 

            error = target - measurement; 

            %1. compute the proportional term

            P_out = obj.Kp*error; 

            %2. compute the integrator term (with anti-windup clamping)
            
            I_out = 0.0;
            
            if obj.Ki>0 && dt>0
                
                obj.integrator = obj.integrator + obj.Ki*error*dt; 

                % clamp integrator to imax limits 
                if obj.integrator > obj.imax
                    obj.integrator = obj.imax; 
                elseif obj.integrator < -obj.imax
                    obj.integrator = -obj.imax; 
                end

                I_out = obj.integrator;

            else
                obj.integrator=0.0;
            end

           
            %3. compute the derivative term 
            D_out = 0.0; 

            if obj.Kd>0 && dt>0
                
                raw_derivative = (error-obj.last_error)/dt;
                
                if isnan(obj.last_derivative)
                    derivative = raw_derivative;  % first sample initialization
               
                else

                % Low pass filter computation

                    if obj.fCutDeriv > 0
                        rc = 1/(2*pi*obj.fCutDeriv); 
                        alpha = dt/(rc+dt); 
                        
                        derivative = alpha*raw_derivative + (1 -alpha)*obj.last_derivative;
                    else
                        derivative = raw_derivative; 
                    end
                end

              obj.last_derivative = derivative; 

              D_out = obj.Kd*derivative;
            
            end

            Ff_out = obj.Ff*target; % feedforward term 


            % Compute the final output
            output = P_out + I_out + D_out + Ff_out;

            % Update the last error for the next iteration
            obj.last_error = error;
               

        end


    end
end