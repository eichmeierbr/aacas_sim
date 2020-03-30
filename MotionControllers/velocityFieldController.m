classdef velocityFieldController < MotionController
    properties
        
        v_max = 5;
        inplot;
        
        % Waypoint params
        waypoints = [0, 20, 0; ...
                     0 0 0];
        goalPt = 1;
        switch_dist = 1;
        
        % Orbit params
        safe_dist = 8;
        freq = 1;% Orbit direction (+: CW, -: ccw)
        rad % Radius of orbit
        k_conv = .01; % Gain to converge to orbit
        K_theta = 2;
        
        % Go to Goal Parms
        g2g_sig = 1.5;
        g2g_sig_sq;
    end
    
    methods        
        function obj = velocityFieldController()
            obj.A = zeros(4);
            obj.B = eye(4);
            Q = eye(4);
            R = eye(4);
            obj.K = lqr(obj.A,obj.B,Q,R);
            
            obj.rad = obj.safe_dist - 2; % Radius of orbit
            obj.g2g_sig_sq = obj.g2g_sig^2;
        end
        
        function velDes = getXdes(obj,veh_state)
            velDes = zeros(4,1);          
            goal = obj.waypoints(obj.goalPt,:);
            
            % Check if we are close to an object
            [closeObject, move] = obj.getCloseObject(veh_state);
            
            % If close to object, orbit
            if and(move,closeObject.dist < obj.safe_dist)
                obj.decideOrbitDirection(closeObject, veh_state);
                velDes(1:3) = obj.getOrbit(closeObject.pos,veh_state)';          
            else % Go to goal 
                velDes(1:3) = obj.goToGoalField(veh_state);
            end
            
            % Normalize velocity
            if norm(velDes(1:3)) > obj.v_max
                velDes(1:3) = velDes(1:3)/norm(velDes(1:3))*obj.v_max;
            end
            
            % Heading Control
            w_d = obj.headingControl(velDes, veh_state);
            velDes(4) = w_d;
        end
            
        
       function x_out = move(obj,inState)
           % Check if we have reached the next waypoint. If so, update
           obj.changeGoalPt(inState(1:3));
           obj.inplot=true;
          obj.plotVectField(inState);
          obj.inplot=false;
          
          x_out = zeros(12,1);
          velDes = obj.getXdes(inState); % Get velocity vector
          
          velDot = (obj.A - obj.B*obj.K)*(inState(4:7) - velDes); % Control
          
          % Prepare Output Vector
          x_out(4:6) = inState(4:6) + velDot(1:3) * obj.dt;
          x_out(1:3) = inState(1:3) + x_out(4:6)* obj.dt;
          x_out(10) = inState(10) + velDot(4) * obj.dt;
          x_out(7) = inState(7) + x_out(7)* obj.dt;
       end
        
       function [closeObject, move] = getCloseObject(obj, veh_state)
           closeObject = Detections;
           closeObject.dist = obj.safe_dist;
           move = false;
           
           goal = obj.waypoints(obj.goalPt,:);
           % Perform transformed coordinates
           dp = goal' - veh_state(1:3);
           th = wrapToPi(atan2(dp(2),dp(1)) - pi/2);
           R_ov = [cos(th) -sin(th); sin(th) cos(th)];
           t_ov = veh_state(1:2);
           T_vo = [R_ov' -R_ov'*t_ov; 0 0 1];          
           
           for i=1:size(obj.detections)
               if obj.detections(i).dist < closeObject.dist
                   closeObject = obj.detections(i);
                   move = true;
               end
           end
       end
       
       function changeGoalPt(obj,pos)
            dist_to_goal = norm(pos'-obj.waypoints(obj.goalPt,:));
            if(dist_to_goal < obj.switch_dist)
                obj.goalPt = obj.goalPt + 1;
                if(obj.goalPt > length(obj.waypoints(:,1)))
                    obj.goalPt = 1;
                end                
            end
        end

        function plotVectField(obj, in_state)
            check_state = in_state;
            cushion = 20;
            spacing = 3;
            x_in = in_state(1)-cushion:spacing:in_state(1)+cushion;  
            y_in = in_state(2)-cushion:spacing:in_state(2)+cushion;

            [x,y] = meshgrid(x_in, y_in);
            xVec = zeros(size(x));
            yVec = zeros(size(y));
            for i=1:length(x_in)
                for j=1:length(y_in)
                    check_state(1) = x_in(j);
                    check_state(2) = y_in(i);
                    velo = getXdes(obj,check_state);
                    xVec(i,j) = velo(1);
                    yVec(i,j) = velo(2);
                end
            end

            quiver(x,y,xVec,yVec,'k-')            
        end
        
        function w_d = headingControl(obj, velDes, veh_state)
            vel_angle = wrapToPi(atan2(velDes(2), velDes(1)));
            angleDiff = vel_angle - veh_state(7);
            w_d = obj.K_theta * angleDiff;
        end
        
        function velDes = getOrbit(obj, center, veh_state)
               xhat = veh_state(1:2) - center(1:2); % Change to orbit coords
               gam = obj.k_conv*(obj.rad^2 - (xhat'*xhat)); % Convergence to orbit

               A = [gam, obj.freq; -obj.freq, gam]; % Modified harmonic oscillator
               g = A*xhat(1:2);   %  Calculate nominal velocity
               
               % Scale the vector field
               v_g = norm(g);
               if v_g > obj.v_max
                   g = obj.v_max/v_g * g; 
               end
               % Pad output with z-vel
               velDes = [g; 0];
        end
        
        
        function decideOrbitDirection(obj, closeObst, veh_state)
            % Note: All directions are assuming the vehicle is looking
            % straight at the goal
            
            obst_vel = closeObst.vel;
            obst_pos = closeObst.pos;
            goal = obj.waypoints(obj.goalPt,:);
            
            % Perform transformed coordinates
            dp = goal' - veh_state(1:3);
            th = wrapToPi(atan2(dp(2),dp(1)) - pi/2);
            R_ov = [cos(th) -sin(th); sin(th) cos(th)];
            t_ov = veh_state(1:2);
            T_vo = [R_ov' -R_ov'*t_ov; 0 0 1];
            
            trans_vel = T_vo * [obst_vel(1:2); 0];
            trans_pos = T_vo * [obst_pos(1:2); 1];
            % Check if object is stationary
            if norm(trans_vel) > 0.1
                if(trans_vel(1) >= 0) % If obstacle is moving right
                    obj.freq = 1; % Orbit CW
                else % If obstacle is moving left
                    obj.freq = -1; % Orbit CCW
                end
            % else object is stationary
            else
                if(trans_pos(1) >= 0) % If object is to the right
                    obj.freq = 1; % Orbit CW
                else % If object is to the left
                    obj.freq = -1; % Orbit CCW
                end
            end
            
        end
        
        
        function velDes = goToGoalField(obj, veh_state)
            g = obj.waypoints(obj.goalPt,:)' - veh_state(1:3);
            
            % Scale the magnitude of the resulting vector
            dist2goal = norm(g);
            v_g = obj.v_max * (1- exp(-dist2goal^2/obj.g2g_sig_sq));
            
            if dist2goal > 0 % Avoid dividing by zero
                velDes = v_g/dist2goal * g; % Dividing by dist is dividing by the norm
            else
                velDes = [0;0;0];
            end
            
            if v_g > obj.v_max
                g = obj.vel/v_g * g; 
            end
        end
        
    end
end