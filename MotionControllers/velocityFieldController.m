classdef velocityFieldController < MotionController
    properties
        tol = 0.2;
        
        vel
        waypoints = [0, 20, 0; ...
                     0 0 0];
        goalPt = 1;
        dist = 1;
    end
    methods        
        
        function xdes = getXdes(obj,object)
            xdes = zeros(size(object));
            obj = obj.changeGoalPt(object(1:3));
            goal = obj.waypoints(obj.goalPt,:);
            
            angleToGoal = atan2(goal(2)-object(2),goal(1)-object(1));
            xdes(7) = angleToGoal;
            angleDiff = angleToGoal - object(7);
            
            if abs(angleDiff) > obj.tol
                xdes(1:3) = object(1:3);
            else
                xdes(1:3) = goal;
            end
            a=4;
        end
            
        
        function obj = changeGoalPt(obj,pos)
            dis = norm(pos'-obj.waypoints(obj.goalPt,:));
            if(dis < obj.dist)
                obj.goalPt = obj.goalPt + 1;
                if(obj.goalPt > length(obj.waypoints(:,1)))
                    obj.goalPt = 1;
                end                
            end
        end
            
        
    end
end