classdef waypointG2Gstraight < waypointG2G
    properties
        tol = 0.2;
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
        
    end
end