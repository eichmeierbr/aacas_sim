classdef waypointG2Gstraight < waypointG2G
    properties
        tol = 0.2;
    end
    methods        
        
        function xdes = move(obj,object)
            xdes = zeros(size(object.state));
            obj = obj.changeGoalPt(object.state(1:3));
            goal = obj.waypoints(obj.goalPt,:);
            
            angleToGoal = atan2(goal(2)-object.state(2),goal(1)-object.state(1));
            xdes(7) = angleToGoal;
            angleDiff = angleToGoal - object.state(7);
            
            if abs(angleDiff) > obj.tol
                xdes(1:3) = object.state(1:3);
            else
                xdes(1:3) = goal;
            end
            
            
            a=4;
        end
        
    end
end