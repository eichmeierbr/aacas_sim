classdef waypointG2G < MotionController
    properties
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
            xdes(1:3) = obj.waypoints(obj.goalPt,:);
            
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