classdef constantVel < MotionController
    properties
        vel
    end
    methods
        function xdes = getXdes(obj,object)
            xdes = zeros(size(object.state));
%             xdes(1:3) = obj.vel;
            object.state(4:6) = obj.vel;
        end
    end
end