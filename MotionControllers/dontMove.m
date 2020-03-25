classdef dontMove < MotionController
    methods
        function xdot = move(obj,object)
            xdot = zeros(size(object.state));
            xdot(1:3) = obj.state(1:3);
            xdot(7:9) = obj.state(7:9);
        end
    end
end