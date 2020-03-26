classdef MotionController < handle
    % Class to implement motion control for the Object class
    % This class only has a move method.
    % Move returns a 12x1 array for the new x [pos,vel,orient,rot].
    % getXdot returns an xdot for the motion update
    
   properties
	A = [zeros(3), eye(3), zeros(3,6); zeros(3,12); zeros(3,9) eye(3); zeros(3,12)];
	B = [zeros(3,6); eye(3), zeros(3); zeros(3,6); zeros(3), eye(3)];
	K = [eye(3) 1.7321*eye(3) zeros(3,6); zeros(3,6) eye(3) 1.7321*eye(3)];
    t = 0;
    dt = 0.005;
    detections = [];
       
   end
   
   methods
      
       function x_out = move(obj,inState)
          xdes = obj.getXdes(inState); 
          xdot = (obj.A - obj.B*obj.K)*(inState - xdes);
          x_out = inState + xdot * obj.dt;
           
       end
       
       
       function xdes = getXdes(obj, inState)
           xdes = inState;
       end
   end
end