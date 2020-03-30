classdef PerfectSensor < VisionSensor
    %RangeSensor Creates a sensor based on a limited range
    
    methods (Access = public)
        function obj = PerfectSensor()
            obj.fov_h = 360*pi/180/2;
            obj.pos_bias = 0;
            obj.pos_dev = 0;
        end
        
        function plotMeasurements(obj, q, xo, yo)   
            % Loop through and update the lines 
            for k = 1:length(xo)
                xdata = [q(1) xo(k)];
                ydata = [q(2) yo(k)];
%                 set(obj.h_obs_lines{k}, 'xdata', xdata, 'ydata', ydata);
                plot(xdata,ydata, '-k')
            end
            
        end
    end
    
end

