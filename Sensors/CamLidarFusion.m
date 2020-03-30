classdef CamLidarFusion < handle
    properties
        vision
        lidar
    end
	
    methods
        function obj = CamLidarFusion()
           obj.vision = VisionSensor;
           obj.lidar = RangeSensor;
        end
        
         function detections = getObstacleDetections(obj,q, th, world)
            visionDetections = obj.vision.getObstacleDetections(q, th, world);
%             obj.vision.plotMeasurements(q,xo,yo);
            lidarDetections = obj.lidar.getObstacleDetections(q, th, world);
%             obj.lidar.plotMeasurements(q,xo2,yo2);
            
            detections = [visionDetections lidarDetections];
         end 
         
         function plotMeasurements(obj, q, xo, yo)
            obj.vision.plotMeasurements(q,xo,yo);
            obj.lidar.plotMeasurements(q,xo,yo);
         end
    end
end