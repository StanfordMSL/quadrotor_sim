
classdef AttitudeTarget < ros.Message
    %AttitudeTarget MATLAB implementation of mavros_msgs/AttitudeTarget
    %   This class was automatically generated by
    %   ros.internal.pubsubEmitter.
    %   Copyright 2014-2020 The MathWorks, Inc.
    properties (Constant)
        MessageType = 'mavros_msgs/AttitudeTarget' % The ROS message type
    end
    properties (Constant, Hidden)
        MD5Checksum = '456f3af666b22ccd0222ea053a86b548' % The MD5 Checksum of the message definition
        PropertyList = { 'Header' 'Orientation' 'BodyRate' 'TypeMask' 'Thrust' } % List of non-constant message properties
        ROSPropertyList = { 'header' 'orientation' 'body_rate' 'type_mask' 'thrust' } % List of non-constant ROS message properties
        PropertyMessageTypes = { 'ros.msggen.std_msgs.Header' ...
			 'ros.msg.geometry_msgs.Quaternion' ...
			 'ros.msggen.geometry_msgs.Vector3' ...
			 '' ...
			 '' ...
			 } % Types of contained nested messages
    end
    properties (Constant)
        IGNOREROLLRATE = uint8(1)
        IGNOREPITCHRATE = uint8(2)
        IGNOREYAWRATE = uint8(4)
        IGNORETHRUST = uint8(64)
        IGNOREATTITUDE = uint8(128)
    end
    properties
        Header
        Orientation
        BodyRate
        TypeMask
        Thrust
    end
    methods
        function set.Header(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.std_msgs.Header'};
            validateattributes(val, validClasses, validAttributes, 'AttitudeTarget', 'Header')
            obj.Header = val;
        end
        function set.Orientation(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msg.geometry_msgs.Quaternion'};
            validateattributes(val, validClasses, validAttributes, 'AttitudeTarget', 'Orientation')
            obj.Orientation = val;
        end
        function set.BodyRate(obj, val)
            validAttributes = {'nonempty', 'scalar'};
            validClasses = {'ros.msggen.geometry_msgs.Vector3'};
            validateattributes(val, validClasses, validAttributes, 'AttitudeTarget', 'BodyRate')
            obj.BodyRate = val;
        end
        function set.TypeMask(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'AttitudeTarget', 'TypeMask');
            obj.TypeMask = uint8(val);
        end
        function set.Thrust(obj, val)
            validClasses = {'numeric'};
            validAttributes = {'nonempty', 'scalar'};
            validateattributes(val, validClasses, validAttributes, 'AttitudeTarget', 'Thrust');
            obj.Thrust = single(val);
        end
    end
    methods (Static, Access = {?matlab.unittest.TestCase, ?ros.Message})
        function obj = loadobj(strObj)
        %loadobj Implements loading of message from MAT file
        % Return an empty object array if the structure element is not defined
            if isempty(strObj)
                obj = ros.msggen.mavros_msgs.AttitudeTarget.empty(0,1);
                return
            end
            % Create an empty message object
            obj = ros.msggen.mavros_msgs.AttitudeTarget;
            obj.reload(strObj);
        end
    end
end
