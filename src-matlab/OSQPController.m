classdef OSQPController
    properties
        P
        q
        A
        lb
        ub
        Nmpc
        Xref
        Uref
        times
        
        n
        m
        Np
        Nd
    end
    
    methods
        function obj = untitled7(inputArg1,inputArg2)
            %UNTITLED7 Construct an instance of this class
            %   Detailed explanation goes here
            obj.Property1 = inputArg1 + inputArg2;
        end
        
        function outputArg = method1(obj,inputArg)
            %METHOD1 Summary of this method goes here
            %   Detailed explanation goes here
            outputArg = obj.Property1 + inputArg;
        end
    end
end

