classdef lqr_controller < handle
    properties(Access = public)
        % altitude(1,1) double;
        % k(1,2) double;
        u0(1,1) double;
        A(12, 12) double;
        B(12, 4) double;
        Q(12, 12) double;
        R(4, 4) double;
    end


    methods(Access = public)
        function obj = lqr_controller(quad)
            obj.u0 = quad.m*quad.g/4;
            % obj.altitude = altitude;
            % obj.k = gains;

            obj.A = [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;
                     0, 0, 0, 0, quad.g, 0, 0, 0, 0, 0, 0, 0;
                     0, 0, 0, -quad.g, 0, 0, 0, 0, 0, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
                     0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
            obj.B = [0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0;
                     0, 0, 0, 0;
                     1/quad.m, 1/quad.m, 1/quad.m, 1/quad.m;
                     0, quad.l/quad.I(1,1), 0, -quad.l/quad.I(1,1);
                     -quad.l/quad.I(2,2), 0, quad.l/quad.I(2,2), 0;
                     quad.sigma/quad.I(3,3), -quad.sigma/quad.I(3,3), quad.sigma/quad.I(3,3), -quad.sigma/quad.I(3,3)];
            % disp(obj.A);
            % disp(obj.B);

            obj.Q = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
            obj.R = eye(4) * 0.001;

        end

        function u = output(obj, iscaptured, z, y)
            % u = obj.u0 + repmat(obj.k*[(obj.altitude - z(3)); -z(9)],[4,1]);
            [K, S, P] = lqr(obj.A, obj.B, obj.Q, obj.R);
            % disp(K);
            % u = obj.u0 * [1;1;1;1]; % + K(:, 1:12) * ()

            % catch uav
            % u = -K * (z - [y; zeros(9, 1)]);
            % move to certain position
            u = -K * (z - [[1;1;1]; zeros(9, 1)]);

        end
    end


end