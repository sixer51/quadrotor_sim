classdef lqr_controller < handle
    properties(Access = public)
        % altitude(1,1) double;
        % k(1,2) double;
        u0(4, 1) double;
        A(12, 12) double;
        B(12, 4) double;
        Q(12, 12) double;
        R(4, 4) double;
    end


    methods(Access = public)
        function obj = lqr_controller(quad)
            obj.u0 = quad.m*quad.g/4 * ones(4, 1);

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

            % C = diag([1000, 1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
            % obj.Q = C.' * C;
            % % obj.Q = diag([100,100, 1000, 100, 100, 100, 100, 100, 100, 1000, 1000, 1000]);
            % obj.R = eye(4)*0.001;

            % C = diag([1, 1, 10, 1, 1, 1, 1, 1, 1, 10, 10, 10]);
            % obj.Q = C.' * C;
            % obj.Q = diag([50,50, 500, 0.1, 0.1, 0.01, 1, 1, 1, 1, 1,
            % 0.01]); work for some straight line
            % obj.Q = diag([50,50, 500, 0.1, 0.1, 0.01, 1, 1, 1, 1, 1, 0.01]);
            
            % bryson's rule
            obj.Q = diag([10,10, 5, 100, 100, 100, 1, 1, 1, 1000, 1000, 1000]);
            obj.R = eye(4) * 1/(quad.mu^2);

            % obj.Q = diag([0.1, 0.1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]);
            % obj.R = eye(4);

            % obj.Q = diag([1,1, 5, 0.1,0.1, 0.1, 1,1,3,1,1, 0.1]);
            % obj.R = eye(4) * 10;

            % obj.Q = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
            % obj.R = eye(4)*0.001;

            % obj.Q = [0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %      0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %      0, 0, 0.2, 0, 0, 0, 0, 0, 0, 0, 0, 0;
            %      0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0, 0;
            %      0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0, 0;
            %      0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 0, 0;
            %      0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0, 0;
            %      0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0, 0;
            %      0, 0, 0, 0, 0, 0, 0, 0, 0.0, 0, 0, 0;
            %      0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0;
            %      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0;
            %      0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1];
            % obj.R = eye(4);
        end

        function u = output(obj, iscaptured, z, y)
            Q_captured = diag([10,10, 100, 1000, 1000, 200, 1, 1, 5, 1000, 1000, 1000]);

            % good for horizontal straight line
            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 1, 1000, 1000, 1000]);
            % Q_conservative = diag([200,200, 100, 200, 200, 200, 1, 1, 1, 1000, 1000, 1000]);
            % Q_close = diag([1000,1000, 100, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            
            % tuning for circle
            Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            Q_conservative = diag([100,100, 100, 200, 200, 200, 1, 1, 5, 1000, 1000, 1000]);
            % Q_close = diag([200,200, 2000, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            Q_close = diag([200,200, 200, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            % Q_very_close = diag([1000,1000, 100, 200, 200, 200, 0.1, 0.1, 0.1, 1000, 1000, 1000]);

            % Q_far = diag([10,10, 10, 100, 100, 100, 0.001, 0.001, 0.001, 1000, 1000, 1000]);
            % Q_conservative = diag([100,100, 100, 200, 200, 200, 0.001, 0.001, 0.001, 1000, 1000, 1000]);
            % Q_close = diag([200,200, 200, 200, 200, 200, 0.001, 0.001, 0.001, 1000, 1000, 1000]);
            

            is_inside_nest = false;
            cage_size = 10;
            x_ok = z(1) < cage_size/2 && z(1) > -cage_size/2;
            y_ok = z(2) < cage_size/2 && z(2) > -cage_size/2;
            z_ok = z(3) < cage_size && z(3) >= 0;
            if x_ok && y_ok && z_ok
                is_inside_nest = true;
            end

            if iscaptured || ~is_inside_nest
                % move to certain position
                % disp("back to home");
                zd = [[0;0;1]; zeros(9, 1)];
                z_error = z - zd;
            else
                % catch uav
                zd = [y; zeros(9, 1)];
                z_error = z - zd;
                % v = normalize(z_error(1:3))/10.;
                % z_error(7:9) = -v;
            end
            % fake gravity compensation
            z_error(3) = z_error(3)-0.1;
            
            has_aggresive_rotation = false;
            if abs(z(4)) > 0.3 || abs(z(5)) > 0.3
                has_aggresive_rotation = true;
            end
            
            Q_curr = Q_conservative;
            if iscaptured
                Q_curr = Q_captured;
            elseif abs(z_error(1)) > 2 || abs(z_error(2)) > 2 || abs(z_error(3)) > 2
                Q_curr = Q_far;
            elseif (abs(z_error(1)) < 0.8 || abs(z_error(2)) < 0.8 || abs(z_error(3)) < 0.8) && ~has_aggresive_rotation
                % z_error(7:9) = -z_error(1:3)/1000.;
                % z_error(7:9) = z(7:9)-z_error(1:3);
                % disp(z_error(7:9));
                % disp("close");
                Q_curr = Q_close;
            % elseif (abs(z_error(1)) < 0.3 || abs(z_error(2)) < 0.3 || abs(z_error(3)) < 0.3) && ~has_aggresive_rotation
            %     % z_error(7:9) = z(7:9)-z_error(1:3);
            %     % disp(z_error(7:9));
            %     % disp("very close");
            %     Q_curr = Q_very_close;
            end

            [K, ~, ~] = lqr(obj.A, obj.B, Q_curr, obj.R);

            % u = -K * (z - [[-4.8;1;3]; zeros(9, 1)]);
            % u = -K * (z - [y; zeros(9, 1)]);
            u = -K * z_error;
            
            % if iscaptured || ~is_inside_nest
            %     % move to certain position
            %     disp("outside of the nest");
            %     u = -K * (z - [[0;0;1]; zeros(9, 1)]); % + obj.u0;
            % else
            %     % catch uav
            %     u = -K * (z - [y; zeros(9, 1)]); % + obj.u0;
            % end
            

        end
    end


end