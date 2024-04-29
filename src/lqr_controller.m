classdef lqr_controller < handle
    properties(Access = public)
        % altitude(1,1) double;
        % k(1,2) double;
        u0(4, 1) double;
        A(12, 12) double;
        B(12, 4) double;
        Q(12, 12) double;
        R(4, 4) double;
        y_prev(5, 3) double;
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
            
            % bryson's rule
            obj.Q = diag([10,10, 5, 100, 100, 100, 1, 1, 1, 1000, 1000, 1000]);
            obj.R = eye(4) * 1/(quad.mu^2);

            % Q_far = diag([10,10, 10, 1000, 1000, 1000, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([100,100, 100, 1000, 1000, 1000, 1, 1, 1, 1000, 1000, 1000]);
            % Q_close = diag([1000,1000, 1000, 1000, 1000, 1000, 1, 1, 1, 1000, 1000, 1000]);
            % Q_approach = diag([1000,1000, 1000, 10, 10, 10, 1, 1, 1, 10, 10, 10]);
            % 
            % [K, ~, ~] = lqr(obj.A, obj.B, Q_far, obj.R)
            % [K, ~, ~] = lqr(obj.A, obj.B, Q_conservative, obj.R)
            % [K, ~, ~] = lqr(obj.A, obj.B, Q_close, obj.R)
            % [K, ~, ~] = lqr(obj.A, obj.B, Q_approach, obj.R)

        end

        function y_next = predict_next_y(obj, y, y_all, z_error)
            % k_t = 1;
            % if isempty(y_all)
            %     y_next = y;
            % else
            %     y_last = y_all(end-1, :)';
            %     dy = y - y_last;
            % 
            %     y_next = y + dy*k_t;
            %     % disp(y_last);
            %     % disp("next");
            %     % disp(y);
            %     % disp(y_next);
            % end
            y_next = y;
            
        end

        function u = output(obj, iscaptured, z, y, y_all)
            Q_captured = diag([10,10, 100, 10000, 10000, 1000, 1, 1, 10, 1000, 1000, 1000]);

            % good for horizontal straight line
            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([200,200, 100, 200, 200, 200, 1, 1, 1, 1000, 1000, 1000]);
            % Q_close = diag([1000,1000, 100, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            
            % tuning for circle
            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([100,100, 100, 200, 200, 200, 1, 1, 5, 1000, 1000, 1000]);
            % % Q_close = diag([200,200, 2000, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            % Q_close = diag([100,100, 100, 100, 100, 100, 1, 1, 1, 1000, 1000, 1000]);
            % Q_very_close = diag([1000,1000, 100, 200, 200, 200, 0.1, 0.1, 0.1, 1000, 1000, 1000]);

            % Q_far = diag([10,10, 10, 100, 100, 100, 0.001, 0.001, 0.001, 1000, 1000, 1000]);
            % Q_conservative = diag([100,100, 100, 200, 200, 200, 0.001, 0.001, 0.001, 1000, 1000, 1000]);
            % Q_close = diag([200,200, 200, 200, 200, 200, 0.001, 0.001, 0.001, 1000, 1000, 1000]);

            % Q_far = diag([10,10, 10, 100, 100, 100, 2, 2, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([100, 100, 100, 200, 200, 200, 1, 1, 5, 500, 500, 500]);
            % Q_close = diag([500,500, 500, 500, 500, 500, 1, 1, 1, 500, 500, 500]);
            % Q_approach = diag([1500,1500, 1500, 100, 100, 100, 1, 1, 1, 500, 500, 500]);
            
            % can catch path = @(t) [3; -5+0.2*t; 4]; with distance 1m as threshold
            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([1000, 1000, 1000, 200, 200, 200, 1, 1, 5, 500, 500, 500]);
            % Q_captured = diag([10,10, 100, 1000, 1000, 1000, 1, 1, 10, 1000, 1000, 1000]);

            % can catch path = @(t) [3; -5+0.4*t; 4]; with distance 1m, 0.6m as threshold
            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([1000, 1000, 1000, 200, 200, 200, 1, 1, 5, 500, 500, 500]);
            % Q_close = diag([1000,1000, 1000, 500, 500, 500, 1, 1, 5, 500, 500, 500]);
            
            % catched path = @(t) [-5+0.3*t; 2; 2+cos(t)]; 0.8m, 0.6m
            Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            Q_conservative = diag([1200, 1200, 2000, 1000, 1000, 500, 1, 1, 5, 500, 500, 500]);
            Q_close = diag([1000,1000, 2000, 1000, 1000, 1000, 1, 1, 5, 1000, 1000, 1000]);

            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([1200, 1200, 2000, 1000, 1000, 500, 1, 1, 5, 500, 500, 500]);
            % Q_close = diag([2000,2000, 4000, 1000, 1000, 1000, 1, 1, 5, 1000, 1000, 1000]);
           

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
                zd = [[0;0;2]; zeros(9, 1)];
                z_error = z - zd;
            else
                % catch uav
                zd = [y; zeros(9, 1)];
                z_error = z - zd;

                % y_next = obj.predict_next_y(y, y_all, z_error);
                % zd = [y_next; zeros(9, 1)];
                % z_error = z - zd;

                % v = normalize(z_error(1:3))/10.;
                % z_error(7:9) = -v;
            end
            % fake gravity compensation
            % z_error(3) = z_error(3)-0.1;
            
            has_aggresive_rotation = false;
            if abs(z(4)) > 0.1 || abs(z(5)) > 0.1
                has_aggresive_rotation = true;
            end
            
            Q_curr = Q_conservative;
            distance = vecnorm(z(1:3) - y);
            if iscaptured
                Q_curr = Q_captured;
            elseif distance > 0.8
                Q_curr = Q_far;
                % z_error(3) = z_error(3)+0.3;
            elseif distance < 0.6 && ~has_aggresive_rotation
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