classdef lqr_controller < handle
    properties(Access = public)
        % altitude(1,1) double;
        % k(1,2) double;
        u0(1,1) double;
        A(12, 12) double;
        B(12, 4) double;
        Q(13, 13) double;
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
<<<<<<< Updated upstream
            % disp(obj.A);
            % disp(obj.B);
=======
            
            % bryson's rule
            obj.Q = diag([10,10, 5, 100, 100, 100, 1, 1, 1, 1000, 1000, 1000, 0.00001]);
            obj.R = eye(4) * 1/(quad.mu^2);
>>>>>>> Stashed changes

%             C = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
%             C = diag([10, 10, 10, 0.1, 0.1, 0.1, 10, 10, 10, 10, 10, 10]); %this is funny
%             C = diag([15, 15, 15, 0.1, 0.1, 0.1, 10, 10, 10, 35, 35, 35]);%v close
%             C = diag([15, 15, 15, 0.1, 0.1, 0.1, 20, 20, 20, 40, 40, 40]);
%             C = diag([1000,1000, 1000, 100, 100, 100, 100, 100, 100, 10000, 10000, 10000]);
            obj.Q = diag([1,1, 5, 0.1,0.1, 0, 1,1,3,1,1, 0, 0.00001]);
            obj.R = eye(4) * 10;

        end

        function u = output(obj, iscaptured, z, y)
            % u = obj.u0 + repmat(obj.k*[(obj.altitude - z(3)); -z(9)],[4,1]);
%             [K, S, P] = lqr(obj.A, obj.B, obj.Q, obj.R);
            % disp(K);
            % u = obj.u0 * [1;1;1;1]; % + K(:, 1:12) * ()

            C = ones(1,12);
            D = [0 0 0 0];
            [K, ~, ~] = lqi(ss(obj.A,obj.B,C,D),obj.Q,obj.R,0);
            ud = obj.u0 * ones(4,1);
            
<<<<<<< Updated upstream
            if iscaptured
                target = zeros(12,1);  % all states to zero
                target(3) = 1;  
                u = ud + K(:,1:12)*(target - z);
            else
                % catch uav
%                 u = -K * (z - [y; zeros(10, 1)]);
                target = [y; zeros(9, 1)];
                u = ud + K(:,1:12)*(target - z);
            end           
=======
        end

        function u = output(obj, iscaptured, z, y, y_all)
            Q_captured = diag([10,10, 100, 1000, 1000, 1000, 1, 1, 10, 1000, 1000, 1000, 0.00001]);

            % good for horizontal straight line
            % Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
            % Q_conservative = diag([200,200, 100, 200, 200, 200, 1, 1, 1, 1000, 1000, 1000]);
            % Q_close = diag([1000,1000, 100, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            
%             tuning for circle
%             Q_far = diag([170, 170, 170, 30, 30, 200, 1, 1, 5, 1000, 1000, 1000, 0.00001]);
            Q_far = diag([50,50,50, 1,1,1, 1,1,3, 10,10,10, 0.00001]);
            Q_conservative = diag([120,120, 120, 200, 200, 200, 1, 1, 5, 1000, 1000, 1000, 0.00001]);
            % Q_close = diag([200,200, 2000, 500, 500, 500, 1, 1, 1, 1000, 1000, 1000]);
            Q_close = diag([100,100, 100, 200, 200, 200, 100, 100, 100, 1000, 1000, 1000, 0.00001]);
%             Q_very_close = diag([1000,1000, 100, 200, 200, 200, 0.1, 0.1, 0.1, 1000, 1000, 1000]);

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

%             Q_far = diag([10,10, 10, 100, 100, 100, 1, 1, 5, 1000, 1000, 1000]);
%             Q_conservative = diag([1000, 1000, 1000, 200, 200, 200, 1, 1, 5, 500, 500, 500]);
%             Q_close = diag([1000,1000, 1000, 500, 500, 500, 1, 1, 5, 500, 500, 500]);
%             Q_captured = diag([10,10, 100, 1000, 1000, 1000, 1, 1, 10, 1000, 1000, 1000]);
           

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
            elseif distance > 1
                Q_curr = Q_far;
                disp(distance + " far")
            elseif distance < 0.6 && ~has_aggresive_rotation
                Q_curr = Q_close;
                disp("close")
            
            % elseif (abs(z_error(1)) < 0.3 || abs(z_error(2)) < 0.3 || abs(z_error(3)) < 0.3) && ~has_aggresive_rotation
            %     % z_error(7:9) = z(7:9)-z_error(1:3);
            %     % disp(z_error(7:9));
            %     % disp("very close");
            %     Q_curr = Q_very_close;
            end
            C = ones(1,12);
            D = [0 0 0 0];
            ud = obj.u0 .* ones(4,1);
            [K, ~, ~] = lqi(ss(obj.A,obj.B,C,D),obj.Q,obj.R,0);

%             [K, ~, ~] = lqr(obj.A, obj.B, Q_curr, obj.R);

            % u = -K * (z - [[-4.8;1;3]; zeros(9, 1)]);
            % u = -K * (z - [y; zeros(9, 1)]);
            target = [y; zeros(9, 1)];
            u = ud + K(:,1:12)*(target-z) +  K(:,13) * [z(3)];
            
            % if iscaptured || ~is_inside_nest
            %     % move to certain position
            %     disp("outside of the nest");
            %     u = -K * (z - [[0;0;1]; zeros(9, 1)]); % + obj.u0;
            % else
            %     % catch uav
            %     u = -K * (z - [y; zeros(9, 1)]); % + obj.u0;
            % end
            

>>>>>>> Stashed changes
        end
    end
end