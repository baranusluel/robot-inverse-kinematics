disp('Inverse Kinematics Calculator by Baran Usluel');
disp('For 6DOF Manipulators with Euler Wrists');

px = 0.2588190451; py = 0.2588190451; pz = 2.366025404;
d1 = 1; d2 = 1; d3 = 1;
r = eye(3);

tol = 0.0001; % Tolerance for difference between desired and calculated target coordinates

%syms px py pz d1 d2 d3;
%r = sym('r%d%d',3);
%syms Q1 Q2 Q3 Q4 Q5 Q6;
%T = [r(1,1) r(1,2) r(1,3) px; r(2,1) r(2,2) r(2,3) py; r(3,1) r(3,2) r(3,3) pz; 0 0 0 1];

% TRANSFORMATION MATRICES
%{
T1 = [cos(Q1) -sin(Q1) 0 0; sin(Q1) cos(Q1) 0 0; 0 0 1 0; 0 0 0 1];
T2 = [cos(Q2) -sin(Q2) 0 0; 0 0 -1 0; sin(Q2) cos(Q2) 0 d1; 0 0 0 1];
T3 = [cos(Q3) -sin(Q3) 0 0; sin(Q3) cos(Q3) 0 d2; 0 0 1 0; 0 0 0 1];
T4 = [cos(Q4) -sin(Q4) 0 0; 0 0 1 d3; -sin(Q4) -cos(Q4) 0 0; 0 0 0 1];
T5 = [cos(Q5) -sin(Q5) 0 0; 0 0 -1 0; sin(Q5) cos(Q5) 0 0; 0 0 0 1];
T6 = [cos(Q6) -sin(Q6) 0 0; 0 0 1 0; -sin(Q6) -cos(Q6) 0 0; 0 0 0 1];
%}


% FORWARD KINEMATICS
% --------------------
%{
T = T1*T2*T3*T4*T5*T6;
disp('px: '); disp(T(1,4));
disp('py: '); disp(T(2,4));
disp('pz: '); disp(T(3,4));
%}


% INVERSE KINEMATICS
% --------------------

% SYMBOLIC EQUATIONS
%{
LHS = (T1*T2)\T; % Equivalent of inv(T1*T2)*T
RHS = T3*T4*T5*T6;
disp('LHS px: '); disp(LHS(1,4));
disp('LHS py: '); disp(LHS(2,4));
disp('LHS pz: '); disp(LHS(3,4));
disp('-------');
disp('RHS px: '); disp(RHS(1,4));
disp('RHS py: '); disp(RHS(2,4));
disp('RHS pz: '); disp(RHS(3,4));
%}


% ANGLE CALCULATIONS
Q4 = 0;
Q5 = 0;
Q6 = 0;

Q1 = [atan2(py, px), atan2(-py, -px)];
for i = 1 : 2,
    v = pz - d1;
    w = px * cos(Q1(i)) + py * sin(Q1(i));

    tmp = (v^2 + w^2 - d2^2 - d3^2)/(2*d2*d3);
    Q3 = [atan2(sqrt(1 - tmp^2), tmp), atan2(-sqrt(1 - tmp^2), tmp)];

    for j = 1 : 2,
        tmp = -d3*sin(Q3(j));
        Q2 = [(atan2(v, w) + atan2(sqrt(v^2 + w^2 - tmp^2), tmp)) (atan2(v, w) - atan2(sqrt(v^2 + w^2 - tmp^2), tmp))];

        for k = 1 : 2,
            % VERIFY RESULTS
            T1 = [cos(Q1(i)) -sin(Q1(i)) 0 0; sin(Q1(i)) cos(Q1(i)) 0 0; 0 0 1 0; 0 0 0 1];
            T2 = [cos(Q2(k)) -sin(Q2(k)) 0 0; 0 0 -1 0; sin(Q2(k)) cos(Q2(k)) 0 d1; 0 0 0 1];
            T3 = [cos(Q3(j)) -sin(Q3(j)) 0 0; sin(Q3(j)) cos(Q3(j)) 0 d2; 0 0 1 0; 0 0 0 1];
            T4 = [cos(Q4) -sin(Q4) 0 0; 0 0 1 d3; -sin(Q4) -cos(Q4) 0 0; 0 0 0 1];
            T5 = [cos(Q5) -sin(Q5) 0 0; 0 0 -1 0; sin(Q5) cos(Q5) 0 0; 0 0 0 1];
            T6 = [cos(Q6) -sin(Q6) 0 0; 0 0 1 0; -sin(Q6) -cos(Q6) 0 0; 0 0 0 1];
            T = T1*T2*T3*T4*T5*T6;
            if abs(T(1,4) - px) < tol && abs(T(2,4) - py) < tol && abs(T(3,4) - pz) < tol
                if abs(Q1(i)) <= pi && abs(Q2(k)) <= pi/2 && abs(Q3(j)) < 3*pi/4
                    disp('--------');
                    disp('Q1: ');
                    disp(Q1(i)/pi*180);

                    disp('Q2: ');
                    disp(Q2(k)/pi*180);

                    disp('Q3: ');
                    disp(Q3(j)/pi*180);
                    
                    disp(T);
                end
            end

        end
    end
end