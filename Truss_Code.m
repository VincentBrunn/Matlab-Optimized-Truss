% EK301, Section A4, Group: Moataz S., Avyukta S., Vincent B., 11/12/2024
% Truss Design Project
% Boston University - College of Engineering

load('TrussDesign2','C','X','Y','Sx','Sy','L')
 
% [J, M] = size(C);
% disp(L);
% Construct Equilibrium Equations

A = zeros(2 * J, M + 3); % 16 x 16 matrix
for j = 1:J
    for m = 1:M
        % Identify the two joints connected by member m
        connected_joints = find(C(:, m) == 1);
    
        % Ensure exactly two joints are found
        if numel(connected_joints) ~= 2
        error('Member %d does not connect exactly two joints.', m);
        end
    
        joint1 = connected_joints(1);
        joint2 = connected_joints(2);
    
        % Calculate dx, dy, and r
        dx = X(joint2) - X(joint1);
        dy = Y(joint2) - Y(joint1);
        r = sqrt(dx^2 + dy^2);
    
        % Assign x and y components for joint1
        A(joint1, m) = dx / r;
        A(J + joint1, m) = dy / r;
    
        % Assign x and y components for joint2 (with opposite sign)
        A(joint2, m) = -dx / r;
        A(J + joint2, m) = -dy / r;
    end
end

% Add the support reactions in the A matrix
A(1:J, M+1:M+3) = Sx;           % Assign `Sx` to the first J rows and last 3 columns
A(J+1:2*J, M+1:M+3) = Sy;       % Assign `Sy` to the next J rows and last 3 columns

%disp('Load vector L just before solving:');
%disp(L);
%rank_A = rank(A);
%disp(['Rank of A: ', num2str(rank_A)]);
%disp(['Size of A: ', num2str(size(A, 2))]);
%disp('Matrix A:');
%disp(A);

% Solve for unknown forces
T = A \ L; % Solution vector containing member forces and support reactions

% Find critical member and max load
R_m = T(1:end-3) / max(L);
p_crit = 3054.789 * lengths.^-2.009;
failure_loads = -p_crit / R_m';
failure_loads(failure_loads < 0) = inf;
maxload = min(abs(failure_loads));
crit_member = find(abs(failure_loads) == maxload);

% Calculate cost
total_len = sum(lengths);
joints = size(C, 1);
cost = 10 * joints + total_len;

% Load to cost ratio
load_to_cost_ratio = maxload / cost;

% After solving for critical member and finding the index `crit_member`
connected_joints_crit = find(C(:, crit_member) == 1);
joint1_crit = connected_joints_crit(1);
joint2_crit = connected_joints_crit(2);

% Define joint labels (e.g., 'A', 'B', 'C', etc.)
jointLabels = ['A', 'B', 'C', 'D', 'E', 'F', 'G', 'H'];

% Display results with member names using joint labels
fprintf('\\%%EK301, Section A4, Group: Moataz S., Avyukta S., Vincent B., 11/12/2024\n')
load_value = L(J+4);
fprintf('Load: %.2f N\n', load_value);
fprintf('Member Forces in N:\n');
for m = 1:M
    % Find the joints connected by member m
    connected_joints = find(C(:, m) == 1);
    joint1 = connected_joints(1);
    joint2 = connected_joints(2);
    
    % Determine if the member is in tension or compression
    if T(m) >= 0
        status = 'Tension';
    else
        status = 'Compression';
    end
    
    % Print member information with joint labels
    fprintf('Member %s%s: %.2f N (%s)\n', ...
        jointLabels(joint1), jointLabels(joint2), abs(T(m)), status);
end

% Display support reaction forces (last three elements of T)
fprintf('\nReaction Forces in N:\n');
fprintf('  Sx1: %.2f N\n', T(M+1));
fprintf('  Sy1: %.2f N\n', T(M+2));
fprintf('  Sy2: %.2f N\n', T(M+3));
fprintf("Cost of truss: $%.2f\n", cost);
% Display critical member with joint labels
fprintf("Critical member: %s%s\n", jointLabels(joint1_crit), jointLabels(joint2_crit));
fprintf("Theoretical max load: %.2f\n", maxload);
fprintf("Theoretical max load/cost ratio in N/$: %f\n", load_to_cost_ratio);