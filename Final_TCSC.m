% Get inputs from the user
prompt = {'The sending voltage (Vs) for the transmission line in kV:' , 'The needed power transfer capability in MW: ', ...
    'The line impedance of the transmission line in ohms: ', 'The series capacitor impedance per TCSC stage in ohms: ', 'The parallel inductive reactance per TCSC stage in ohms'};
dlgtitle = 'Enter the following';
fieldsize = [1 70 ; 1 70 ; 1 70 ; 1 70 ; 1 70];
inputs = inputdlg(prompt,dlgtitle,fieldsize);

Vs = str2double(inputs{1}) * 1e3;   %Sending voltage in volts
P_opt = str2double(inputs{2});      %Needed power capability in MW
X_line = str2double(inputs{3});     %Line impedance in ohm
Xc = str2double(inputs{4});         %Capacitive reactance in ohm 
Xl_stage = str2double(inputs{5});   %Inductive reactance in ohm

C_TCSC = 1 / (2*pi*60*Xc);
L_TCSC = Xl_stage / (2*pi*60);
L_line = X_line / (2*pi*60);

% Constants
R_load = 1000;         
num_stages = 4; % Number of stages for TCSC
Xtcsc_stage = (-1j *Xc * 1j*Xl_stage) / (-1j*Xc + 1j*Xl_stage);  %The equivalent impedance for the parallel branch when the stage is on

%To store needed values for each stage
Vr_values = zeros(1, num_stages);           % Array to store receiving voltages
P_calculated = zeros(1, num_stages);        % Array to store calculated power
voltage_regulation = zeros(1, num_stages);  % Array to store voltage regulation
Xeff_values = zeros(1, num_stages);         % Array to store X effective (magnitude)
Z_eff_values = zeros(1, num_stages);        % Array to store effective impedance (complex)
delta_values = zeros(1, num_stages);        % Array to store power angle (degree)
P_diff=zeros(1, num_stages);                % Array to store power capability difference


% Iterating through the number of TCSC stages
for n = 1:num_stages
    % Calculate the effective impedance in each iteration
    Z_eff = X_line*1j - 1j*(4-n)*Xc + n* Xtcsc_stage;
    Z_eff_values(n) = Z_eff;            %Store the effective impedance (complex)
    Xeff=  X_line - (4-n)*Xc + n* abs(Xtcsc_stage);                  % Calculate effective reactance
    Xeff_values(n) = Xeff;              %Store the effective reactance (magnitude)

    % Calculate the receiving voltage in each iteration
    Vr = Vs * (R_load / (R_load + Z_eff));
    Vr_values(n) = abs(Vr) / 1e3 ;           % Store receiving voltage in kV

    % Calculate the power angle in each iteration
    delta = rad2deg(-angle (Vr));
    delta_values(n) = delta;            %Store the power angle in degree

    % Calculate voltage regulation in each iteration
    voltage_regulation(n) = ((Vs - abs(Vr)) / abs(Vr)) * 100;    % Store voltage regulation

    % Calculate new power P_calculated & Power capability difference in each iteration
    P_calculated(n) = (Vs * abs(Vr) / Xeff) * sind(delta) / 1e6; % Store calculated power
    P_diff = P_opt - P_calculated;
end

% Finding the best combination
valid_indices = find(voltage_regulation < 10);              % To ensure the voltage regulation is below 10%
% In case no voltage regulation is in the needed range
if isempty(valid_indices)
    disp('No configuration meets the criteria of more than 10% voltage regulation.');
else 
    % Finding the minimum difference between calculated Power capabibility and the needed one
    [min_p_diff, index] = min(abs(P_diff(valid_indices)));      % The best number of stages is stored in "index"
    if (min_p_diff < 0.15 * P_opt)                              % To ensure that the power capability is close to the needed
        best_index = valid_indices(index); 
        fprintf('Best Configuration: %d stages\n', best_index); 
        fprintf('Highest Power Capability: %.2f MW\n',P_opt); 
        fprintf('Achieved Power Capability: %.2f MW\n',P_calculated(best_index)); 
        fprintf('Voltage Regulation: %.2f%%\n', voltage_regulation(best_index));
    else
        disp('No combination can achieve a power capability equal or up to 15% of the needed');
    end
end

% Store steps based on the used stages to turn on the needed TCSC stages
if best_index == 1
    final_step = [1,0,0,0];
elseif best_index == 2 
    final_step = [1,1,0,0];
elseif best_index == 3
    final_step = [1,1,1,0];
elseif best_index == 4
    final_step = [1,1,1,1];
end

% Export to Base Workspace to assign the values to Simulink model
assignin('base', 'L_line', L_line);
assignin('base', 'C_TCSC', C_TCSC);
assignin('base', 'L_TCSC', L_TCSC);
assignin('base', 'Vs', Vs);
assignin('base', 'R_load', R_load);
assignin('base', 'final_step', final_step);