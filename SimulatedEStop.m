function SimEStop()
    while true
        fprintf('Simulation is paused. Press any key to resume or type ''restart'' to start from the beginning: ');
        userInput = input('','s');

        if strcmpi(userInput, 'restart')
            % Ask for confirmation
            confirmInput = input('Are you sure you want to restart? (y/n): ', 's');

            if strcmpi(confirmInput, 'y')
                fprintf('Code is restarting...\n'); 
                close all; 
                clear;
                run('IRBscript.m');
                return;
            else
                fprintf('Restart cancelled.\n');
            end

        else
            % Ask for confirmation
            confirmInput = input('Are you sure you want to resume? (y/n): ', 's');

            if strcmpi(confirmInput, 'y')
                fprintf('Resuming code...\n');
                clc;
                return;
            else
                fprintf('Resume cancelled.\n');
            end
        end
    end
end
