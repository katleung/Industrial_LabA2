% Create an Arduino object
% Replace 'COM4' with the port to which your Arduino is connected
arduinoObj = arduino('COM5', 'Uno');

% Define the pin where the button is connected
buttonPin = 'D2';

% Configure the pin as an input
configurePin(arduinoObj, buttonPin, 'DigitalInput');

% Continuously read the button state
while true
    buttonState = readDigitalPin(arduinoObj, buttonPin);
    
    % Check if button is pressed (assuming active low configuration)
    if buttonState == 0
        disp('Button Pressed');
        
        % Debounce delay
        pause(0.5);
    end
end

% Clean up
clear arduinoObj;
