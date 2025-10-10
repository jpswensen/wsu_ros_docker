%% 

% Create a ROS 2 node
node = ros2node("my_node");

% Define the topic name
topic = "/chatter";

% Define the message type (e.g., example_interfaces/Int32)
msgType = "std_msgs/String";

% Define the callback function
function subCallback(msg)
    % Display the received message data
    disp("Received message:");
    disp(msg.data);
end

% Create a subscriber for the topic with the specified message type and callback
sub = ros2subscriber(node, topic, msgType, @subCallback);

% Keep the node alive to listen for messages
disp("Listening for messages on " + topic + "...");

pause(20);

disp("DONE");