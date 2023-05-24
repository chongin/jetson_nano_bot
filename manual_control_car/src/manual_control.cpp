#include <ros/ros.h>
#include <std_msgs/String.h>
#include <ncurses.h>


std::map<char, std::string> key_mapping = {
    {KEY_UP, "up"},
    {KEY_DOWN, "down"},
    {KEY_LEFT, "left"},
    {KEY_RIGHT, "right"},
    {119, "speed+"}, //w
    {115, "speed-"} //s
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manual_control_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("drive_car", 10);

    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    refresh();
    while (ros::ok())
    {
        int key = getch();
        if (key != -1) {
            if (key_mapping.count(key) > 0) {
                std_msgs::String msg;
                msg.data = key_mapping[key];
                pub.publish(msg);
            }
            else if (key == 3) {
                std::cout << "Ctrl-C exit." << std::endl; 
                break;
            }
            else {
                std_msgs::String msg;
                msg.data = "stop";
                pub.publish(msg);
            }
        }
       
        ros::spinOnce();
    }

    return 0;
}