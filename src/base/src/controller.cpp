#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <raylib.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "raylib_controller");
    ros::NodeHandle nh;
    
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    
    InitWindow(800, 600, "ROS Robot Controller");
    SetTargetFPS(60);
    
    while (!WindowShouldClose() && ros::ok()) {
        geometry_msgs::Twist cmd;
        
        if (IsKeyDown(KEY_W)) cmd.linear.x = 0.5;  // Forward
        if (IsKeyDown(KEY_S)) cmd.linear.x = -0.5; // Backward
        if (IsKeyDown(KEY_A)) cmd.angular.z = 0.5; // Left
        if (IsKeyDown(KEY_D)) cmd.angular.z = -0.5; // Right
        
        cmd_pub.publish(cmd);
        
        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawText("WASD Keys to Control Robot", 10, 10, 20, BLACK);
        DrawText(TextFormat("Linear: %.1f Angular: %.1f", cmd.linear.x, cmd.angular.z), 10, 40, 20, BLACK);
        EndDrawing();
        
        ros::spinOnce();
    }
    
    CloseWindow();
    return 0;
}
