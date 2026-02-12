#include "../include/filterSmooth.h"

FilterSmooth::FilterSmooth(){
    this->cmd_filtered.twist.linear.x = 0;
    this->cmd_filtered.twist.linear.y = 0;
    this->cmd_filtered.twist.linear.z = 0;
    this->cmd_filtered.twist.angular.x = 0;
    this->cmd_filtered.twist.angular.y = 0;
    this->cmd_filtered.twist.angular.z = 0;

    this->factor = 0;
}


void FilterSmooth::update(geometry_msgs::TwistStamped *input){

    this->cmd_filtered.twist.linear.x = this->factor * this->cmd_filtered.twist.linear.x + (1 - this->factor) * input->twist.linear.x;
    this->cmd_filtered.twist.linear.y = this->factor * this->cmd_filtered.twist.linear.y + (1 - this->factor) * input->twist.linear.y;
    this->cmd_filtered.twist.linear.z = this->factor * this->cmd_filtered.twist.linear.z + (1 - this->factor) * input->twist.linear.z;

    this->cmd_filtered.twist.angular.x = this->factor * this->cmd_filtered.twist.angular.x + (1 - this->factor) * input->twist.angular.x;
    this->cmd_filtered.twist.angular.y = this->factor * this->cmd_filtered.twist.angular.y + (1 - this->factor) * input->twist.angular.y;
    this->cmd_filtered.twist.angular.z = this->factor * this->cmd_filtered.twist.angular.z + (1 - this->factor) * input->twist.angular.z;




}
geometry_msgs::TwistStamped FilterSmooth::get(){
    return this->cmd_filtered;
}

float FilterSmooth::get(geometry_msgs::TwistStamped *output){
    output->twist.linear.x = this->cmd_filtered.twist.linear.x;
    output->twist.linear.y = this->cmd_filtered.twist.linear.y;
    output->twist.linear.z = this->cmd_filtered.twist.linear.z;
    output->twist.angular.x = this->cmd_filtered.twist.angular.x;
    output->twist.angular.y = this->cmd_filtered.twist.angular.y;
    output->twist.angular.z = this->cmd_filtered.twist.angular.z;

}
