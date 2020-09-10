#include<iostream>
#include "../src/motion_model/include/motion_model.h"

int main(){
    iav::state_predictor::motion_model::MyCtrv mm_;
    mm_.call();

    std::cout<<"MIkel"<<std::endl;

}