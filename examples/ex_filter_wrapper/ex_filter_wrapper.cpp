#include<iostream>
#include<filter/filter_wrapper.h>

int main(){
    const char * path = "config/filter_config.json";
    // const char * path = "../../config/filter_config.json";

    // parse_filter_config(path);

    iav::state_predictor::filter::FilterCtrvEKF2D my_filter_wrapper(path);
    return 1;
}