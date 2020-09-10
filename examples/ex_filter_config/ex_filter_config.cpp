#include<iostream>
#include<filter/filter_config.h>

int main(){
    const char * path = "config/filter_config.json";
    // const char * path = "../../config/filter_config.json";

    // parse_filter_config(path);

    iav::state_predictor::filter::FilterConfig2D my_filter_config(path);
    my_filter_config.print();
    return 1;
}