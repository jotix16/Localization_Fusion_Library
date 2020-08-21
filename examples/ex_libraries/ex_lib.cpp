#include<rapidjson/document.h>
#include<Eigen/Dense>
#include<iostream>

using namespace rapidjson;
int main()
{
    Eigen::MatrixXd m(2,2);
    std::cout<< m<<"\n";

    //Parse a JSON string into doc.
    const char* json = "{\"project\":\"rapidjson\",\"stars\":10}";
    Document d;
    d.Parse(json);

    std::cout<<d["project"].GetString();
}