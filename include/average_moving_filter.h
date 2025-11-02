#ifndef _average_moving_filter_H_
#define _average_moving_filter_H_

#include <iostream>
#include <queue>
using namespace std;



class average_moving_filter
{
private:
    // State transition matrix
    queue <double> measurements;
    int i;
    int window_length;
public:

    double current_avg;
    /*
        * @brief Constructor for class average_moving_filter: assigns the parameters to the private variables of the class.
        *
        * @param n window length.
    */
    explicit average_moving_filter(int n);
    ~average_moving_filter(){}


    /*
        * @brief Gets the first n measurements and fills stack
        *
    */
    void fill_stack(double);

    /*
        * @brief Computes the average based on the last measure
        *
    */
    double predict(double);



};

#endif