

#include "average_moving_filter.h"



average_moving_filter::average_moving_filter(int  n){
    window_length = n;
    i = 0;
    current_avg = 0;

}


void average_moving_filter::fill_stack(double y){

    measurements.push(y);
    current_avg += y;
    if (i == window_length-1){
        current_avg /= window_length;
    }
    
}

double average_moving_filter::predict(double y){

    if (i < window_length){
        fill_stack(y);
        i++;
        return y;
    }else{
        current_avg += (y - measurements.front())/(double)window_length;
        measurements.pop();
        measurements.push(y);
        return current_avg;
    }
}
