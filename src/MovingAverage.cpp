//
//  MovingAverage.cpp
//  movingAverageFilter
//
//  Created by Grantland Dickson on 6/11/2020.
//

#include "MovingAverage.hpp"
#include <iostream>
#include <stdio.h>



MovingAverage::MovingAverage(unsigned int inputSize){ //constructor
    runningTotal = 0.0;
    windowSize = inputSize;
}// MovingAverage

double MovingAverage::next(int inputValue){
    
    /*check if buffer is full*/
    if(buffer.size() == windowSize)
    {
        // Subtract front value
        runningTotal -= buffer.front();
        // Delete value from front of queue
        buffer.pop();
    }
    
    //add new value
    buffer.push(inputValue);
    
    //update running total
    runningTotal += inputValue;
    
    //calculate average
    return static_cast<double>(runningTotal/ buffer.size());
}
