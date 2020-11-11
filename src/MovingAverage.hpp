//
//  MovingAverage.hpp
//  movingAverageFilter
//
//  Created by Grantland Dickson on 6/11/2020.
//

#ifndef MovingAverage_hpp
#define MovingAverage_hpp

#include <queue>

class MovingAverage{
    
    double runningTotal;
    unsigned int windowSize;
    std::queue<int> buffer;
    
public:
    
    MovingAverage(unsigned int inputSize); //Constructor
    double next(int inputValue); // Function
};

#endif /* MovingAverage_hpp */
