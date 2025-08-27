#include "MovingAverage.h"

MovingAverage::MovingAverage(int movAvgStep)
{
    this->stepSize = movAvgStep;
    this->placeHolderArr = new float[movAvgStep];
    this->indexCounter = 0;
    this->startCalculate = false;
}

MovingAverage::~MovingAverage()
{
    delete[]placeHolderArr;
    placeHolderArr = nullptr;
}

float MovingAverage::calculateAvg(float *placeHolder)
{
    float sum = 0.0f;
    for (int i = 0; i < stepSize; i++)
    {
        sum += placeHolder[i];
    }
    return sum/(float)this->stepSize;
}

float MovingAverage::movingAverage(float data)
{
    float firstSum = 0.0f;

    if (!startCalculate)
    {
        this->placeHolderArr[indexCounter] = data;
        indexCounter++;

        if (indexCounter == stepSize)
        {
            startCalculate = true;
            indexCounter = 0;
            return calculateAvg(this->placeHolderArr);
        }

        for (int i = 0; i < stepSize; i++)
        {
            firstSum += this->placeHolderArr[i];
        }

        return firstSum/(float)indexCounter;
    }
    else
    {
        this->placeHolderArr[indexCounter] = data;
        indexCounter++;

        if (indexCounter == stepSize)
        {
            startCalculate = true;
            indexCounter = 0;
        }
        return calculateAvg(this->placeHolderArr);
    }
}

// #include "MovingAverage.h"

// // Constructor
// MovingAverage::MovingAverage(size_t _maxSize) : maxSize(_maxSize), sum(0.0) {}

// // Add a new value and calculate the average
// float MovingAverage::addValue(float newValue) {
//     // Add the new value to the queue and update the sum
//     window.push(newValue);
//     sum += newValue;

//     // If the window exceeds the maximum size, remove the oldest value
//     if (window.size() > maxSize) {
//         sum -= window.front();
//         window.pop();
//     }

//     // Return the current average
//     return getAverage();
// }

// // Reset the moving average
// void MovingAverage::reset() {
//     while (!window.empty()) {
//         window.pop();
//     }
//     sum = 0.0;
// }

// // Get the current moving average
// float MovingAverage::getAverage() const {
//     return window.empty() ? 0.0 : sum / window.size();
// }

// // Check if the moving average has enough values to be valid
// bool MovingAverage::isReady() const {
//     return window.size() == maxSize;
// }
