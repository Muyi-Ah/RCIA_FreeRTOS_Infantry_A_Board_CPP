#pragma once

class RotateFeedForward {
   public:
    RotateFeedForward(float C) : C_(C){};
    float Compute() const;

    float C_;
    float vw = 0;
};