#pragma once

class TD {
   public:
    TD(float r, float h) : r_(r), h_(h){};

    float x1_ = 0;
    float x2_ = 0;
    float u_ = 0;
    float r_ = 0;
    float h_ = 0;

    float Compute(float u);
};
