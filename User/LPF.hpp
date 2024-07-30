#pragma once

class LPF {
   private:
    float alpha_ = 0;
    float output_prev_ = 0;

   public:
    LPF(float alpha);

    float Compute(float input);
    void ClearOutputPrev();
};