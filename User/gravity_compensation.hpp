#pragma once

class EmpiricalGravityCompensator {
   public:
    float C_;
    EmpiricalGravityCompensator(float C) { C_ = C; };
    float Compute(float theta) const;

   private:
};
