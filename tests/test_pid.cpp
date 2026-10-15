#include "landing.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
int main() {
 PID p; assert(p.step(0,0.05)==0);
 p.reset();assert(std::abs(p.integral)<1e-10);
 auto a=runTrial(1,0,0,1,0.8,0.02,0.25,nullptr);
 assert(std::isfinite(a.finalError));
 assert(a.finalError<0.2);
 assert(a.dropped==0);
 std::cout<<"PID simulation tests passed\n";
}
