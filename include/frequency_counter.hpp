#pragma once

#include <chrono>
#include <vector>

struct FrequencyCounter
{
  std::vector<std::chrono::system_clock::time_point> frames_ts;
  
  void tic()
  {
    frames_ts.push_back( std::chrono::system_clock::now() );
    while( frames_ts.size() > 100 ){
      frames_ts.erase( frames_ts.begin() );
    }
  }

  double fps()
  {
    double fps = 0;
    const size_t &size = frames_ts.size();
    if( size > 2 ){
      auto milliseconds = std::chrono::duration_cast<std::chrono::microseconds>( frames_ts.back() - frames_ts.front() ).count();
      fps = 1000000.0 * static_cast<double>(size) / milliseconds;
    }
    return fps;
  }
};

