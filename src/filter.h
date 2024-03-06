#ifndef FILTER_H
#define FILTER_H

#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

class FilterMean {
 private:
  int _Nx;
  std::vector<std::vector<double>> _x_queue;

 public:
  FilterMean(double period, double dt) { _Nx = static_cast<int>(period / dt); }

  std::vector<double> filter(std::vector<double> q) {
    if (q.size() != 6) {
      throw std::invalid_argument("q should be size 6");
    }
    if (_x_queue.size() == _Nx) {
      _x_queue.erase(_x_queue.begin());
    }

    if (!_x_queue.empty() && std::abs(q[5] - _x_queue[0][5]) > 1.5 * M_PI) {
      handle_modulo(q[5] - _x_queue[0][5] > 0);
    }

    _x_queue.push_back(q);

    std::vector<double> mean(6, 0.0);
    for (const auto& element : _x_queue) {
      std::transform(mean.begin(), mean.end(), element.begin(), mean.begin(),
                     std::plus<double>());
    }
    std::transform(mean.begin(), mean.end(), mean.begin(),
                   [this](double val) { return val / _x_queue.size(); });

    return mean;
  }

  void handle_modulo(bool dir) {
    for (auto& x : _x_queue) {
      if (dir) {
        x[5] += 2 * M_PI;
      } else {
        x[5] -= 2 * M_PI;
      }
    }
  }
};

class Filter {
 private:
  int _nb, _na;
  std::vector<std::vector<double>> _x_queue, _y_queue;
  bool _is_initialized;
  bool _is_reset;
  std::vector<std::vector<double>> _b, _a;

 public:
  Filter(std::vector<double> cutoff, double fs, int order) {
    if (order > 1){
      throw std::runtime_error("Butterworth filter order N > 1 not implemented.");
    }
    for (int k = 0; k < 6; ++k) {
      auto [b, a] = butter_lowpass(cutoff[k], fs, order);
      _b.push_back(b);
      _a.push_back(a);
    }
    _nb = _b[0].size();
    _na = _a[0].size();
    _is_initialized = false;
    _is_reset = true;
  }

  void reset(){
    // Otherwise vectors point to nullptr.
    if (_is_initialized){
      _x_queue.clear();
      _y_queue.clear();
      _is_reset = false;
    }
  }

  std::pair<std::vector<double>, std::vector<double>> butter_lowpass(
      double cutoff, double fs, int order = 1) {
    double nyq = 0.5 * fs;
    double normal_cutoff = cutoff / nyq;

    // Butterworth coefficients calculation
    // Your code for butterworth coefficients calculation here

    // Find a better way.
    // Low pass filter :
    std::vector<double> b(order + 1), a(order + 1);
    b[0] = (2 * M_PI * (1/fs) * cutoff) / (2 * M_PI * (1/fs) * cutoff + 1.0);
    b[1] =  0.;
    a[0] = 1.0;
    a[1] = -(1.0 - b[0]);

    return std::make_pair(b, a);
  }

  std::array<double, 6> filter(const std::array<double, 18>& q) {
    // Extract the first 6 elements from qvel
    std::vector<double> q_tmp;
    for (int i = 0; i < 6; ++i) {
        q_tmp.push_back(q[i]);
    }
    // Perform filtering on qvel
    std::vector<double> r_vec = _filter(q_tmp);

    // Convert filtered qvel back to std::array<double, 18>
    std::array<double, 6> result;
    for (int i = 0; i < 6; ++i) {
        result[i] = r_vec.at(i);
    }

    // Return the result
    return result;
  }


  std::vector<double> _filter(std::vector<double> q) {
    if (!_is_initialized) {
      _x_queue = std::vector<std::vector<double>>(_nb, q);
      _y_queue = std::vector<std::vector<double>>(_na - 1, q);
      _is_initialized = true;
    }

    if (!_is_reset) {
      for (size_t i = 0; i < _nb; ++i) {
        _x_queue.push_back(q);
      }
      for (size_t i = 0; i < _na; ++i) {
        _y_queue.push_back(q);
      }
      _is_reset = true;
    }

    if (std::abs(q[5] - _y_queue[0][5]) > 1.5 * M_PI) {
      handle_modulo(q[5] - _y_queue[0][5] > 0);
    }

    _x_queue.pop_back();
    _x_queue.insert(_x_queue.begin(), q);

    std::vector<double> acc(6, 0.0);
    for (int i = 0; i < _nb; ++i) {
      for (int j = 0; j < 6; ++j) {
        acc[j] += _b[j][i] * _x_queue[i][j];
      }
    }
    for (int i = 0; i < _na - 1; ++i) {
      for (int j = 0; j < 6; ++j) {
        acc[j] -= _a[j][i + 1] * _y_queue[i][j];
      }
    }
    _y_queue.pop_back();
    _y_queue.insert(_y_queue.begin(), acc);
    return _y_queue[0];
  }

  void handle_modulo(bool dir) {
    for (auto& x : _x_queue) {
      if (dir) {
        x[5] += 2 * M_PI;
      } else {
        x[5] -= 2 * M_PI;
      }
    }

    for (auto& y : _y_queue) {
      if (dir) {
        y[5] += 2 * M_PI;
      } else {
        y[5] -= 2 * M_PI;
      }
    }
  }
};

#endif  // FILTER_H
