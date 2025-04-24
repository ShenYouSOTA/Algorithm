#include <chrono>
#include <iostream>
#include <string>
#include <vector>

#include "reference.h"

using namespace std;

void test_case(string start, string end, int** w) {
  vector<string> path;
  using std::chrono::high_resolution_clock;
  using std::chrono::duration;
  using std::chrono::milliseconds;
  cout << "Find shortest path from " << start << " to " << end << ".\n";
  auto t1 = high_resolution_clock::now();
  int dist = reference::FindShortestPath(start, end, path, w);
  auto t2 = high_resolution_clock::now();
  duration<double, std::milli> ms_double = t2 - t1;
  cout << "Computation took " << ms_double.count() << "ms.\n";
  cout << "Distance: " << dist << "\nPath: ";
  for(const auto& step : path) cout << step << ", ";
  cout << "\n\n";
}

int main() {
  int m = 1000;
  vector<int> weights[4];
  weights[0].resize(m, 10);
  weights[1].resize(m, 10);
  weights[2].resize(m, 1000);
  weights[3].resize(m, 1000);
  weights[0][0] = 1;
  int* w[4] = {weights[0].data(), weights[1].data(), weights[2].data(), weights[3].data()};

  test_case("B3908", "B3928", w);
  test_case("B3901", "B228", w);
  test_case("A3102", "B501", w);
  test_case("A3102", "A3116", w);
  test_case("A3105", "A3105", w);
  test_case("A98405", "B12501", w);
  
  return 0;
}
