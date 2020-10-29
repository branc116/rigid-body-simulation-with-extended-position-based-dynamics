// https://en.wikipedia.org/wiki/List_of_moments_of_inertia
#include "Types.hpp"
constexpr double oneOver12 = 1.0 / 12.0;

m_t<3, 3> IRect(v_t<3> size, double mass) {
  auto sq = size * size;
  return m_t<3, 3>{{oneOver12 * mass * (sq[1] + sq[2]), 0.0, 0.0},
                   {0.0, oneOver12 * mass * (sq[0] + sq[2]), 0.0},
                   {0.0, 0.0, oneOver12 * mass * (sq[0] + sq[1])}};
}
m_t<3, 3> ICilinder(double r, double h, double mass) {
  double rs = r * r, hs = h * h;
  return m_t<3, 3>{{oneOver12 * mass * (3 * rs + hs), 0.0, 0.0},
                   {0.0, 0.5 * mass * rs, 0.0},
                   {0.0, 0.0, oneOver12 * mass * (3 * rs + hs)}};
}