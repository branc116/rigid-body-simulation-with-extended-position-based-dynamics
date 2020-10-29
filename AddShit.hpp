#include "ITensors.hpp"
#include "Types.hpp"

int AddRect(Config &c, v_t<3> pos, v_t<3> size, double mass) {
  c.bodies.push_back(Body{IRect(size, mass), pos, v_t<3>(), v_t<3>(), v_t<4>(),
                          v_t<4>(), v_t<3>(), mass});
  return c.bodies.size() - 1;
}
int AddWheel(Config &c, v_t<3> pos, double radius, double thickens,
              double mass) {
  c.bodies.push_back(Body{ICilinder(radius, thickens, mass), pos, v_t<3>(),
                          v_t<3>(), v_t<4>(), v_t<4>(), v_t<3>(), mass});
  return c.bodies.size() - 1;
}
int AddPosConstraint(Config &c, int i, int j, v_t<3> ri, v_t<3> rj, v_t<3> dx,
                      double damp) {
  c.posConstraints.push_back(PosConstraint{i, j, ri, rj, dx, damp, 0});
  return c.posConstraints.size() - 1;
}

void AddRectSpring(Config& c) {
  int i = AddRect(c, v_t<3>{0, 0, 0}, v_t<3>{1, 1, 1}, 100);
  int j = AddRect(c, v_t<3>{10, 0, 0}, v_t<3>{1, 1, 1}, 1);
  c.addConstraints.push_back([=](Config& conf) {
    auto r1 = v_t<3>{0, 0, 1};
    auto r2 = v_t<3>{0, 0.2, 0.1};
    auto diff = blaze::normalize(conf.bodies[j].x + r2 - (r1 + conf.bodies[i].x)) * -5;
    AddPosConstraint(conf, i, j, r1, r2, diff, 10);
  });
}
void Setup0g(Config& c) {
  c.fExt = v_t<3>{0.0, 0.0, 0.0};
  c.numPosSteps = 1;
  c.numSubSteps = 1;
  *c.stop = false;
  c.tExt = v_t<3>{0.0, 0.0, 0.0};
  c.dt = 0.16;
}