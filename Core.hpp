#include "Types.hpp"
// #include "bits/stdc++.h"
#include <iostream>

v_t<4> qInverse(const v_t<4> &q) {
  return q * v_t<4>{1, -1, -1, -1} / blaze::norm(q);
}
m_t<4, 4> to44(m_t<3, 3> m) {
  return m_t<4, 4>{{m(0, 0), m(0, 1), m(0, 2), 0.0},
                   {m(1, 0), m(1, 1), m(1, 2), 0.0},
                   {m(2, 0), m(2, 1), m(2, 2), 0.0},
                   {0.0, 0.0, 0.0, 0.0}};
}
v_t<4> to4(v_t<3> v) { return v_t<4>{v[0], v[1], v[2], 0.0}; }

void CollectCollisionPairs(Config &c) {
    c.angularConstraints.clear();
    c.posConstraints.clear();
    for(auto&& a : c.addConstraints) {
        a(c);
    }
}
void PrePosSolve(Config &c) {
  auto h = c.dt / c.numSubSteps;
  for (auto&& b : c.bodies) {
    auto &el = b;
    el.xPrev = el.x;
    el.v += h * c.fExt / el.m;
    el.x += h * el.v;

    el.qPrev = el.q;
    el.omega +=
        h * el.Iinv() * (c.fExt - (blaze::cross(el.omega, el.I * el.omega)));
    el.q = blaze::normalize(
        el.q +
        h * 0.5 * el.q * v_t<4>{el.omega[0], el.omega[1], el.omega[2], 0});
  }
}
void SolvePositionConstraints(Config &c) {
  auto h = c.dt / c.numSubSteps;
  auto h2 = h * h;
  for (auto &&b : c.posConstraints) {
    auto &eli = c.bodies[b.i];
    auto &elj = c.bodies[b.j];
    auto [n, c] = b.n_c();
    auto wi = 1 / eli.m + blaze::trans(blaze::cross(b.ri, n)) * eli.Iinv() *
                              blaze::cross(b.ri, n);
    auto wj = 1 / elj.m + blaze::trans(blaze::cross(b.rj, n)) * elj.Iinv() *
                              blaze::cross(b.rj, n);
    auto alphac = b.compliance / h2;
    auto dlambda = (-c - alphac * b.lambda) / (wi + wj + alphac);
    // b.lambda += dlambda;
    auto p = dlambda * n;
    eli.x += p / eli.m;
    elj.x -= p / elj.m;
    eli.q += 0.5 * to4(eli.Iinv() * blaze::cross(b.ri, p)) * eli.q;
    elj.q -= 0.5 * to4(elj.Iinv() * blaze::cross(b.rj, p)) * elj.q;
  }
}
void SolveAngularConstraints(Config &c) {
  auto h = c.dt / c.numSubSteps;
  auto h2 = h * h;
  for (auto &&b : c.angularConstraints) {
    auto &eli = c.bodies[b.i];
    auto &elj = c.bodies[b.j];
    auto [n, t] = b.n_t();
    auto wi = blaze::trans(n) * eli.Iinv() * n;
    auto wj = blaze::trans(n) * elj.Iinv() * n;
    auto alphac = b.compliance / h2;
    auto dlambda = (-t - alphac * b.lambda) / (wi + wj + alphac);
    b.lambda += dlambda;
    auto p = dlambda * n;
    eli.q += 0.5 * to4(eli.Iinv() * p) * eli.q;
    elj.q -= 0.5 * to4(elj.Iinv() * p) * elj.q;
  }
}
void PosSolve(Config &c) {
  for (int i = 0; i < c.numPosSteps; i++) {
    SolvePositionConstraints(c);
    SolveAngularConstraints(c);
  }
}
void PreVelSolve(Config &c) {
  auto h = c.dt / c.numSubSteps;
  for (auto&& b : c.bodies) {
    auto &el = b;
    el.v = (el.x - el.xPrev) / h;
    auto dq = el.q * qInverse(el.qPrev);
    el.omega = 2 * v_t<3>{dq[0], dq[1], dq[2]} / h * (dq[3] > 0 ? 1 : -1);
  }
}
void VelSolve(Config &c) {}
void loop(Config &c) {
  while (!*c.stop) {
    CollectCollisionPairs(c);
    for(int i = 0; i< c.numSubSteps; i++) {
      PrePosSolve(c);
      PosSolve(c);
      PreVelSolve(c);
      VelSolve(c);
    }
    for(auto&& cb : c.callBacks) {
        cb(c);
    }
  }
}