#pragma once
#include <utility>
#include <vector>

// #include "bits/stdc++.h"
#include "blaze/Blaze.h"
template <size_t n> using v_t = blaze::StaticVector<double, n>;

template <size_t n, size_t m> using m_t = blaze::StaticMatrix<double, n, m>;

struct Body {
  m_t<3, 3> I;

  v_t<3> x;
  v_t<3> xPrev;
  v_t<3> v;

  v_t<4> q;
  v_t<4> qPrev;
  v_t<3> omega;

  double m;

  m_t<3, 3> Iinv() {
    auto a = I;
    blaze::invert3x3<blaze::InversionFlag::byLU>(a);
    return a;
  }
};
struct PosConstraint {
  int i, j;
  v_t<3> ri, rj;
  v_t<3> dx;
  double compliance;
  double lambda;

  std::tuple<v_t<3>, double> n_c() {
    return {blaze::normalize(dx), blaze::norm(dx)};
  }
};

struct AngularConstraint {
  int i, j;
  v_t<3> ri, rj;
  v_t<3> dq;
  double compliance;
  double lambda;

  std::tuple<v_t<3>, double> n_t() {
    return {blaze::normalize(dq), blaze::norm(dq)};
  }
};

struct Config {
  bool *stop;
  double dt;
  int numSubSteps;
  int numPosSteps;
  v_t<3> fExt;
  v_t<3> tExt;
  std::vector<Body> bodies;
  std::vector<PosConstraint> posConstraints;
  std::vector<AngularConstraint> angularConstraints;
  std::vector<std::function<void(Config&)>> callBacks;
  std::vector<std::function<void(Config&)>> addConstraints;
};
