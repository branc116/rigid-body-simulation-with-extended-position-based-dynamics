#include "AddShit.hpp"
#include "Core.hpp"
void print(v_t<3> v) {
  std::cout << '(' << v[0] << ", " << v[1] << ", " << v[2] << ')' << '\n';
}
int main(int, char **) { 
  Config c;
  c.callBacks.push_back([](Config& c) {
    print(c.bodies[0].x);
    print(c.bodies[1].x);
    std::cout << '\n';
  });
  Setup0g(c);
  AddRectSpring(c);
  loop(c);
}
