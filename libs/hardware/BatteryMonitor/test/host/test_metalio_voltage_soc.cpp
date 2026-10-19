#include <cstdio>
#include <cstdlib>

#include "../../src/MetalioVoltageSoc.h"

namespace {
int g_failures = 0;

void expectEq(const char* name, const int actual, const int expected) {
  if (actual == expected) return;
  std::fprintf(stderr, "FAIL %s: got %d, expected %d\n", name, actual, expected);
  ++g_failures;
}

MetalioVoltageSocSample sample(const uint16_t mv, const int16_t ma, const bool known, const uint32_t now) {
  MetalioVoltageSocSample s;
  s.millivolts = mv;
  s.currentMa = ma;
  s.currentKnown = known;
  s.nowMs = now;
  return s;
}
}  // namespace

int main() {
  expectEq("empty", metalioVoltagePercent(3300), 0);
  expectEq("full-map", metalioVoltagePercent(4330), 100);
  expectEq("mid", metalioVoltagePercent(3815), 50);
  expectEq("low", metalioVoltagePercent(3500), 19);

  {
    MetalioVoltageSocState state;
    expectEq("anchor", metalioVoltageSocUpdate(state, sample(3815, 0, true, 0)), 50);
    expectEq("charge-no-jump", metalioVoltageSocUpdate(state, sample(4200, 200, true, 1000)), 50);
    expectEq("charge-one-step", metalioVoltageSocUpdate(state, sample(4200, 200, true, 31000)), 51);
  }

  {
    MetalioVoltageSocState state;
    expectEq("sag-anchor", metalioVoltageSocUpdate(state, sample(4021, -30, true, 0)), 70);
    expectEq("sag-held", metalioVoltageSocUpdate(state, sample(3600, -100, true, 2000)), 70);
    expectEq("long-discharge-capped", metalioVoltageSocUpdate(state, sample(3500, -50, true, 600000)), 50);
  }

  {
    MetalioVoltageSocState state;
    expectEq("full-wait", metalioVoltageSocUpdate(state, sample(4400, 100, true, 1000)), 99);
    expectEq("full-confirm", metalioVoltageSocUpdate(state, sample(4400, 100, true, 31000)), 100);
    expectEq("full-hold", metalioVoltageSocUpdate(state, sample(4300, 0, true, 90000)), 100);
    expectEq("full-exit-one", metalioVoltageSocUpdate(state, sample(4200, -20, true, 120000)), 99);
  }

  {
    MetalioVoltageSocState state;
    expectEq("flip-anchor", metalioVoltageSocUpdate(state, sample(3815, 100, true, 0)), 50);
    expectEq("flip-reset", metalioVoltageSocUpdate(state, sample(3600, -100, true, 25000)), 50);
    expectEq("flip-wait", metalioVoltageSocUpdate(state, sample(3600, -100, true, 50000)), 50);
    expectEq("flip-step", metalioVoltageSocUpdate(state, sample(3600, -100, true, 56000)), 49);
  }

  {
    MetalioVoltageSocState state;
    expectEq("idle-anchor", metalioVoltageSocUpdate(state, sample(3815, -20, true, 0)), 50);
    expectEq("idle-no-rebound", metalioVoltageSocUpdate(state, sample(4000, 0, true, 60000)), 50);
  }

  if (g_failures != 0) {
    std::fprintf(stderr, "%d failure(s)\n", g_failures);
    return 1;
  }
  std::puts("ok");
  return 0;
}
