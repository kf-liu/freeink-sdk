#pragma once

#include <cmath>
#include <cstdint>

// Metalio E-Ink 4 的 BQ27220 没有写入电池曲线，芯片 SOC 寄存器会非线性乱跳。
// 原厂（CloudZao/Metalio-E-INK4，docs/battery-soc-and-charge.md）不用该寄存器，
// 只把电压线性映射成百分比：3300 mV = 0%，4330 mV = 100%。
//
// 平滑和步进按原厂时间常数实现，但采样不假设 1 Hz：
//   * 指数平均时间常数 60 s（原厂是 1 Hz 下 60 点滑动平均）
//   * 充电只升、非充电只降，每经过 30 s 最多 1%
//   * 电压连续 ≥4330 mV 满 30 s 才允许 100%
//   * 已经显示 100% 时，电压跌破 4280 mV 之前保持 100%
// 长时间没有刷新时，按真实经过的 30 s 段补步，补完就把这些时段用掉，
// 避免下一次采样把积压的步数一次放出来。
// 状态放在调用方的静态对象里，这里不分配堆内存。

enum class MetalioChargeDirection : int8_t { Idle, Charging, Discharging };

struct MetalioVoltageSocState {
  bool primed = false;
  bool fullTracking = false;
  float smoothed = 0.f;
  int displayed = -1;
  uint32_t lastSampleMs = 0;
  uint32_t lastStepMs = 0;
  uint32_t fullAboveSinceMs = 0;
  MetalioChargeDirection lastDirection = MetalioChargeDirection::Idle;
};

struct MetalioVoltageSocSample {
  uint16_t millivolts = 0;
  int16_t currentMa = 0;
  bool currentKnown = false;
  uint32_t nowMs = 0;
};

inline MetalioChargeDirection metalioChargeDirection(const bool currentKnown, const int16_t currentMa) {
  if (!currentKnown) return MetalioChargeDirection::Idle;
  if (currentMa > 5) return MetalioChargeDirection::Charging;
  if (currentMa < -5) return MetalioChargeDirection::Discharging;
  return MetalioChargeDirection::Idle;
}

inline float metalioVoltageRawPercent(const uint16_t millivolts) {
  constexpr uint16_t kEmptyMv = 3300;
  constexpr uint16_t kFullMv = 4330;
  if (millivolts >= kFullMv) return 100.f;
  if (millivolts <= kEmptyMv) return 0.f;
  return (static_cast<float>(millivolts - kEmptyMv) * 100.f) / static_cast<float>(kFullMv - kEmptyMv);
}

inline int metalioVoltagePercent(const uint16_t millivolts) {
  return static_cast<int>(metalioVoltageRawPercent(millivolts) + 0.5f);
}

// 返回 0..100 的显示电量。电压读失败不要调用；电流未知按空载（只允许下降）。
inline uint16_t metalioVoltageSocUpdate(MetalioVoltageSocState& state, const MetalioVoltageSocSample& sample) {
  constexpr uint16_t kFullMv = 4330;
  constexpr uint16_t kFullExitMv = 4280;
  constexpr uint32_t kStepMs = 30000;
  constexpr uint32_t kFullHoldMs = 30000;
  constexpr uint32_t kSmoothTauMs = 60000;

  const float raw = metalioVoltageRawPercent(sample.millivolts);
  const uint32_t sampleDt = state.primed ? (sample.nowMs - state.lastSampleMs) : 0;
  if (!state.primed) {
    state.smoothed = raw;
    state.primed = true;
  } else if (sampleDt > 0) {
    const float alpha = 1.f - std::exp(-static_cast<float>(sampleDt) / static_cast<float>(kSmoothTauMs));
    state.smoothed += alpha * (raw - state.smoothed);
  }
  state.lastSampleMs = sample.nowMs;

  int target = static_cast<int>(state.smoothed + 0.5f);
  if (target < 0) target = 0;
  if (target > 100) target = 100;

  bool fullOk = false;
  if (sample.millivolts >= kFullMv) {
    if (!state.fullTracking) {
      state.fullTracking = true;
      state.fullAboveSinceMs = sample.nowMs;
    }
    fullOk = (sample.nowMs - state.fullAboveSinceMs) >= kFullHoldMs;
  } else {
    state.fullTracking = false;
  }
  if (target >= 100 && !fullOk) target = 99;
  if (state.displayed >= 100 && sample.millivolts >= kFullExitMv) target = 100;

  const MetalioChargeDirection direction = metalioChargeDirection(sample.currentKnown, sample.currentMa);
  if (state.displayed < 0) {
    state.displayed = target;
    state.lastStepMs = sample.nowMs;
    state.lastDirection = direction;
  } else {
    // 采样还比较密时，插拔电重开步进计时，避免方向一变就立刻 ±1%。
    // 间隔已经超过一个步进周期，说明中间的翻转时刻未知，按整段时间补步。
    if (direction != state.lastDirection && sampleDt < kStepMs) state.lastStepMs = sample.nowMs;
    state.lastDirection = direction;

    const uint32_t elapsed = sample.nowMs - state.lastStepMs;
    const uint32_t intervals = elapsed / kStepMs;
    if (intervals > 0) {
      const int budget = intervals > 100 ? 100 : static_cast<int>(intervals);
      switch (direction) {
        case MetalioChargeDirection::Charging:
          if (target > state.displayed) {
            const int room = target - state.displayed;
            state.displayed += room < budget ? room : budget;
          }
          break;
        case MetalioChargeDirection::Discharging:
        case MetalioChargeDirection::Idle:
          if (target < state.displayed) {
            const int room = state.displayed - target;
            state.displayed -= room < budget ? room : budget;
          }
          break;
      }
      state.lastStepMs = sample.nowMs - (elapsed % kStepMs);
    }
  }

  if (fullOk && target >= 100 && state.displayed >= 99) state.displayed = 100;
  if (state.displayed < 0) state.displayed = 0;
  if (state.displayed > 100) state.displayed = 100;
  return static_cast<uint16_t>(state.displayed);
}
