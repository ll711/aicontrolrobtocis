#pragma once
#include <stdint.h>
// Output: requested right PWM minus requested left PWM.
// eCW: shortest signed clockwise angle in radians; prevL/prevR: signed PWM.
// signal=0 must be handled by the caller as an immediate motor stop.
// Features: eCW, signal, prevR-prevL; equal wheel speed is common-mode.
// Prediction uses the synthetic teacher's left/right mirror symmetry.
// Retrained 2026-10-19; synthetic teacher, seed 42, 6144 samples; wheel-difference features and mirror symmetry; not robot measurements.
namespace Tree2Model {
inline float rawPredict(float eCW, int8_t signal, float previousDifference) {
  const float x[3] = {eCW, float(signal), previousDifference};
  const float z[4] = {
    1.0f, x[0], x[1],
    x[2]/255.0f};
  if (double(x[2]) <= 8.5) {
    if (double(x[0]) <= -0.31914682686328888) {
      if (double(x[2]) <= -73.5) {
        if (double(x[2]) <= -123.5) {
          return (15.6208368f * z[0]) + (-12.7120013f * z[1]) + (-0.371098682f * z[2]) + (161.450161f * z[3]);
        } else {
          return (19.5561974f * z[0]) + (-12.0961213f * z[1]) + (0.00264069729f * z[2]) + (165.908163f * z[3]);
        }
      } else {
        if (double(x[2]) <= -27.5) {
          return (17.1991317f * z[0]) + (-12.7074034f * z[1]) + (0.124305326f * z[2]) + (158.637103f * z[3]);
        } else {
          return (17.8286972f * z[0]) + (-12.3914122f * z[1]) + (-0.0102480822f * z[2]) + (146.57933f * z[3]);
        }
      }
    } else {
      if (double(x[2]) <= -58.5) {
        if (double(x[0]) <= 0.8830629289150238) {
          return (-4.00470432f * z[0]) + (-30.42939f * z[1]) + (0.472779252f * z[2]) + (155.724667f * z[3]);
        } else {
          return (-61.2329394f * z[0]) + (-4.68279387f * z[1]) + (3.72199931f * z[2]) + (87.5189992f * z[3]);
        }
      } else {
        if (double(x[0]) <= 0.82410550117492676) {
          return (-0.0870039502f * z[0]) + (-31.4814813f * z[1]) + (-0.00294013595f * z[2]) + (164.817645f * z[3]);
        } else {
          return (-31.5869623f * z[0]) + (-7.15997295f * z[1]) + (-0.22173425f * z[2]) + (160.858414f * z[3]);
        }
      }
    }
  } else {
    if (double(x[0]) <= 0.40681156516075134) {
      if (double(x[2]) <= 67.5) {
        if (double(x[0]) <= -0.83660966157913208) {
          return (31.0205994f * z[0]) + (-7.24748194f * z[1]) + (0.210030556f * z[2]) + (165.309189f * z[3]);
        } else {
          return (0.184553401f * z[0]) + (-31.4840078f * z[1]) + (-0.000148182338f * z[2]) + (164.467386f * z[3]);
        }
      } else {
        if (double(x[0]) <= -0.5196928083896637) {
          return (59.3226918f * z[0]) + (-7.08000446f * z[1]) + (-3.86259299f * z[2]) + (82.6197206f * z[3]);
        } else {
          return (3.15727599f * z[0]) + (-30.5444501f * z[1]) + (-0.41724737f * z[2]) + (157.617651f * z[3]);
        }
      }
    } else {
      if (double(x[2]) <= 78.5) {
        if (double(x[2]) <= 42.5) {
          return (-23.4817927f * z[0]) + (-10.9199841f * z[1]) + (0.00767939555f * z[2]) + (176.897018f * z[3]);
        } else {
          return (-18.9302801f * z[0]) + (-11.2181024f * z[1]) + (-0.421179329f * z[2]) + (156.564305f * z[3]);
        }
      } else {
        if (double(x[2]) <= 125.5) {
          return (-22.2697239f * z[0]) + (-11.1590104f * z[1]) + (0.582386771f * z[2]) + (167.984f * z[3]);
        } else {
          return (-20.1006461f * z[0]) + (-12.1490819f * z[1]) + (-0.0410306134f * z[2]) + (165.540397f * z[3]);
        }
      }
    }
  }
}
inline float predict(float eCW, int8_t signal, int16_t prevL, int16_t prevR) {
  const float difference = float(prevR) - float(prevL);
  // Mirror without wrapping +/-pi; preserve opposite turn choices.
  return 0.5f * (rawPredict(eCW, signal, difference)
               - rawPredict(-eCW, signal, -difference));
}
} // namespace Tree2Model
