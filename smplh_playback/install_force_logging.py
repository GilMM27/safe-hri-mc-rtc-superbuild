"""Install SMPL-H force-limit logging in the external KinovaPositionSSMPFLController."""

import argparse
from pathlib import Path
import shutil


MARKER = "// SMPLH force logging installed by the superbuild"


def replace_once(text, old, new):
    if text.count(old) != 1:
        raise RuntimeError(f"Unsupported KinovaPositionSSMPFLController source: {old[:80]!r}")
    return text.replace(old, new, 1)


def install(source):
    source = Path(source)
    folder = source / "src"
    cpp_path = folder / "KinovaPositionSSMPFLController.cpp"
    header_path = folder / "KinovaPositionSSMPFLController.h"
    cpp, header = cpp_path.read_text(), header_path.read_text()
    old_availability = '''    forceSampleAvailable_ = true;
    regionNormalN_ = forces.positiveNormalSumN;
    regionPeakN_ = forces.peakPositiveNormalSumN;'''
    new_availability = '''    forceSampleAvailable_ = forces.valid;
    if(forceSampleAvailable_)
    {
      regionNormalN_ = forces.positiveNormalSumN;
      regionPeakN_ = forces.peakPositiveNormalSumN;
    }'''
    if MARKER in cpp and old_availability in cpp:
        cpp = replace_once(cpp, old_availability, new_availability)
    if MARKER not in cpp:
        cpp = replace_once(cpp, '#include "SSMPFLMath.h"',
                           '#include "SSMPFLMath.h"\n#include "SmplhContactForces.h"\n' + MARKER)
        cpp = replace_once(cpp, '    regionParameters_[name] = parameters;\n  }',
                           '''    regionParameters_[name] = parameters;
  }
  for(size_t i = 0; i < smplh_contact::regions.size(); ++i)
  {
    regionLimitN_[i] = regionParameters_.at(smplh_contact::regions[i]).force;
    const std::string prefix = std::string("SMPLH_") + smplh_contact::regions[i] + "_";
    logger().addLogEntry(prefix + "positive_normal_sum_N", [this, i]() { return regionNormalN_[i]; });
    logger().addLogEntry(prefix + "positive_normal_peak_N", [this, i]() { return regionPeakN_[i]; });
    logger().addLogEntry(prefix + "ForceMaxN", [this, i]() { return regionLimitN_[i]; });
    logger().addLogEntry(prefix + "exceeded", [this, i]() { return regionExceeded_[i]; });
    logger().addLogEntry(prefix + "peak_exceeded", [this, i]() { return regionPeakN_[i] > regionLimitN_[i]; });
  }
  logger().addLogEntry("SMPLH_force_sample_available", [this]() { return forceSampleAvailable_; });
  logger().addLogEntry("SMPLH_external_joint_torque_norm_Nm", [this]() { return externalJointTorqueNormNm_; });
  logger().addLogEntry("SMPLH_external_torque_feedback_enabled", [this]() { return externalTorqueFeedback_; });
  logger().addLogEntry("SMPLH_high_pass_compensation_enabled", [this]() { return highPassCompensation_; });''')
        cpp = replace_once(cpp, '  speedPlot_ = SpeedPlotSample{};\n}\n\nconst human_robot_distance::Snapshot *',
                           '''  speedPlot_ = SpeedPlotSample{};
  regionNormalN_.fill(0);
  regionPeakN_.fill(0);
  regionExceeded_.fill(false);
  forceSampleAvailable_ = false;
  externalTorqueFeedback_ = false;
  highPassCompensation_ = false;
  externalJointTorqueNormNm_ = 0;
}

void KinovaPositionSSMPFLController::updateContactForces()
{
  regionNormalN_.fill(0);
  forceSampleAvailable_ = false;
  if(datastore().has(smplh_contact::datastoreKey))
  {
    const auto & forces = datastore().get<smplh_contact::Snapshot>(smplh_contact::datastoreKey);
    forceSampleAvailable_ = forces.valid;
    if(forceSampleAvailable_)
    {
      regionNormalN_ = forces.positiveNormalSumN;
      regionPeakN_ = forces.peakPositiveNormalSumN;
    }
    externalTorqueFeedback_ = forces.externalTorqueFeedback;
    highPassCompensation_ = forces.highPassCompensation;
    externalJointTorqueNormNm_ = forces.externalJointTorqueNormNm;
  }
  for(size_t i = 0; i < regionNormalN_.size(); ++i)
    regionExceeded_[i] = forceSampleAvailable_ && regionNormalN_[i] > regionLimitN_[i];
}

const human_robot_distance::Snapshot *''')
        cpp = replace_once(cpp, 'bool KinovaPositionSSMPFLController::run()\n{',
                           'bool KinovaPositionSSMPFLController::run()\n{\n  updateContactForces();')
        header = replace_once(header, '  std::map<std::string, RegionParameters> regionParameters_;',
                              '''  std::map<std::string, RegionParameters> regionParameters_;
  std::array<double, 14> regionNormalN_{};
  std::array<double, 14> regionPeakN_{};
  std::array<double, 14> regionLimitN_{};
  std::array<bool, 14> regionExceeded_{};
  bool forceSampleAvailable_ = false;
  bool externalTorqueFeedback_ = false;
  bool highPassCompensation_ = false;
  double externalJointTorqueNormNm_ = 0;
  void updateContactForces();''')
        header_path.write_text(header)
    if cpp != cpp_path.read_text():
        cpp_path.write_text(cpp)
    shutil.copyfile(Path(__file__).with_name("SmplhContactForces.h"),
                    folder / "SmplhContactForces.h")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", type=Path, required=True)
    install(parser.parse_args().source)
