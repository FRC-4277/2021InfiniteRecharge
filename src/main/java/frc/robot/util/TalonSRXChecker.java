package frc.robot.util;

import com.ctre.phoenix.motorcontrol.Faults;
import com.ctre.phoenix.motorcontrol.StickyFaults;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.subsystems.VerificationSystem;

public class TalonSRXChecker {
  public static boolean check(String name, TalonSRX talonSRX, VerificationSystem system) {
    Faults faults = new Faults();
    talonSRX.getFaults(faults);
    if (faults.hasAnyFault()) {
      if (faults.UnderVoltage) {
        system.error(faultError(name, "UnderVoltage"));
        return false;
      } else if (faults.ForwardLimitSwitch) {
        system.error(faultError(name, "ForwardLimitSwitch"));
        return false;
      } else if (faults.ReverseLimitSwitch) {
        system.error(faultError(name, "ReverseLimitSwitch"));
        return false;
      } else if (faults.ForwardSoftLimit) {
        system.error(faultError(name, "ForwardSoftLimit"));
        return false;
      } else if (faults.ReverseSoftLimit) {
        system.error(faultError(name, "ReverseSoftLimit"));
        return false;
      } else if (faults.ResetDuringEn) {
        system.error(faultError(name, "ResetsDuringEn"));
        return false;
      } else if (faults.SensorOverflow) {
        system.error(faultError(name, "SensorOverflow"));
        return false;
      } else if (faults.SensorOutOfPhase) {
        system.error(faultError(name, "SensorOutOfPhase"));
        return false;
      } else if (faults.RemoteLossOfSignal) {
        system.error(faultError(name, "RemoteLossOfSignal"));
        return false;
      } else if (faults.APIError) {
        system.error(faultError(name, "APIError"));
        return false;
      } else {
        system.error(faultError(name, "CheckTuner"));
        return false;
      }
    }
    StickyFaults stickyFaults = new StickyFaults();
    talonSRX.getStickyFaults(stickyFaults);
    if (stickyFaults.hasAnyFault()) {
      if (stickyFaults.UnderVoltage) {
        system.error(stickyFaultError(name, "UnderVoltage"));
        return false;
      } else if (stickyFaults.ForwardLimitSwitch) {
        system.error(stickyFaultError(name, "ForwardLimitSwitch"));
        return false;
      } else if (stickyFaults.ReverseLimitSwitch) {
        system.error(stickyFaultError(name, "ReverseLimitSwitch"));
        return false;
      } else if (stickyFaults.ForwardSoftLimit) {
        system.error(stickyFaultError(name, "ForwardSoftLimit"));
        return false;
      } else if (stickyFaults.ReverseSoftLimit) {
        system.error(stickyFaultError(name, "ReverseSoftLimit"));
        return false;
      } else if (stickyFaults.ResetDuringEn) {
        system.error(stickyFaultError(name, "ResetsDuringEn"));
        return false;
      } else if (stickyFaults.SensorOverflow) {
        system.error(stickyFaultError(name, "SensorOverflow"));
        return false;
      } else if (stickyFaults.SensorOutOfPhase) {
        system.error(stickyFaultError(name, "SensorOutOfPhase"));
        return false;
      } else if (stickyFaults.RemoteLossOfSignal) {
        system.error(stickyFaultError(name, "RemoteLossOfSignal"));
        return false;
      } else {
        system.error(stickyFaultError(name, "CheckTuner"));
        return false;
      }
    }

    return true;
  }

  private static String faultError(String name, String faultName) {
    return "Talon[" + name + "] Fault: " + faultName;
  }

  private static String stickyFaultError(String name, String faultName) {
    return "Talon[" + name + "] StickyFault: " + faultName;
  }
}
