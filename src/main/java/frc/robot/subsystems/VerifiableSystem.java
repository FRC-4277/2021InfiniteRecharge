package frc.robot.subsystems;

import java.util.List;

public interface VerifiableSystem {
  List<Verification> getVerifications(VerificationSystem system);
}
