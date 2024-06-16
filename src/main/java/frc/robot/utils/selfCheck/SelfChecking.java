package frc.robot.utils.selfCheck;

import java.util.List;

public interface SelfChecking {
  List<SubsystemFault> checkForFaults();
}
