package frc.robot.utils.selfCheck.drive;

import java.util.List;

import frc.robot.utils.selfCheck.SubsystemFault;

public interface SelfChecking {
  List<SubsystemFault> checkForFaults();
  Object getHardware();
}
