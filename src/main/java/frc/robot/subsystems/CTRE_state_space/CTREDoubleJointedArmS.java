package frc.robot.subsystems.CTRE_state_space;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.DataHandler;
import frc.robot.utils.CTRE_state_space.CTRESpaceConstants;

public class CTREDoubleJointedArmS extends SubsystemBase {
	private TalonFX armMotor = new TalonFX(
			CTRESpaceConstants.DoubleJointedArm.kArmMotorID);
	private TalonFX elbowMotor = new TalonFX(
			CTRESpaceConstants.DoubleJointedArm.kElbowMotorID);
	private Notifier m_updatePositionsNotifier = null; //Checks for updates
	public static double expectedArmRads = 0, expectedElbowRads = 0;
	private final VoltageOut m_voltReq = new VoltageOut(0.0);
	private final Mechanism2d m_mech2d = new Mechanism2d(
			CTRESpaceConstants.DoubleJointedArm.simSizeWidth,
			CTRESpaceConstants.DoubleJointedArm.simSizeLength);
	private final MechanismRoot2d m_DoubleJointedarmPivot = m_mech2d.getRoot(
			"DoubleJointedArmPivot", CTRESpaceConstants.DoubleJointedArm.physicalX,
			CTRESpaceConstants.DoubleJointedArm.physicalY);
	private final MechanismLigament2d m_DoubleJointedArm = m_DoubleJointedarmPivot
			.append(new MechanismLigament2d("DoubleJointedArm",
					CTRESpaceConstants.DoubleJointedArm.armLength,
					Units.radiansToDegrees(expectedArmRads), 1,
					new Color8Bit(Color.kYellow)));
	private final MechanismLigament2d m_DoubleJointedElbow = m_DoubleJointedArm
			.append(new MechanismLigament2d("DoubleJointedElbow",
					CTRESpaceConstants.DoubleJointedArm.elbowLength,
					Units.radiansToDegrees(expectedElbowRads), 1,
					new Color8Bit(Color.kYellow)));
  private final StatusSignal<Double> m_armpos = armMotor.getPosition();
  private final StatusSignal<Double> m_elbowpos = elbowMotor.getPosition();
  private final StatusSignal<Double> m_armvel = armMotor.getVelocity();
  private final StatusSignal<Double> m_elbowvel = elbowMotor.getVelocity();
	public CTREDoubleJointedArmS() {
		TalonFXConfiguration motorConfig = new TalonFXConfiguration();
		motorConfig.CurrentLimits.StatorCurrentLimitEnable = true;
		motorConfig.CurrentLimits.StatorCurrentLimit = CTRESpaceConstants.DoubleJointedArm.armStatorCurrentLimit;
		motorConfig.MotorOutput.Inverted = CTRESpaceConstants.DoubleJointedArm.inverted;
		motorConfig.MotorOutput.NeutralMode = CTRESpaceConstants.DoubleJointedArm.mode;
		motorConfig.Feedback.SensorToMechanismRatio = CTRESpaceConstants.DoubleJointedArm.armGearing;
		armMotor.getConfigurator().apply(motorConfig);
		//do elbow
		motorConfig.CurrentLimits.StatorCurrentLimit = CTRESpaceConstants.DoubleJointedArm.elbowStatorCurrentLimit;
		motorConfig.Feedback.SensorToMechanismRatio = CTRESpaceConstants.DoubleJointedArm.elbowGearing;
		elbowMotor.getConfigurator().apply(motorConfig);
		if (Constants.currentMode == Constants.Mode.SIM){
			m_updatePositionsNotifier = new Notifier(() -> {
				DataHandler.logData(new double[]{expectedArmRads,expectedElbowRads},"DoubleJointedEncoders");
			});
		}else{
			m_updatePositionsNotifier = new Notifier(() -> {
				DataHandler.logData(getEncoders(),"DoubleJointedEncoders");
			});
		}

		m_updatePositionsNotifier.startPeriodic(1);
	}

	public void setMotors(double voltsArm, double voltsElbow) {
		armMotor.setControl(m_voltReq.withOutput(voltsArm));
		elbowMotor.setControl(m_voltReq.withOutput(voltsElbow));
	}

	public double[] getEncoders() {
		BaseStatusSignal.refreshAll(m_armpos, m_armvel,m_elbowpos, m_elbowvel);
		return new double[] {
			Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(m_armpos, m_armvel)),
			Units.rotationsToRadians(BaseStatusSignal.getLatencyCompensatedValue(m_elbowpos, m_elbowvel))
		};
	}

	@Override
	public void periodic() {
		if (Constants.currentMode == Constants.Mode.REAL) {
			m_DoubleJointedArm.setAngle(Units
					.rotationsToDegrees(armMotor.getPosition().getValueAsDouble()));
			m_DoubleJointedElbow.setAngle(Units
					.rotationsToDegrees(elbowMotor.getPosition().getValueAsDouble()));
		} else {
			m_DoubleJointedArm.setAngle(Units.radiansToDegrees(expectedArmRads));
			m_DoubleJointedElbow
					.setAngle(Units.radiansToDegrees(expectedElbowRads));
		}
		Logger.recordOutput("DoubleJointedArmMechanism", m_mech2d);
	}
}
