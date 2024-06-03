package frc.robot.utils;

/**
 * Class designed to hold the values for Kp, Ks, Kv for state-space models and P and D for PID loops. Remember to check whether your loop should be position or velocity
 * @param Kp
 * The Kp value from SysID, used in state-space
 * @param Ks
 * The Ks value from SysID, used in state-space
 * @param Kv
 * The Kv value from SysID, used in state-space
 * @param P
 * The P value from SysID, used in PID loops
 * @param D
 * the D Value from SysId, used in PID loops
 * 
 */
public class MotorConstantContainer {
	private static double[] valueHolderArray = new double[5];
	
	public MotorConstantContainer(double Ks, double Kv, double Ka, double P, double D) {
		if ((Ka <= 0) || (Kv <= 0)) {
			throw new ArithmeticException("Ka and Kv must be greater than 0");
		} else if ((P < 0) || (D < 0)) {
			throw new ArithmeticException(
					"P and D must be greater than or equal to 0");
		} else {
			valueHolderArray[0] = Ks;
			valueHolderArray[1] = Kv;
			valueHolderArray[2] = Ka;
			valueHolderArray[3] = P;
			valueHolderArray[4] = D;
		}
	}

	public double getKs() {
		 return valueHolderArray[0]; 
		}

	public double getKv() { 
		return valueHolderArray[1]; 
	}

	public double getKa() { 
		return valueHolderArray[2]; 
	}

	public double getP() { 
		return valueHolderArray[3]; 
	}

	public double getD() { 
		return valueHolderArray[4]; 
	}
}
