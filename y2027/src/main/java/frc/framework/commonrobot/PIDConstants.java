package frc.framework.commonrobot;

/**
 * Represents Position-Integral-Derivative constants, with an additional gravitational compensation constant
 */
public class PIDConstants {
	private double p;
	private double i;
	private double d;
	private double g;
	
	/**
	 * @return The derivative constant
	 */
	public double getD() {
		return d;
	}
	
	/**
	 * @return The gravitational counteraction constant
	 */
	public double getG() {
		return g;
	}
	
	/**
	 * @return The integral constant
	 */
	public double getI() {
		return i;
	}
	
	/**
	 * @return The proportional constant
	 */
	public double getP() {
		return p;
	}
	
	/**
	 * Set the derivative constant
	 *
	 * @param next The new derivative constant
	 *
	 * @return this
	 */
	public PIDConstants withD(double next) {
		this.d = next;
		return this;
	}
	
	/**
	 * Set the gravitational counteraction constant
	 *
	 * @param next The new gravitational counteraction constant
	 *
	 * @return this
	 */
	public PIDConstants withG(double next) {
		this.g = next;
		return this;
	}
	
	
	/**
	 * Set the integral constant
	 *
	 * @param next The new integral constant
	 *
	 * @return this
	 */
	public PIDConstants withI(double next) {
		this.i = next;
		return this;
	}
	
	/**
	 * Set the proportional constant
	 *
	 * @param next The new proportional constant
	 *
	 * @return this
	 */
	public PIDConstants withP(double next) {
		this.p = next;
		return this;
	}
}
