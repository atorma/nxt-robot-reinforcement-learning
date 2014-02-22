package org.atorma.robot.simplebumper;

import java.util.Arrays;

import org.atorma.robot.Action;

public class BumperAction implements Action {
	
	public static final BumperAction FORWARD = new BumperAction(0);
	public static final BumperAction BACKWARD = new BumperAction(1);
	public static final BumperAction LEFT = new BumperAction(2);
	public static final BumperAction RIGHT = new BumperAction(3);

	private final double[] values;

	public BumperAction(int value) {
		this.values = new double[] {value};
	}
	
	public BumperAction(double[] values) {
		this.values = values;
	}
	
	
	@Override
	public double[] getValues() {
		return values;
	}

	@Override
	public int hashCode() {
		return (int) values[0];
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		BumperAction other = (BumperAction) obj;
		if (!Arrays.equals(values, other.values))
			return false;
		return true;
	}
	
	
}
