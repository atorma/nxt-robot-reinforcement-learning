package org.atorma.robot.simplebumper;

import java.util.Arrays;

import org.atorma.robot.Action;

public class BumperAction implements Action {
	
	public static final BumperAction FORWARD = new BumperAction(0, "Drive forward");
	public static final BumperAction BACKWARD = new BumperAction(1, "Drive backward");
	public static final BumperAction LEFT = new BumperAction(2, "Turn left");
	public static final BumperAction RIGHT = new BumperAction(3, "Turn right");

	private final double[] values;
	private final String name;

	public BumperAction(int value, String name) {
		this.values = new double[] {value};
		this.name = name;
	}
	
	public BumperAction(double[] values, String name) {
		this.values = values;
		this.name = name;
	}
	
	public int getId() {
		return (int) this.values[0];
	}

	public String getName() {
		return name;
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

	@Override
	public String toString() {
		return "BumperAction [name=" + name + "]";
	}
	
	public static BumperAction getAction(int actionId) {
		switch(actionId) {
		case 0: return FORWARD;
		case 1: return BACKWARD;
		case 2: return LEFT;
		case 3: return RIGHT;
		default: throw new IllegalArgumentException("Unknown action id " + actionId);
		}
	}
}
