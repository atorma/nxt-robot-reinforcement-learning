package org.atorma.robot.simplebumper;

import org.atorma.robot.State;

public class BumperState implements State {
	
	public static final int MAX_ULTRASONIC_DIST = 255;
	public static final int MIN_ULTRASONIC_DIST = 7;
	
	private final double[] values;

	public BumperState(int distanceToObstacle) {
		this.values = new double[] {distanceToObstacle};
	}
	
	public BumperState(double[] values) {
		this.values = values;
	}

	@Override
	public double[] getValues() {
		return values;
	}

	public int getDistanceToObstacle() {
		return (int) values[0];
	}
	
}
