package org.atorma.robot.simplebumper;

import org.atorma.robot.State;

public class BumperState implements State {
	
	public static final int MAX_ULTRASONIC_DIST = 255;
	public static final int MIN_ULTRASONIC_DIST = 7;
	
	private final double[] values;

	public BumperState(int distanceToObstacle) {
		this.values = new double[] {distanceToObstacle, 0};
	}
	
	public BumperState(int distanceToObstacle, boolean isCollided) {
		this.values = new double[] {distanceToObstacle, isCollided ? 1 : 0};
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
	
	public boolean isCollided() {
		return values[1] > 0;
	}
}
