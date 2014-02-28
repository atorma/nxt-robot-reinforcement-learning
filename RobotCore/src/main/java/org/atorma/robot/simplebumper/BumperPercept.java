package org.atorma.robot.simplebumper;

import org.atorma.robot.State;

public class BumperPercept implements State {
	
	public static final int MAX_ULTRASONIC_DIST = 255;
	public static final int MIN_ULTRASONIC_DIST = 7;
	
	private final double[] values;
	

	public BumperPercept(int distanceToObstacle) {
		this.values = new double[] {distanceToObstacle, 0};
	}
	
	public BumperPercept(int distanceToObstacle, boolean isCollided) {
		this.values = new double[] {distanceToObstacle, isCollided ? 1 : 0, 0};
	}
	
	public BumperPercept(int distanceToObstacle, boolean isCollided, int lightValue) {
		this.values = new double[] {distanceToObstacle, isCollided ? 1 : 0, lightValue};
	}
	
	public BumperPercept(double[] values) {
		this.values = values;
	}

	
	@Override
	public double[] getValues() {
		return values;
	}

	public int getDistanceToObstacleInFrontCm() {
		return (int) values[0];
	}
	
	public boolean isCollided() {
		return values[1] > 0;
	}
	
	public int getLightValue() {
		return (int) values[2];
	}

	@Override
	public String toString() {
		return "BumperState [getDistanceToObstacle()="
				+ getDistanceToObstacleInFrontCm() + ", isCollided()=" + isCollided()
				+ ", getLightValue()=" + getLightValue() + "]";
	}
	
	
}
