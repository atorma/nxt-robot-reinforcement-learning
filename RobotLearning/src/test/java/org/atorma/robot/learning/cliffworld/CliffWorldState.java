package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.mdp.State;

public class CliffWorldState implements State {

	public static final int X_MIN = 0;
	public static final int X_MAX = 11;
	public static final int Y_MIN = 0;
	public static final int Y_MAX = 3;

	public static final CliffWorldState START = new CliffWorldState(X_MIN, Y_MIN);
	
	private final int x;
	private final int y;
	
	public CliffWorldState(int x, int y) {
		this.x = x;
		this.y = y;
	}
	
	@Override
	public double[] getValues() {
		return new double[] {x, y};
	}
	
	// Cliff world environment
	
	public boolean isOutOfBounds() {
		return x < X_MIN || x > X_MAX || y < Y_MIN || y > X_MAX;
	}
	
	public boolean isCliff() {
		return x > X_MIN && x < X_MAX && y == Y_MIN; 
	}
	
	public boolean isGoal() {
		return x == X_MAX && y == Y_MIN;
	}
	
	public CliffWorldState getNextState(CliffWorldAction action) {
		CliffWorldState nextState = new CliffWorldState(x + action.dx, y + action.dy);
		if (nextState.isOutOfBounds() || isGoal()) {
			return this;
		} else if (nextState.isCliff()) {
			return START;
		} else {
			return nextState;
		}
	}

}
