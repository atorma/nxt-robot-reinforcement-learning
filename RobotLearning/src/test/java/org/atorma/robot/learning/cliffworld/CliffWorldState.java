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

	public int getX() {
		return x;
	}

	public int getY() {
		return y;
	}

	@Override
	public String toString() {
		return "CliffWorldState [x=" + x + ", y=" + y + "]";
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

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + x;
		result = prime * result + y;
		return result;
	}

	@Override
	public boolean equals(Object obj) {
		if (this == obj)
			return true;
		if (obj == null)
			return false;
		if (getClass() != obj.getClass())
			return false;
		CliffWorldState other = (CliffWorldState) obj;
		if (x != other.x)
			return false;
		if (y != other.y)
			return false;
		return true;
	}
	
	

}
