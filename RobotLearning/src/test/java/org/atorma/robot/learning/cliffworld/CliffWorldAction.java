package org.atorma.robot.learning.cliffworld;

import org.atorma.robot.mdp.DiscreteAction;

public enum CliffWorldAction implements DiscreteAction {

	UP(0, 1, 0),
	DOWN(0, -1, 1),
	LEFT(-1, 0, 2),
	RIGHT(1, 0, 3);
	
	public final int dx;
	public final int dy;
	public final int id; 
	
	private CliffWorldAction(int dx, int dy, int id) {
		this.dx = dx;
		this.dy = dy;
		this.id = id;
	}

	@Override
	public int getId() {
		return id;
	}
	
	public static CliffWorldAction getActionById(int id) {
		if (id == UP.id) {
			return UP;
		} else if (id == DOWN.id) {
			return DOWN;
		} else if (id == LEFT.id) {
			return LEFT;
		} else if (id == RIGHT.id) {
			return RIGHT;
		} else {
			throw new IllegalArgumentException();
		}
	}

}
