package org.atorma.robot.simplebumper;

import org.atorma.robot.mdp.DiscreteAction;

public enum BumperAction implements DiscreteAction {
	FORWARD,
	BACKWARD,
	LEFT,
	RIGHT;
	
	public static final int DRIVE_DISTANCE_CM = 3; 
	public static final int TURN_DEGREES = 15; 

	@Override
	public int getId() {
		return this.ordinal();
	}
	
	public static BumperAction getAction(int actionId) {
		return BumperAction.values()[actionId];
	}

}
