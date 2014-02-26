package org.atorma.robot.simplebumper;

import org.atorma.robot.DiscreteAction;

public class BumperAction implements DiscreteAction {
	
	public static final BumperAction FORWARD = new BumperAction(0, "Drive forward");
	public static final BumperAction BACKWARD = new BumperAction(1, "Drive backward");
	public static final BumperAction LEFT = new BumperAction(2, "Turn left");
	public static final BumperAction RIGHT = new BumperAction(3, "Turn right");

	private final int id;
	private final String name;

	public BumperAction(int id, String name) {
		this.id = id;
		this.name = name;
	}

	public int getId() {
		return id;
	}

	public String getName() {
		return name;
	}

	@Override
	public int hashCode() {
		final int prime = 31;
		int result = 1;
		result = prime * result + id;
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
		BumperAction other = (BumperAction) obj;
		if (id != other.id)
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
