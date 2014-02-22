package org.atorma.robot.communications;

public class StateAndAction {

	private double[] stateValues;
	private double[] actionValues;
	
	public StateAndAction(double[] stateValues, double[] actionValues) {
		if (stateValues == null || actionValues == null) {
			throw new IllegalArgumentException("Null state or action");
		}
		this.stateValues = stateValues;
		this.actionValues = actionValues;
	}

	public double[] getStateValues() {
		return stateValues;
	}

	public double[] getActionValues() {
		return actionValues;
	}

}
