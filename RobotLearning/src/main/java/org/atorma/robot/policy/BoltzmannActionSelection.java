package org.atorma.robot.policy;

import static java.lang.Math.exp;

import java.util.Arrays;

import org.apache.commons.math3.distribution.EnumeratedIntegerDistribution;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.DiscreteAction;
import org.atorma.robot.mdp.DiscretizedStateAction;

public class BoltzmannActionSelection implements DiscretePolicy {
	
	private final QTable qTable;
	private double temperature;
	private final int[] actionIds;

	public BoltzmannActionSelection(QTable qTable, double initialTemperature, int... allActionIds) {
		this.qTable = qTable;
		this.actionIds = Arrays.copyOf(allActionIds, allActionIds.length);
		setTemperature(initialTemperature);
	}
	
	public BoltzmannActionSelection(QTable qTable, double initialTemperature, DiscreteAction... actions) {
		actionIds = new int[actions.length];
		for (int i = 0; i < actions.length; i++) {
			actionIds[i] = actions[i].getId();
		}
		this.qTable = qTable;
		setTemperature(initialTemperature);
	}

	@Override
	public Integer getActionId(int stateId) {
		EnumeratedIntegerDistribution distribution = getActionDistribution(stateId);
		return distribution.sample();
	}

	public EnumeratedIntegerDistribution getActionDistribution(int stateId) {
		double[] probMass = new double[actionIds.length];
		for (int i = 0; i < actionIds.length; i++) {
			DiscretizedStateAction stateAction = new DiscretizedStateAction(stateId, actionIds[i]);
			probMass[i] = exp(qTable.getValue(stateAction)/temperature);
		}
		return new EnumeratedIntegerDistribution(actionIds, probMass);
	}

	public double getTemperature() {
		return temperature;
	}

	public void setTemperature(double temperature) {
		if (temperature <= 0) {
			throw new IllegalArgumentException("Temperature must be greater than zero");
		}
		this.temperature = temperature;
	}

	
}
