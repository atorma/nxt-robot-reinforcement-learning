package org.atorma.robot.learning.montecarlo;

import java.util.*;

import org.apache.commons.math3.distribution.EnumeratedDistribution;
import org.apache.commons.math3.util.Pair;
import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.learning.DiscreteActionModel;
import org.atorma.robot.learning.QTable;
import org.atorma.robot.mdp.*;
import org.atorma.robot.policy.DiscretePolicy;

/**
 * Upper confidence bounds in tree.
 */
public class UctPlanning implements DiscretePolicy {

	private DiscreteActionModel model;
	private StateDiscretizer stateDiscretizer;
	private QTable qTable;
	private int horizon;
	private Random random = new Random();
	
	private State startState;

	
	public UctPlanning(DiscreteActionModel model, StateDiscretizer stateDiscretizer, QTable qTable, int horizon) {
		this.model = model;
		this.stateDiscretizer = stateDiscretizer;
		this.qTable = qTable;
		this.horizon = horizon;
	}
	
	public void setRolloutStartState(State startState) {
		this.startState = startState;
	}
	
	public void performRollouts(int num) {
		Queue<State> stateQueue = new LinkedList<>();		
		stateQueue.add(startState);
		
		for (int i = 0; i < num; i++) {
			State state = stateQueue.poll();
			
			if (state == startState) {
				
			} else {
				for (int s = 1; s < horizon; s++) {
					StochasticTransitionReward tr = sampleFrom(model.getOutgoingTransitions(state));
					
				}
			}
		}
	}
	
	private StochasticTransitionReward sampleFrom(Set<StochasticTransitionReward> transitions) {
		double prob = random.nextDouble();
		double cumProb = 0;
		for (StochasticTransitionReward tr : transitions) {
			cumProb += tr.getProbability();
			if (prob < cumProb) {
				return tr;
			}
		}
		
		// Should not happen
		return transitions.iterator().next();
	}

	@Override
	public Integer getActionId(int stateId) {
		// TODO Auto-generated method stub
		return null;
	}
	
}
