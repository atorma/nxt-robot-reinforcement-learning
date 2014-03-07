package org.atorma.robot.objecttrackingbumper.prioritizedsweeping;

import java.util.Map;
import java.util.Set;

import org.atorma.robot.discretization.VectorDiscretizer;
import org.atorma.robot.learning.prioritizedsweeping.PrioritizedSweepingModel;
import org.atorma.robot.mdp.*;
import org.atorma.robot.objecttracking.ObjectTrackingModel;
import org.atorma.robot.simplebumper.BumperAction;

import com.google.common.collect.Sets;

public class BumperModel implements PrioritizedSweepingModel {
	
	private RewardFunction rewardFunction;
	private Map<Double, Double> collisionProbGivenDistanceToObstacle;	
	

	@Override
	public Set<? extends DiscreteAction> getAllActions() {
		return Sets.newHashSet(BumperAction.values());
	}

	@Override
	public Set<StochasticTransitionReward> getOutgoingTransitions(StateAction stateAction) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public Set<StochasticTransitionReward> getIncomingTransitions(State state) {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void updateModel(TransitionReward observation) {
		// TODO Auto-generated method stub

	}

}
