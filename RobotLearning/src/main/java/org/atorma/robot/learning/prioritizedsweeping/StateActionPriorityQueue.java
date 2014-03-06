package org.atorma.robot.learning.prioritizedsweeping;

import java.util.*;

import org.atorma.robot.mdp.StateAction;

class StateActionPriorityQueue {
	
	private Set<StateAction> stateActions = new HashSet<StateAction>();
	private PriorityQueue<PrioritizedStateAction> priorityQueue = new PriorityQueue<>();
	
	public void addOrUpdate(StateAction stateAction, double priority) {
					
		if (this.stateActions.contains(stateAction)) {
			Iterator<PrioritizedStateAction> iter = priorityQueue.iterator();
			while (iter.hasNext()) {
				PrioritizedStateAction element = iter.next();
				if (element.stateAction.equals(stateAction)) {
					iter.remove();
				}
			}
		}
		
		this.stateActions.add(stateAction);
		this.priorityQueue.add(new PrioritizedStateAction(stateAction, priority));
	}
	
	public boolean isEmpty() {
		return priorityQueue.isEmpty();
	}
	
	public StateAction poll() {
		PrioritizedStateAction head = priorityQueue.poll();
		stateActions.remove(head.stateAction);
		return head.stateAction;
	}

	public StateAction peek() {
		return priorityQueue.peek().stateAction;
	}

}