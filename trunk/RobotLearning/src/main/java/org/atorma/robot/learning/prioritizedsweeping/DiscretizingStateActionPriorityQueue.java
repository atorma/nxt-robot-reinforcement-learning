package org.atorma.robot.learning.prioritizedsweeping;

import org.atorma.robot.discretization.StateDiscretizer;
import org.atorma.robot.mdp.DiscretizedStateAction;
import org.atorma.robot.mdp.StateAction;

/**
 * This priority queue stores <tt>StateAction</tt>s but so that there's only one
 * per discretization. This prevents the queue from growing too large in 
 * case of continuous states. 
 */
class DiscretizingStateActionPriorityQueue {
	
	private StateDiscretizer stateDiscretizer;
	private FibonacciHeap<PrioritizedStateAction> fibonacciHeap = new FibonacciHeap<>();
	
	DiscretizingStateActionPriorityQueue(StateDiscretizer stateDiscretizer) {
		this.stateDiscretizer = stateDiscretizer;
	}
	
	/**
	 * Adds a <tt>StateAction</tt> or decreases its priority. The <tt>StateAction</tt>
	 * is considered to be in the queue if there's any <tt>StateAction</tt> that has 
	 * an equal <tt>DiscretizedStateAction</tt>. Updating the priority 
	 * does NOT update the queue <tt>StateAction</tt> instance. An update with higher
	 * priority than existing does nothing.
	 */
	public void addOrDecreasePriority(StateAction stateAction, int priority) {
		int stateId = stateDiscretizer.getId(stateAction.getState()); 			
		int actionId = stateAction.getAction().getId();
		DiscretizedStateAction discretization = new DiscretizedStateAction(stateId, actionId);
		PrioritizedStateAction prioritized = new PrioritizedStateAction(stateAction, discretization, priority);
		
		if (fibonacciHeap.contains(prioritized)) {
			if (fibonacciHeap.getPriority(prioritized) > priority) {
				fibonacciHeap.decreaseKey(prioritized, priority);
			}
		} else {
			fibonacciHeap.add(prioritized, priority);
		}
	}
	
	public boolean isEmpty() {
		return fibonacciHeap.size() == 0;
	}
	
	public StateAction pollMin() {
		PrioritizedStateAction prioritized = fibonacciHeap.popMin();
		return prioritized != null ? prioritized.stateAction : null;
	}

	public StateAction peekMin() {
		PrioritizedStateAction prioritized = fibonacciHeap.peekMin();
		return prioritized != null ? prioritized.stateAction : null;
	}

	public int size() {
		return fibonacciHeap.size();
	}

}