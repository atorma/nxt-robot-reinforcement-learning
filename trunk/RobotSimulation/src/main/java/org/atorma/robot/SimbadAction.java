package org.atorma.robot;

public interface SimbadAction {
	
	/** 
	 * Perform or start the action.
	 */
	void perform();

	/**
	 * True if the action is now completed and no more calls to perform are needed.
	 * This method is called by the main robot loop after each <tt>perform()</tt> call. 
	 */
	boolean isCompleted();
}
