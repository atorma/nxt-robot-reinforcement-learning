package org.atorma.robot;

public interface SimbadAction {
	
	/** Number of times per _simulated_ second the simulator calls the robot's {@link #perform()} methods */
	public static final double ACTION_CALL_FREQUENCY_HZ = 20;
	
	/** 
	 * Perform or start the action. The simulator will call this method
	 * {@link #ACTION_CALL_FREQUENCY_HZ} times per second.
	 */
	void perform();

	/**
	 * True if the action is now completed and no more calls to perform are needed.
	 * This method is called by the main robot loop after each <tt>perform()</tt> call. 
	 */
	boolean isCompleted();
}
