package org.atorma.robot.communications;

import java.util.Collection;

import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.communications.StateAndAction;

public interface RobotCommunications {

	void updatePolicy(PolicyIdMap policy);

	void drainStatesAndActionsInto(Collection<StateAndAction> target);

	StateAndAction takeStateAndAction();

	void disconnect();

}
