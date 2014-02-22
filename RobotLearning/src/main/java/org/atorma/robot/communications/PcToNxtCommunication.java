package org.atorma.robot.communications;

import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.util.Collection;
import java.util.Map;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.LinkedBlockingQueue;

import lejos.pc.comm.NXTCommException;
import lejos.pc.comm.NXTCommFactory;
import lejos.pc.comm.NXTCommLogListener;
import lejos.pc.comm.NXTConnector;

import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.communications.DataStreamCommunications;
import org.atorma.robot.communications.StateAndAction;

/**
 * The PC side of PC <-> NXT Bluetooth communications. Initiates the connection.
 */
public class PcToNxtCommunication implements RobotCommunications, Runnable {

	private DataStreamCommunications comms;
	private NXTConnector nxtConnector;
	private volatile boolean isOpen;
    private BlockingQueue<StateAndAction> states = new LinkedBlockingQueue<StateAndAction>();
    private volatile PolicyIdMap policy;

	public PcToNxtCommunication(String nxtName) throws NXTCommException {
		
		nxtConnector = new NXTConnector();
		nxtConnector.addLogListener(new NXTCommLogListener() {
			public void logEvent(String message) {
				System.out.println(message);				
			}

			public void logEvent(Throwable throwable) {
				System.err.println(throwable.getMessage());			
			}			
		});
		isOpen = nxtConnector.connectTo(nxtName, null, NXTCommFactory.BLUETOOTH);
        
		
        comms = new DataStreamCommunications(new DataInputStream(nxtConnector.getInputStream()), new DataOutputStream(nxtConnector.getOutputStream()));
        
        new Thread(this).start();
    }

	@Override
	public void disconnect() {
		try {
			comms.closeDataStreams();
			nxtConnector.close();
			isOpen = false;
		} catch (Exception e) {}
	}
	
	@Override
	public StateAndAction takeStateAndAction() {
		StateAndAction values = null;
		while (values == null) {
			try {
				values = states.take();
			} catch (InterruptedException e) {}
		}
		return values;
	}
	
	@Override
	public void drainStatesAndActionsInto(Collection<StateAndAction> target) {
		states.drainTo(target);
	}
	
	@Override
	public synchronized void updatePolicy(PolicyIdMap policy) {
		this.policy = policy;
	}
	
	private synchronized PolicyIdMap takePolicy() {
		PolicyIdMap taken = this.policy;
		this.policy = null;
		return taken;
	}
	
	private void receiveStates() {
		comms.flushInt(DataStreamCommunications.STATE_AND_ACTION);
		double[] stateValues = comms.receiveDoubleArray();
		double[] actionValues = comms.receiveDoubleArray();
		
		if (stateValues == null || actionValues == null) {
			return;
		}
		
		StateAndAction values = new StateAndAction(stateValues, actionValues);
		try {
			states.put(values);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
	}
		
	private void transmitPolicy() {
		PolicyIdMap policy = takePolicy();
		comms.flushInt(DataStreamCommunications.POLICY_VALUES);
		comms.flushInt(policy.size());
		for (Map.Entry<Integer, Integer> entry : policy.entrySet()) {
			comms.flushInt(entry.getKey());
			comms.flushInt(entry.getValue());
		}
	}
	
	
	@Override
	public void run() {
		while (isOpen) {
			try {
				
				if (policy != null) {
					transmitPolicy();
				} else {
					receiveStates();
				}
				
				
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
}
