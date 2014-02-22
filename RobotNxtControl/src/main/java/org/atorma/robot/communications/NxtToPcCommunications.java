package org.atorma.robot.communications;

import java.io.IOException;
import java.util.Queue;

import lejos.nxt.Button;
import lejos.nxt.comm.BTConnection;
import lejos.nxt.comm.Bluetooth;

import org.atorma.robot.NxtAction;
import org.atorma.robot.PolicyIdMap;
import org.atorma.robot.State;

/**
 * The NXT side of PC <-> NXT Bluetooth communications. Waits for PC to initiate all communication. NXT acts as a slave.
 */
public class NxtToPcCommunications implements Runnable {
	
	private static final int STATE_QUEUE_LIMIT = 1000;
	
	private DataStreamCommunications comms;
	private BTConnection btc = null;
	private Queue<StateAndAction> stateQueue = new Queue<StateAndAction>(); 
	private PolicyIdMap policy = null;
	private volatile boolean run;

	public NxtToPcCommunications() {
        try {
            System.out.println("Waiting for client.");
            
            btc = Bluetooth.waitForConnection();            
            comms = new DataStreamCommunications(btc.openDataInputStream(), btc.openDataOutputStream());
            
            System.out.println("Connected.");
            
            run = true;
            new Thread(this).start();
            
        } catch(Exception e) {
            System.out.println("Error in Bluetooth connection.");      
            Button.ENTER.waitForPressAndRelease();
        }
    }

	public synchronized void disconnect() {
		run = false;
		try {
			comms.closeDataStreams();
			btc.close();
		} catch (Exception e) {}
		System.out.println("Disconnected.");
	}

	/**
	 * Listen to what the PC wants to send or receive.
	 */
	@Override
	public void run() {
		
		while (run) {
			try {
				
				int request = comms.readInt();
				if (request == DataStreamCommunications.STATE_AND_ACTION) {
					transmitStateAndAction();
				} else if (request == DataStreamCommunications.POLICY_VALUES) {
					receivePolicy();
				} else {
					System.out.println("Invalid request");
					disconnect();
					break;
				}
				
			} catch (IOException e) {
				System.out.println("Communication error!");
				disconnect(); // TODO OK?
				break;
			}
		}
	}

	private void transmitStateAndAction() {
		StateAndAction sa = popStateAndAction();
		comms.transmitDoubleArray(sa.getStateValues());
		comms.transmitDoubleArray(sa.getActionValues());
	}
	
	private synchronized StateAndAction popStateAndAction() {
		if (stateQueue.isEmpty()) {
			try {
				wait();
			} catch (InterruptedException e) {}
		}
		return (StateAndAction) stateQueue.pop();
	}
	
	public synchronized void pushStateAndAction(State state, NxtAction action) {
		if (stateQueue.size() == STATE_QUEUE_LIMIT) {
			stateQueue.pop(); // throw oldest away
		}
		stateQueue.push(new StateAndAction(state.getValues(), action.getValues()));
		notify();
	}
	
	
	private void receivePolicy() {
		PolicyIdMap policy = new PolicyIdMap();
		try {
			int numEntries = comms.readInt();
			for (int i=0; i<numEntries; i++) {
				int stateId = comms.readInt();
				int actionId = comms.readInt();
				policy.put(stateId, actionId);
			}
		} catch (IOException e) {
			throw new RuntimeException("Error when receiving policy");
		}
		setPolicy(policy);
	}
	
	private synchronized void setPolicy(PolicyIdMap policy) {
		this.policy = policy;
	}
	
	public synchronized boolean isPolicyAvailable() {
		return policy != null;
	}
	
	public synchronized PolicyIdMap takePolicy() {
		PolicyIdMap temp = this.policy;
		this.policy = null;
		return temp;
	}
	
	
}
