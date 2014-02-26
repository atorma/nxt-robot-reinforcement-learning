package org.atorma.robot.communications;

import static org.atorma.robot.communications.MessageConstants.*;

import java.io.DataInputStream;
import java.io.DataOutputStream;

import org.atorma.robot.DiscreteActionPolicy;

import lejos.pc.comm.NXTCommException;
import lejos.pc.comm.NXTCommFactory;
import lejos.pc.comm.NXTCommLogListener;
import lejos.pc.comm.NXTConnector;

/**
 * The PC side of PC <-> NXT Bluetooth communications. Initiates the connection, then listen to 
 * NXT asking for action given current state.
 */
public class PcToNxtCommunication implements Runnable {

	private DiscreteActionPolicy policy;
	private DataStreamCommunications comms;
	private NXTConnector nxtConnector;
	private volatile boolean isOpen;

	public PcToNxtCommunication(String nxtName, DiscreteActionPolicy policy) throws NXTCommException {
		
		if (policy == null) {
			throw new IllegalArgumentException("Policy must not be null");
		}
		
		this.policy = policy;
		
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

	public void disconnect() {
		try {
			comms.closeDataStreams();
			nxtConnector.close();
			isOpen = false;
		} catch (Exception e) {}
	}

	
	@Override
	public void run() {
		while (isOpen) {
			try {
				
				int request = comms.readInt();
				
				if (request == DISCONNECT) {
					disconnect();

				} else if (request == SEND_PERCEPT_RECEIVE_ACTION) {
					
					double[] stateValues = comms.receiveDoubleArray();
					int actionId = policy.getActionId(stateValues);
					comms.flushInt(actionId);
					
				}
				
			} catch (Exception e) {
				e.printStackTrace();
			}
		}
	}
}
