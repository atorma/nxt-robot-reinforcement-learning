package org.atorma.robot.communications;

public class CommunicationException extends RuntimeException {

	public CommunicationException() {
		super();
	}

	public CommunicationException(String message, Throwable cause) {
		super(message, cause);
	}

	public CommunicationException(String message) {
		super(message);
	}

	public CommunicationException(Throwable cause) {
		super(cause);
	}

	
}
