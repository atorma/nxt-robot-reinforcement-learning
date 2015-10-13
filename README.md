# nxt-robot-reinforcement-learning
Implements reinforcement learning algorithms such as Q-learning (incl. eligibility traces), prioritized sweeping, and Monte-Carlo planning. Uses these algorithms to teach a robot to explore and avoid collisions. A robot control interface is implemented for <a href="http://simbad.sourceforge.net/">Simbad</a> simulator and Lego NXT using <a href="http://www.lejos.org/">LeJOS</a>. 

The NXT robot works by periodically sending its sensor data to a PC and expecting control commands back. The PC runs its learning algorithm between these cycles.

Modules
* RobotCore: functionality common for the LeJOS NXT robot and the simulated robot, most notably DiscreteRobotController.java. Use only classes that LeJOS JVM understands.
* RobotLearning: learning algorithms and modeling of the problem
* RobotNxtControl: the code to deploy to the LeJOS NXT brick. Use the LeJOS Eclipse plugin.
* RobotNxtPC: the PC side of robot control control (main class NxtPcControl.java)
* RobotSimulation: simulated robot. Main class is SimbadBumberSimulation.java. <a href="https://java3d.java.net/binary-builds.html">Java 3D</a> installation is required.

For simulations only you can leave out the RobotNxtControl and RobotNxtPC modules. 

