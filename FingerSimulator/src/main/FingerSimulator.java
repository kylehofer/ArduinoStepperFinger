/*
 * Copyright (c) 2018 Kyle Hofer
 * Email: kylehofer@neurak.com.au
 * Web: https://neurak.com.au/
 *
 * This file is part of ArduinoStepperFinger.
 *
 * ArduinoStepperFinger is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * ArduinoStepperFinger is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

package main;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;
import com.fazecast.jSerialComm.*;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.ScheduledFuture;
import java.util.concurrent.TimeUnit;
import java.awt.geom.Point2D;
import java.awt.geom.Point2D.Double;

/*
 * 
 * A simulation program that handles the data of a finger, along with a
 * window which displays the fingers animation real time. Containing multiple
 * controls to individually control each part of the finger, and display real time
 * data of the fingers location in a 2D space. Also allows for controlling an Arduino
 * connected to stepper motors controlling a real world finger.
 * 
 */

public class FingerSimulator extends JFrame {

	private static final long serialVersionUID = -5872034986175365620L;

	private static final double MOTOR_STEP = 1.8; //In degrees
	private static final int SCALER = 2; //Scales the model for display
	private static final double RADIAN_TO_DEGREES = Math.PI / 180;

	private static final double DISTAL_LENGTH = 23.0F * SCALER;
	private static final double MIDDLE_LENGTH = 38.0F * SCALER;
	private static final double PROXIMAL_LENGTH = 75.0F * SCALER;

	public enum SimulationMode {
		NORMAL, LINKED, SIMULATION
	}


	/*
	 * Observer pattern used to notify the simulation panel and the data panels of changes to the finger.
	 */

	private interface IPhalangeListener { 
		void phalangeUpdated();
	}

	/*
	 * Used to send commands to from the control panel to the data panels.
	 */

	private interface IPhalangeControl {
		void setStepValue(double stepValue);
		void setStep(int step);
		int getStep();
		Point2D.Double getPoint();
	}

	/*
	 * Used to send commands to from the data panels to the Arduino via the control panel.
	 */
	private interface IArduinoControl {
		void rotate(IPhalangeControl phalange, int angle);
	}


	/*
	 * Implements Runnable to allow for synchronous execution of the simulations.
	 * It times it's own events based on their step speed of the Arduino creating
	 * a simulation that runs at the same time as the real world implementation.
	 */
	private static class SimulationController implements Runnable {

		/*
		 * Container for a list of commands for the simulation.
		 */
		private class SimulationCommands {
			private IPhalangeControl _controller;
			private int _step;

			public SimulationCommands(IPhalangeControl controller, int step) {
				_controller = controller;
				_step = step;
			}

			public void runCommand() {
				_controller.setStep(_step);
			}		
		}		

		private static final int BASE_STEP_TIME = 8192;	////Step time based of a Full Step. In Microseconds	


		private final ScheduledExecutorService scheduler =
				Executors.newScheduledThreadPool(1); //Thread used for execution

		private ScheduledFuture<?> _commandScheduler;	

		List<SimulationCommands> _commands;
		private long _microStepTime;


		public SimulationController(int microStep) {			
			_commands = new ArrayList<SimulationCommands>();			
			_microStepTime = BASE_STEP_TIME / microStep;
		}

		public void setMicroStep(int microStep) {
			_microStepTime = BASE_STEP_TIME / microStep;
		}

		//Add commands, and starts the timer if it is currently not running
		public void addCommand(IPhalangeControl controller, int step) {
			_commands.add(new SimulationCommands(controller, step));
			if (_commandScheduler == null || _commandScheduler.isCancelled())
				_commandScheduler = scheduler.scheduleAtFixedRate(this, _microStepTime,
						_microStepTime, TimeUnit.MICROSECONDS);
		}

		//Runs the commands, and stops the timer if there are no more commands to run.
		@Override
		public void run() {
			if (_commands.size() > 0) {
				_commands.remove(0).runCommand();
			}
			else {
				_commandScheduler.cancel(true);
			}

		}
	}

	/*
	 * Controls the serial communication with the Arduino.
	 * Implements Runnable to avoid overflowing the serial data connection.
	 */
	private static class ArduinoCommunication implements Runnable {		

		private static final int BUFFER_LIMIT = 58;	//Limit to avoid over saturating the serial communication.	
		private SerialPort _port;
		ArrayList<byte[]> _commands; //List of commands in the buffer

		private final ScheduledExecutorService scheduler =
				Executors.newScheduledThreadPool(1); //Thread used for execution

		private ScheduledFuture<?> _writeScheduler;


		public ArduinoCommunication(SerialPort port) {
			_port = port;
			_commands = new ArrayList<byte[]>();
		}

		//Add commands, and starts the timer if it is currently not running
		public boolean writeData(byte[] data) {			
			_commands.add(data);
			if (_writeScheduler == null || _writeScheduler.isCancelled())
				_writeScheduler = scheduler.scheduleAtFixedRate(this, 0, 5000, TimeUnit.MICROSECONDS);

			return false;
		}

		//Writes data to the arduino, while also throttling data if there's a possibility it'll go over it's buffer limit. 
		@Override
		public void run() {
			if (_commands.size() > 0) {
				if (_port.bytesAwaitingWrite() < BUFFER_LIMIT) {
					byte[] data = _commands.remove(0);
					_port.writeBytes(data, data.length);
				}
			}
			else {
				_writeScheduler.cancel(true);
			}
		}

	}

	/*
	 * A class used to control the individual Phalange's of a finger.
	 * Each phalange is listened to by both the drawing frame, and the control
	 * frames alerting them to updates. Phalanges also propagate updates from the
	 * base to the tip to correctly reflect updates in angles.
	 */

	private class Phalange {

		private Phalange _joint; //The next phalange of the finger.
		private List<IPhalangeListener> _listeners;

		private double _length;
		private Point2D.Double _point;
		private Point2D.Double _offsetPoint; //Offset of the base Joints
		private double _angle;
		private double _offsetAngle;

		private boolean _hasSlider;
		private boolean _linkedPrevious;


		//Utilizes Sine and Cosine to map the arc of the phalanges
		private void updatePoint() {
			_point.x = _offsetPoint.x + 
					_length * Math.sin(_angle + _offsetAngle);
			_point.y = _offsetPoint.y + 
					_length * Math.cos(_angle + _offsetAngle);

			if (_joint != null)
				_joint.updateAngle(_offsetAngle, _angle);

			for (IPhalangeListener l : _listeners)
				l.phalangeUpdated();
		}

		public double getAngle() {
			return _angle;
		}

		public void setAngle(double value) {
			_angle = value;
			updatePoint();
		}

		public Point2D.Double getPoint() {
			return _point;
		}


		public boolean hasSlider() {
			return _hasSlider;
		}

		public void addListener(IPhalangeListener value) {
			_listeners.add(value);
		}

		public Phalange(Phalange joint, double length, boolean hasSlider, boolean linkedPrevious) {

			_length = length;
			_angle = 0;
			_offsetAngle = 0;
			_point = new Point2D.Double(0,0);
			_offsetPoint = new Point2D.Double(0,0);
			_hasSlider = hasSlider;
			_linkedPrevious = linkedPrevious;

			_listeners = new ArrayList<IPhalangeListener>();

			_joint = joint;

			if (joint != null)
				joint.setOffset(_point);
		}

		public void updateAngle(double offsetAngle, double angle) {
			if (_linkedPrevious) {
				_offsetAngle = angle + offsetAngle;
				_angle = angle;
			}
			else
				_offsetAngle = angle + offsetAngle;

			updatePoint();
		}

		public void setOffset(Point2D.Double offset) {
			_offsetPoint = offset;
		}		
	}	

	/*
	 * A class used to contain the Phalanges of a finger.
	 * About as simple as that.
	 */

	private class Finger {

		private Phalange _distal;
		private Phalange _middle;
		private Phalange _proximal;

		public Phalange getDistal() {
			return _distal;
		}

		public Phalange getMiddle() {
			return _middle;
		}

		public Phalange getProximal() {
			return _proximal;
		}

		public Point2D.Double getDistalPoint() {
			return _distal.getPoint();
		}

		public Point2D.Double getMiddlePoint() {
			return _middle.getPoint();
		}

		public Point2D.Double getProximalPoint() {
			return _proximal.getPoint();
		}

		public Finger() {

			_distal = new Phalange(null, DISTAL_LENGTH, false, true);
			_middle = new Phalange(_distal, MIDDLE_LENGTH, true ,false);
			_proximal = new Phalange(_middle, PROXIMAL_LENGTH, true ,false);

			_proximal.updatePoint();
		}

	}

	/*
	 * 
	 * JFrame used to contain the drawing of the simulation.
	 * Observes the Distal Phalange of a finger to detect any updates requiring drawing.
	 * 
	 */

	private static class SimulationPanel extends JPanel implements IPhalangeListener {

		private static final long serialVersionUID = 127045778703041639L;
		private static final int PREF_W = 600;
		private static final int PREF_H = 600;

		private static final int JOINT_RADIUS = 15 * SCALER;
		private static final int JOINT_DIAMETER = (JOINT_RADIUS << 1);
		private static final int TIP_RADIUS = 10 * SCALER;
		private static final int TIP_DIAMETER = (TIP_RADIUS << 1);


		private Finger _finger;

		private int getOriginX() {
			return this.getWidth() >> 1;
		}

		private int getOriginY() {
			return this.getHeight() - 200;
		}

		public SimulationPanel(Finger finger) {
			_finger = finger;
			finger.getDistal().addListener(this);
		}		

		public void drawFinger(Graphics g) {

			int originX = getOriginX(), originY = getOriginY();
			int distalX =  originX + (int)_finger.getDistalPoint().x ,
					distalY = originY - (int)_finger.getDistalPoint().y;

			int middleX =  originX + (int)_finger.getMiddlePoint().x ,
					middleY = originY - (int)_finger.getMiddlePoint().y;

			int proximalX =  originX + (int)_finger.getProximalPoint().x ,
					proximalY = originY - (int)_finger.getProximalPoint().y;

			//Drawing Joints			
			g.fillOval(originX - JOINT_RADIUS, originY - JOINT_RADIUS, JOINT_DIAMETER, JOINT_DIAMETER);			
			g.fillOval(middleX - JOINT_RADIUS, middleY - JOINT_RADIUS, JOINT_DIAMETER, JOINT_DIAMETER);
			g.fillOval(proximalX - JOINT_RADIUS, proximalY - JOINT_RADIUS, JOINT_DIAMETER, JOINT_DIAMETER);

			//Drawing Tip
			g.fillOval(distalX - TIP_RADIUS, distalY - TIP_RADIUS, TIP_DIAMETER, TIP_DIAMETER);

			//Drawing Stems
			Graphics2D g2 = (Graphics2D) g;

			g2.setStroke(new BasicStroke(TIP_DIAMETER >> 1));
			g2.drawLine(originX, originY, proximalX, proximalY);
			g2.drawLine(proximalX, proximalY, middleX, middleY);
			g2.drawLine(middleX, middleY, distalX, distalY);


		}

		@Override
		protected void paintComponent(Graphics g) {
			super.paintComponent(g);
			g.drawLine(0,this.getHeight() - 200, this.getWidth(), this.getHeight() - 200);
			g.drawLine(getOriginX(),0, getOriginX(), this.getHeight());
			drawFinger(g);
		}

		public Dimension getPreferredSize() {
			return new Dimension(PREF_W, PREF_H);
		}

		@Override
		public void phalangeUpdated() {
			this.repaint();

		}
	}

	/*
	 * JFrame used to control the movement of a phalange.
	 * Each Data Panel controls a single phalange of a finger.
	 * Display the X, Y points, Angle of the phalange, and a
	 * Slider used to control the movement of the finger.
	 * Using the GridBag Layout, on a 5x2 grid.
	 */

	private static class DataPanel extends JPanel implements ActionListener, ChangeListener, IPhalangeListener, IPhalangeControl {

		private static final long serialVersionUID = -704085048705110485L;

		private static final int SLIDER_MIN = 0;

		private static final int PREF_W = 222;
		private static final int PREF_H = 87;

		private double _stepValue;

		private IArduinoControl _arduinoController;
		private int _deltaSlider;

		private JLabel _labelAngle;
		private JLabel _labelX;
		private JLabel _labelY;
		private JLabel _textAngle;
		private JLabel _textX;
		private JLabel _textY;		
		private JSlider _sliderAngle;	

		private Phalange _phalange;

		public DataPanel(String title, Phalange phalange) {

			_phalange = phalange;

			_deltaSlider = 0;

			phalange.addListener(this);

			_labelAngle = new JLabel("Angle");
			_labelX = new JLabel("X");
			_labelY = new JLabel("Y");			

			_textAngle = new JLabel(String.format("%.2f", phalange.getAngle()));
			_textX = new JLabel(String.format("%.2f", phalange.getPoint().x));
			_textY = new JLabel(String.format("%.2f", phalange.getPoint().y));

			this.setLayout(new GridBagLayout());

			GridBagConstraints constraints = new GridBagConstraints();
			constraints.anchor = GridBagConstraints.WEST;
			constraints.insets = new Insets(5, 5, 5, 5);

			constraints.gridx = 0;
			constraints.gridy = 0;
			add(_labelAngle, constraints);

			constraints.gridx = 1;
			add(_textAngle, constraints);

			constraints.gridx = 2;
			add(_labelX, constraints);

			constraints.gridx = 3;
			add(_textX, constraints);
			constraints.gridx = 4;
			add(_labelY, constraints);

			constraints.gridx = 5;
			add(_textY, constraints);

			if (phalange.hasSlider()) {
				_sliderAngle = new JSlider(JSlider.HORIZONTAL,
						SLIDER_MIN, 90, SLIDER_MIN);

				_sliderAngle.addChangeListener(this);

				_sliderAngle.setMajorTickSpacing(10);
				_sliderAngle.setMinorTickSpacing(1);
				_sliderAngle.setPaintTicks(true);

				constraints.gridx = 0;
				constraints.gridy = 1;
				constraints.gridwidth = 6;
				constraints.anchor = GridBagConstraints.CENTER;
				add(_sliderAngle, constraints);
			}

			setBorder(BorderFactory.createTitledBorder(
					BorderFactory.createEtchedBorder(), title));
		}


		public void addArduinoController(IArduinoControl controller) {
			_arduinoController = controller;

		}

		public Dimension getPreferredSize() {
			return new Dimension(PREF_W, PREF_H);
		}

		@Override
		public void stateChanged(ChangeEvent ce) {			
			int sliderValue = ((JSlider) ce.getSource()).getValue();


			//Sending the command to the Arduino

			_arduinoController.rotate(this,  _deltaSlider - sliderValue);		
			_deltaSlider = sliderValue;

			double angle = _stepValue * sliderValue;

			_textAngle.setText(String.format("%.2f", angle));
			_phalange.setAngle(angle * RADIAN_TO_DEGREES);
		}

		@Override
		public void actionPerformed(ActionEvent e) {

		}

		@Override
		public void phalangeUpdated() {
			_textX.setText(String.format("%.2f", _phalange.getPoint().x));
			_textY.setText(String.format("%.2f", _phalange.getPoint().y));			
		}


		@Override
		public void setStep(int step) {
			this._sliderAngle.setValue(step);
		}

		@Override
		public Double getPoint() {
			return _phalange.getPoint();
		}


		@Override
		public void setStepValue(double stepValue) {	
			this._sliderAngle.setValue(0);			
			_stepValue = stepValue;			
			_sliderAngle.setMaximum((int)(90 / stepValue));
		}

		@Override
		public int getStep() {
			return _sliderAngle.getValue();			
		}		
	}

	/*
	 * JFrame used to control the settings of the simulation.
	 * Uses fazecast jSerialComm to handle the Serial Port operations.
	 * Listens to updates from the Phalange controller to send commands to the Arduino.
	 * Contains controls for simulations, changing microstep values to match the Arduino
	 * and linking the controls to the Arduino.
	 */

	private static class ControlPanel extends JPanel implements ActionListener, ItemListener, IArduinoControl {

		//Global Declarations
		private static final long serialVersionUID = -2374712028528866941L;
		private static final int[] MICROSTEP_VALUES = new int[] {1, 2, 4, 8, 16};	//Microstep values of the controller

		private IPhalangeControl _middleControl, _proximalControl;		

		private double _stepValue;
		private int _maxStep;		


		private SimulationMode _mode;				//Mode to change how the controls act.

		private boolean _isConnected;
		private boolean _isLinked;

		private Point2D.Double[][] _mappedPoints;	//Entire map of each possible point of the finger.

		private SerialPort _arduinoPort;			//Connection port

		private SimulationController _simulator;	//Simulator Control run on a separate thread
		private ArduinoCommunication _arduino;		//Communication Control run on a separate thread

		//Button Panel Items

		private JButton _buttonSimulation;

		//Settings Panel Items		

		private JCheckBox _checkBoxLinked;
		private JComboBox<String> _comboStepValue;

		//Connection Panel Items	

		private JLabel _labelIsConnected;
		private JComboBox<String> _comboComPorts;
		private JButton _buttonConnect;

		/*
		 * Runs through every possible orientation of angle combinations to
		 * map out the entire movement of the finger.
		 * Updates on a microstep change
		 */
		private void calculateMap() {
			double proximalAngle = 0, middleAngle = 0;
			double xOffset = 0, yOffset = 0;

			int maxMap = _maxStep + 1;
			_mappedPoints = new Point2D.Double[maxMap][maxMap];

			double radianStep = _stepValue * RADIAN_TO_DEGREES;

			for (int p = 0; p < maxMap; p++) {
				xOffset = PROXIMAL_LENGTH * Math.sin(proximalAngle);
				yOffset = PROXIMAL_LENGTH * Math.cos(proximalAngle);
				middleAngle = proximalAngle;
				for (int m = 0; m < maxMap; m++) {					
					_mappedPoints[p][m] = new Point2D.Double(
							xOffset + MIDDLE_LENGTH * Math.sin(middleAngle) + DISTAL_LENGTH * Math.sin(middleAngle + radianStep),
							yOffset + MIDDLE_LENGTH * Math.cos(middleAngle) + DISTAL_LENGTH * Math.cos(middleAngle + radianStep));
					middleAngle += radianStep;
				}
				proximalAngle += radianStep;
			}			
		}

		private void setMicroStep(int value) {
			_simulator.setMicroStep(value);			

			_stepValue = MOTOR_STEP / value;
			_maxStep = (int)(90 / _stepValue);

			calculateMap();

			_middleControl.setStepValue(_stepValue);
			_proximalControl.setStepValue(_stepValue);
		}

		private void portDisconnect() {
			if (_arduinoPort.closePort() || !_arduinoPort.isOpen()) {
				_labelIsConnected.setText("Disconnected");
				_buttonConnect.setText("Connect");
				_isConnected = false;
				_arduinoPort = null;

				_checkBoxLinked.setSelected(false);

				_checkBoxLinked.setEnabled(false);
				_comboComPorts.setEnabled(true);
			}
		}

		private void portConnect() {
			SerialPort serials[] = SerialPort.getCommPorts();
			int i = this._comboComPorts.getSelectedIndex();
			if (serials[i].openPort()) {
				_labelIsConnected.setText("Connected");
				_buttonConnect.setText("Disconnect");
				_isConnected = true;	
				_arduinoPort = serials[i];
				_arduino = new ArduinoCommunication(_arduinoPort);
				_arduinoPort.setBaudRate(115200);

				_checkBoxLinked.setEnabled(true);
				_comboComPorts.setEnabled(false);
			}	
		}

		private void simulation() {
			_mode = SimulationMode.SIMULATION;		//Redundant at the moment since the simulation is ran asynchronously

			int middleMax = (int)(69 / _stepValue);
			int proximalStep = _maxStep;
			int middleStep = 0;

			int stepCount;

			double nextY;			

			//Reset Middle/Distal to be straight
			stepCount = _middleControl.getStep() - 1;

			for (int i = stepCount; i >= 0; i--)
				_simulator.addCommand(_middleControl, i);


			//Reset Proximal to be flat
			stepCount = _proximalControl.getStep() + 1;

			for (int i = stepCount; i <= _maxStep; i++)
				_simulator.addCommand(_proximalControl, i);				


			while (middleStep < middleMax) {
				nextY =_mappedPoints[proximalStep][middleStep + 1].getY();
				if (nextY < 0) {
					proximalStep--;
					_simulator.addCommand(_proximalControl, proximalStep);				
				}
				else {
					middleStep++;
					_simulator.addCommand(_middleControl, middleStep);
				}				
			}			
			_mode = _isLinked ? SimulationMode.LINKED : SimulationMode.NORMAL;	
		}

		public ControlPanel() {
			_isConnected = false;
			_isLinked = false;

			_mode = SimulationMode.NORMAL;

			_simulator = new SimulationController(1);

			//Button Panel Setup

			JPanel _buttonPanel = new JPanel();

			_buttonSimulation = new JButton("Run Simulation");			

			_buttonSimulation.setPreferredSize(new Dimension(210, 70));
			_buttonPanel.add(_buttonSimulation);

			_buttonPanel.setPreferredSize(new Dimension(222, 100));

			_buttonPanel.setBorder(BorderFactory.createTitledBorder("Simulations"));			

			//Setting Panel Setup

			GridBagConstraints constraints = new GridBagConstraints();
			constraints.anchor = GridBagConstraints.WEST;
			constraints.insets = new Insets(5, 5, 5, 5);

			JPanel _settingsPanel = new JPanel(new GridBagLayout());

			_comboStepValue = new JComboBox<String>();	

			for (int v : MICROSTEP_VALUES)
				_comboStepValue.addItem(String.valueOf(v));

			JLabel _labelMicroStep = new JLabel("Microstep:");
			JLabel _labelLinked = new JLabel("Linked:");
			_checkBoxLinked = new JCheckBox();
			_checkBoxLinked.setEnabled(false);

			constraints = new GridBagConstraints();

			constraints.fill = GridBagConstraints.HORIZONTAL;
			constraints.insets = new Insets(5, 5, 5, 5);
			constraints.gridx = 0;
			constraints.gridy = 0;

			constraints.anchor = GridBagConstraints.WEST;
			_settingsPanel.add(_labelMicroStep, constraints);

			constraints.gridy++;
			_settingsPanel.add(_labelLinked, constraints);

			constraints.anchor = GridBagConstraints.EAST;

			constraints.gridx++;
			constraints.gridy = 0;
			_settingsPanel.add(_comboStepValue, constraints);

			constraints.gridy++;
			_settingsPanel.add(_checkBoxLinked, constraints);

			_settingsPanel.setPreferredSize(new Dimension(190, 100));

			_settingsPanel.setBorder(BorderFactory.createTitledBorder("Settings"));

			//Connection Panel Setup

			JPanel _connectionPanel = new JPanel(new GridBagLayout());			

			JLabel _labelConnected = new JLabel("Status:");
			JLabel _labelComPort = new JLabel("Port:");

			_labelIsConnected = new JLabel("Not Connected");			
			_buttonConnect = new JButton("Connect");

			_comboComPorts = new JComboBox<String>();			

			SerialPort serials[] = SerialPort.getCommPorts();
			for (SerialPort serial : serials)
				_comboComPorts.addItem(serial.getDescriptivePortName());			

			constraints = new GridBagConstraints();
			constraints.anchor = GridBagConstraints.WEST;
			constraints.insets = new Insets(5, 5, 5, 5);

			constraints.gridx = 0;
			constraints.gridy = 0;
			_connectionPanel.add(_labelConnected, constraints);

			constraints.gridy = 1;
			_connectionPanel.add(_labelComPort, constraints);

			constraints.gridx = 1;
			constraints.gridy = 0;
			_connectionPanel.add(_labelIsConnected, constraints);

			constraints.gridy = 1;
			_connectionPanel.add(_comboComPorts, constraints);

			constraints.gridx = 0;
			constraints.gridy = 2;
			constraints.gridwidth = GridBagConstraints.REMAINDER;
			constraints.anchor = GridBagConstraints.CENTER;
			constraints.fill = GridBagConstraints.HORIZONTAL;
			_connectionPanel.add(_buttonConnect, constraints);

			_connectionPanel.setPreferredSize(new Dimension(254, 100));

			_connectionPanel.setBorder(BorderFactory.createTitledBorder("Arduino Connection"));			

			//Event Setup

			_buttonSimulation.addActionListener(this);

			_buttonConnect.addActionListener(this);			
			_comboStepValue.addActionListener(this);			
			_checkBoxLinked.addItemListener(this);	

			add(_buttonPanel, BorderLayout.LINE_START);
			add(_settingsPanel, BorderLayout.CENTER);
			add(_connectionPanel, BorderLayout.LINE_END);

			setBorder(BorderFactory.createTitledBorder(
					BorderFactory.createEtchedBorder(), "Controls"));
		}

		public void addPhalangeControllers(IPhalangeControl middle, IPhalangeControl proximal) {	
			_middleControl = middle;
			_proximalControl = proximal;			
			_stepValue = MOTOR_STEP;			
			setMicroStep(1);
		}

		/*
		 * Organises the data for sending.
		 * Data is sent to the Arduino Communication controller.
		 * All data is shown as their bit values for ease of reading.
		 * Data is sent in pairs of 8-byte chunks.
		 */
		public void sendCommand(int mSteps, int mFeed, boolean mDirection, boolean mEnabled,
				int pSteps, int pFeed, boolean pDirection, boolean pEnabled) {

			System.out.print("MDirection:" );
			System.out.println(mDirection);
			System.out.print("PDirection:" );
			System.out.println(pDirection);


			byte mData = 0b01000000;		//Middle Motor Command
			byte pData = 0;					//Proximal Motor Command

			mData |= 0b10000000 | mFeed;
			pData |= 0b10000000 | pFeed;

			if (mEnabled) {
				mData |= 0b00100000;		//Enable Middle Motor (00n0 0000)
				if (mDirection)
					mData |= 0b00010000;	//Direction (000n 0000)
			}
			if (pEnabled) {
				pData |= 0b00100000;		//Enable Proximal Motor (00n0 0000)
				if (pDirection)
					pData |= 0b00010000;	//Direction (000n 0000)
			}

			byte[] buffer = new byte[4];

			int i = 0;

			buffer[i++] = mData;
			buffer[i++] = (byte)Math.min(mSteps, 255); //Avoid commands over 255 steps in size
			buffer[i++] = pData;
			buffer[i++] = (byte)Math.min(pSteps, 255); //Avoid commands over 255 steps in size

			_arduino.writeData(buffer);

			//If a command was over 255 steps, it is split and the remaining movements will be sent.
			if ((mSteps > 255 && pSteps > 255))
				sendCommand((mSteps - 255), mFeed, mDirection, mEnabled,
						(pSteps - 255), pFeed, pDirection, pEnabled);
		}

		/*
		 * Main event handler for buttons and combo boxes.
		 */
		@Override
		public void actionPerformed(ActionEvent e) {
			Object source = e.getSource();			
			if (source.equals(this._buttonConnect))
				if (_isConnected)
					portDisconnect();					
				else 
					portConnect();

			else if (source == this._buttonSimulation)
				simulation();		
			else if (source == this._comboStepValue)
				setMicroStep(MICROSTEP_VALUES[_comboStepValue.getSelectedIndex()]);

		}

		/*
		 * Main event handler for Check boxes.
		 */
		@Override
		public void itemStateChanged(ItemEvent e) {
			if(e.getStateChange() == ItemEvent.SELECTED) {			
				_mode = SimulationMode.LINKED;
				_isLinked = true;
			}
			else {
				_mode = SimulationMode.NORMAL;
				_isLinked = false;
			};
		}

		/*
		 * Listens to the IPhalangeControl for updates from the control sliders.
		 * All controls go through the sliders, and will then get sent here for
		 * to then be sent to the Arduino if one is connected.
		 */
		@Override
		public void rotate(IPhalangeControl phalange, int angle) {
			if ((_mode == SimulationMode.LINKED || _mode == SimulationMode.SIMULATION) && 
					(_isConnected && _arduinoPort.isOpen()) && Math.abs(angle) > 0) {				
				int steps = Math.abs(angle);
				sendCommand(steps, 1, angle < 0, true, 
						steps, 1, angle > 0, phalange == _proximalControl);		
			}
		} 

	}

	public FingerSimulator() {
		super("Robotic Finger Simulation");

		ControlPanel controlPanel;
		DataPanel distalData;
		DataPanel middleData;
		DataPanel proximalData;
		SimulationPanel simulationPanel;
		Finger finger;

		//Setting up the main Panels
		finger = new Finger();

		controlPanel = new ControlPanel();
		simulationPanel = new SimulationPanel(finger);
		distalData = new DataPanel("Distal Data", finger.getDistal());
		middleData = new DataPanel("Middle Data", finger.getMiddle());
		proximalData = new DataPanel("Proximal Data", finger.getProximal());


		//Linking controls;		
		middleData.addArduinoController(controlPanel);
		proximalData.addArduinoController(controlPanel);
		controlPanel.addPhalangeControllers(middleData, proximalData);


		// Adding the Panels to the Frame
		add(controlPanel, BorderLayout.PAGE_START);		
		add(distalData, BorderLayout.LINE_END);
		add(middleData, BorderLayout.CENTER);
		add(proximalData, BorderLayout.LINE_START);		
		add(simulationPanel, BorderLayout.PAGE_END);


		setDefaultCloseOperation(WindowConstants.EXIT_ON_CLOSE);

		pack();
		setLocationRelativeTo(null);
	}

	public static void main(String[] args) {
		try {
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		} catch (Exception ex) {
			ex.printStackTrace();
		}

		SwingUtilities.invokeLater(new Runnable() {
			@Override
			public void run() {
				new FingerSimulator().setVisible(true);
			}
		});
	}
}
