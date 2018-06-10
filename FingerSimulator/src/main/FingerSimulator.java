package main;
//package components;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import java.util.ArrayList;
import java.util.List;

import java.awt.geom.Point2D;


public class FingerSimulator extends JFrame {
	
	/**
	 * 
	 */
	private static final long serialVersionUID = -5872034986175365620L;
	private static final double MOTOR_STEP = 1.8; //In degrees	
	private static final int SCALER = 2;
	private static final double RADIAN_TO_DEGREES = Math.PI / 180;
	private static double _stepValue = MOTOR_STEP;
	private static int _maxStep = (int)(90 / _stepValue);
	
	private ControlPanel _controlPanel;
	private DataPanel _distalData;
	private DataPanel _middleData;
	private DataPanel _proximalData;
	private SimulationPanel _simulationPanel;
	private Finger _finger;
	
	private interface IPhalangeListener { 
		void phalangeUpdated();
	}
	
	private class Phalange {
		
		private Phalange _joint;
		private List<IPhalangeListener> _listeners;
		
		private double _length;
		private Point2D.Double _point;
		private Point2D.Double _offsetPoint;
		private double _angle;
		private double _offsetAngle;
		
		private boolean _hasSlider;
		private boolean _linkedPrevious;
		
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
		
		public double getLength() {
			return _length;
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
	
	private class Finger {
		
		private static final double DISTAL_LENGTH = 23.0F * SCALER;
		private static final double MIDDLE_LENGTH = 38.0F * SCALER;
		private static final double PROXIMAL_LENGTH = 75.0F * SCALER;
		
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
			//Graphics g = this.getGraphics();
			
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
	
	
	
	private static class DataPanel extends JPanel implements ActionListener, ChangeListener, IPhalangeListener {
		
		private static final long serialVersionUID = -704085048705110485L;

		private static final int SLIDER_MIN = 0;
		
		private static final int PREF_W = 222;
		private static final int PREF_H = 87;
		
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
						SLIDER_MIN, _maxStep, SLIDER_MIN);
				
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
		
		public Dimension getPreferredSize() {
			return new Dimension(PREF_W, PREF_H);
		}
		
		@Override
        public void stateChanged(ChangeEvent ce) {
			
			int v = ((JSlider) ce.getSource()).getValue();
			double a = _stepValue * v;
			
			_textAngle.setText(String.format("%.2f", a));
			_phalange.setAngle(a * RADIAN_TO_DEGREES);			
        }
		
		@Override
		public void actionPerformed(ActionEvent e) {
			
		}

		@Override
		public void phalangeUpdated() {
			_textX.setText(String.format("%.2f", _phalange.getPoint().x));
			_textY.setText(String.format("%.2f", _phalange.getPoint().y));			
		} 
		
	}
	
	private static class ControlPanel extends JPanel implements ActionListener {

		private static final long serialVersionUID = -2374712028528866941L;

		public ControlPanel() {
			setBorder(BorderFactory.createTitledBorder(
	                BorderFactory.createEtchedBorder(), "Controls"));
		}
		
		@Override
		public void actionPerformed(ActionEvent e) {
			// TODO Auto-generated method stub
		} 
		
	}
	
    public FingerSimulator() {
		super("Robotic Finger Simulation");
		  
		//Setting up the main Panels
		_finger = new Finger();
		_controlPanel = new ControlPanel();
		_simulationPanel = new SimulationPanel(_finger);
		_distalData = new DataPanel("Distal Data", _finger.getDistal());
		_middleData = new DataPanel("Middle Data", _finger.getMiddle());
		_proximalData = new DataPanel("Proximal Data", _finger.getProximal());
		
		
		// Adding the Panels to the Frame
		add(_controlPanel, BorderLayout.PAGE_START);
		add(_simulationPanel, BorderLayout.PAGE_END);
		add(_distalData, BorderLayout.LINE_END);
		add(_middleData, BorderLayout.CENTER);
		add(_proximalData, BorderLayout.LINE_START);
		
		
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
