package main;
//package components;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;


public class FingerSimulator extends JFrame {
	
	private Finger _finger;
	private interface IPhalange {
		void updateAngle(double angle);
	}
	
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
			
			//_proximal.setAngle(1.25);
			//_middle.setAngle(0.75);
			//_distal.setAngle(2);
			
			_proximal.updatePoint();
		}
		
	}
	
	private static class SimulationPanel extends JPanel implements IPhalangeListener {
		
		private static final int PREF_W = 600;
		private static final int PREF_H = 600;


		public SimulationPanel() {
			// TODO Auto-generated constructor stub
		}

		protected void paintComponent(Graphics g) {
			super.paintComponent(g);
	}

		public Dimension getPreferredSize() {
			return new Dimension(PREF_W, PREF_H);
		}
	}
	
	private static class DataPanel extends JPanel implements ActionListener {
		
		//PLACE HOLDERS
		//TODO Remove
		private static final int SLIDER_MIN = 0;
		private static final int SLIDER_MAX = 30;
		
		private JLabel _labelAngle;
		private JLabel _labelX;
		private JLabel _labelY;
		private JTextField _textAngle;
		private JTextField _textX;
		private JTextField _textY;
		
		private JSlider sliderAngle;
		

		public DataPanel(String title) {
			
			//Initialize Components
			_labelAngle = new JLabel("Angle");
			_labelX = new JLabel("X");
			_labelY = new JLabel("Y");
			_textAngle = new JTextField(5);
			_textX = new JTextField(5);
			_textY = new JTextField(5);
			
			sliderAngle = new JSlider(JSlider.HORIZONTAL,
					SLIDER_MIN, SLIDER_MAX, SLIDER_MIN);
			
			//Turn on labels at major tick marks.
			sliderAngle.setMajorTickSpacing(10);
			sliderAngle.setMinorTickSpacing(1);
			sliderAngle.setPaintTicks(true);
			
			this.setLayout(new GridBagLayout());
			
			GridBagConstraints constraints = new GridBagConstraints();
	        constraints.anchor = GridBagConstraints.WEST;
	        constraints.insets = new Insets(5, 5, 5, 5);
	         
	        // add components to the panel
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
	         
	        constraints.gridx = 0;
	        constraints.gridy = 1;
	        constraints.gridwidth = 6;
	        constraints.anchor = GridBagConstraints.CENTER;
	        add(sliderAngle, constraints);
	         
			
			setBorder(BorderFactory.createTitledBorder(
	                BorderFactory.createEtchedBorder(), title));
		}
		
		@Override
		public void actionPerformed(ActionEvent e) {
			// TODO Auto-generated method stub
		} 
		
	}
	
	private static class ControlPanel extends JPanel implements ActionListener {

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
		ControlPanel controlPanel = new ControlPanel();
		DataPanel distalData = new DataPanel("Distal Data");
		DataPanel middleData = new DataPanel("Middle Data");
		DataPanel proximalData = new DataPanel("Proximal Data");
		SimulationPanel simulationPanel = new SimulationPanel();
		
		// Adding the Panels to the Frame
		add(controlPanel, BorderLayout.PAGE_START);
		add(distalData, BorderLayout.LINE_START);
		add(middleData, BorderLayout.CENTER);
		add(proximalData, BorderLayout.LINE_END);
		add(simulationPanel, BorderLayout.PAGE_END);
		
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
