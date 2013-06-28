package es.uned.dia.examples.interoperate;

import es.uned.dia.interoperate.ExternalApp;
import es.uned.dia.interoperate.matlab.jimc.MatlabExternalApp;
import javax.swing.*;
import javax.swing.event.*;
import java.awt.*;
import java.awt.geom.*;
import java.awt.event.*;

/**
 * Example of use of the high level protocol and plot
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 *
 */
public class MatlabHighLevelInteractiveExample {

  //Plot variables
  JSlider slider;
  JButton buttonPlay,buttonPause;
  JTextField functionText;
  JLabel sliderLabel;
  PlotPanel plot;
  int transX,oldtransX=0;

  Point2D point1=new Point2D.Double(0, 0);
  Point2D point2=new Point2D.Double(0, 0);
  Line2D line=new Line2D.Double(point1, point2);

  int sizeX=300;
  int sizeY=(int)((sizeX+150)/1.62);

  boolean copy=false;
  int rigthBorder=sizeX-20;
  int ini=0,fin=rigthBorder;  
  int origin=0;
  double scaledTime=0,scaledValue=0;
  double oldTime=0;
  double actualDt=0.1;
  int delta=0;


  double dt=0.04;   
  int minFreq=0;
  int maxFreq=200;

  // Declare local variables
  public double time=0, frequency=1, value=0;     
  ExternalApp externalApp;
  public boolean pauseSimulation=false;

  public static void main (String[] args) {
    new MatlabHighLevelInteractiveExample();
  }

  public MatlabHighLevelInteractiveExample() {    
    //Configurate frame
    createFrame();    
    // Create the external application
    externalApp = new MatlabExternalApp();
    // Set the client application  
    externalApp.setClient(this);
    // Link variables with the external app's
    externalApp.linkVariables("time", "t");     
    externalApp.linkVariables("frequency", "f");
    externalApp.linkVariables("value", "y");
    // Configure the external application 
    externalApp.setCommand("y=sin(2*pi*f*t)*cos(t)");        
    // Start the connection
    if (!externalApp.connect()) {                 
      System.err.println ("ERROR: Could not connect!");
      return;
    }       
    //Perform the simulation
    do{         
      if (!pauseSimulation){                  
        externalApp.step(1);                                                                                  
        //Normalizes variables    
        scaledTime=sizeX*time/10.0;
        scaledValue=(150-60.0*value)/2.0;                           
        //Create a trace for the function
        point1=point2;      
        point2 = new Point2D.Double(scaledTime,scaledValue);  
        line = new Line2D.Double(point1, point2);      
        //check border of plot
        checkBorders();                    
        //Paint plot
        plot.repaint();        
        //Increase time
        time=time+dt;               
      }        
      //Add a delay of 10ms   
      delay(5);                      
    }while(true);                              
  } 

  public void createFrame(){
    //Create frame and panels
    JFrame frame=new JFrame("Plotting an External Function");       
    JPanel mainPanel=new JPanel(new BorderLayout());
    JPanel downPanel=new JPanel(new BorderLayout());
    JPanel buttonPanel=new JPanel(new BorderLayout());

    //Create visual elements                    
    plot=new PlotPanel();                           
    slider =new JSlider(minFreq,maxFreq,(int)(frequency*100.0));                        
    functionText= new JTextField("y=sin(2*pi*f*t)*cos(t)");                         
    buttonPlay = new JButton("Play");
    buttonPause = new JButton("Pause");     
    sliderLabel = new JLabel("Frequency (Hz)", JLabel.CENTER);                        

    //Create layout  
    buttonPanel.add(buttonPause,BorderLayout.EAST);
    buttonPanel.add(buttonPlay,BorderLayout.WEST);
    buttonPanel.add(functionText,BorderLayout.CENTER);        
    downPanel.add(sliderLabel,BorderLayout.NORTH);    
    downPanel.add(slider,BorderLayout.CENTER);
    downPanel.add(buttonPanel,BorderLayout.SOUTH);
    mainPanel.add(downPanel,BorderLayout.SOUTH);  
    mainPanel.add(plot,BorderLayout.CENTER);        
    frame.getContentPane().add(mainPanel);

    //Add interaction to the slider
    slider.addChangeListener(new ChangeListener() {
      public void stateChanged(ChangeEvent ce){             
        if (!slider.getValueIsAdjusting())        
          frequency=slider.getValue()/100.0;                
      }   
    });       

    //Add interaction to the text field     
    ActionListener alText = new ActionListener() {
      public void actionPerformed(ActionEvent actionEvent) {
        externalApp.setCommand(functionText.getText());
      }
    };    
    functionText.addActionListener(alText);


    //Add interaction to Play
    ActionListener alButtonPlay = new ActionListener() {
      public void actionPerformed(ActionEvent actionEvent) {
        pauseSimulation=false;
      }
    };
    buttonPlay.addActionListener(alButtonPlay);

    //Add interaction to Pause
    ActionListener alButtonPause = new ActionListener() {
      public void actionPerformed(ActionEvent actionEvent) {
        pauseSimulation=true;
      }
    };
    buttonPause.addActionListener(alButtonPause);

    //Add interaction for frame
    frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE); 
    frame.addWindowListener(new WindowAdapter() {
      public void windowClosing(WindowEvent evt) {
        // Exit the application
        externalApp.disconnect();
        System.exit(0); 
      }
    });

    //Configure slider    
    slider.setMajorTickSpacing(maxFreq/2);
    slider.setMinorTickSpacing(20);
    slider.setPaintTicks(true);
    slider.setPaintLabels(true);    

    //Configure frame
    frame.setResizable(false);   
    frame.setSize(sizeX+5,sizeY);       
    frame.setLocationRelativeTo(null);
    frame.setVisible(true); 
  }

  public void checkBorders(){
    //Update borders and origin of the plot                                      
    if (scaledTime>=rigthBorder) {                                  
      origin=origin-delta;
      if (origin+scaledTime<rigthBorder) delta=0;
      else delta=(int)(dt*sizeX+1);                   
      ini=ini+delta;
      fin=fin+delta;        
    } 
  }

  //Class PlotPanel
  public class PlotPanel extends Canvas{        
    private static final long serialVersionUID = 1L;
    public void update (Graphics g) {     
      //Get graphic object
      Graphics2D g2D=(Graphics2D)g;

      //Customize the trace
      g2D.setRenderingHint(RenderingHints.KEY_ANTIALIASING, 
          RenderingHints.VALUE_ANTIALIAS_ON);     
      g2D.setStroke (new BasicStroke (2.0F, BasicStroke.CAP_BUTT,
          BasicStroke.JOIN_BEVEL));                       
      g2D.setColor(java.awt.Color.BLUE);  

      //Draw the trace
      g2D.translate(origin, 0);
      ((Graphics2D)g).draw(line);

      //Move the trace
      if (delta>0) g2D.copyArea(ini,0,fin,sizeY,-delta,0);                                              
    }
  }

  //Method for adding a pause
  public void delay(long d){
    try {
      Thread.sleep( d );      
    } catch( InterruptedException e ) {
      System.out.println( e );
    }
  } 
}
