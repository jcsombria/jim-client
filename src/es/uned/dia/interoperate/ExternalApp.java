/**
 * 
 */
package es.uned.dia.interoperate;

/**
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 * @author <a href="mailto:fem@um.es">Francisco Esquembre</a> 
 * 
 * <H4>
 * Example of use
 * </H4>
* <pre>
* {@code
* import es.uned.dia.interoperate.ExternalApp;
* import es.uned.dia.interoperate.matlab.jimc.MatlabExternalApp;

* public class test {
*  public static void main (String[] args) {
*    // Declare local variables 
*    double time=0, frequency=1, value=0;
*    // Create the ExternalApplication
*    ExternalApp externalApp = new MatlabExternalApp();
*    // Start the connection
*    if (!externalApp.connect()) {                 
*      System.err.println ("ERROR: Could not connect!");
*      return;
*    }
*    // Set the frequency
*    externalApp.setValue("f",frequency);
*    // Perform the simulation
*    do {                                          
*      externalApp.setValue("t",time);
*      externalApp.eval("y=sin(2*pi*f*t)*cos(t)");
*      value=externalApp.getDouble("y");
*      System.out.println("time:"+time+" value:"+value);
*      time=time+0.1;
*    } while (time<=10);
*    // Finish the connection 
*    externalApp.disconnect();                           
*  }
*}
* }
* </pre>
* <H4>
* Compile and Run the example
* </H4>
* <p>
* javac -cp jimc.jar test.java
* <br>
* java  -cp jimc.jar;. test
* </p>
*/
public interface ExternalApp {
  
  static final public String CONNECTION_OK="Connection Sucessful";
  static final public String CONNECTION_FAILED="Connection Failed";
  static final public String connectionAuthenticationRequired="Authentication Required";
  static final public String connectionAuthenticationFail="Authentication Failed";
  static final public String connectionSlotFail="No Time Slot";
  static final public String connectionNoServer="Server Off";
  static final public String connectionNoModel="Model Not Found";
  static final public int connectionTimeOut=20000; 
  
/*********************
  Low level protocol
**********************/
  
//--------------------------------------------------------------
//Connection and configuration
//--------------------------------------------------------------
 /**
  * Starts the connection with the external application  
  * @return boolean true if the connection was successful
  */    
 public boolean connect();

 /**
  * Finishes the connection with the external application     
  */         
 public void disconnect();

 /**
  * Checks if the connection was successful
  * @return boolean true if the connection was successful
  */
 public boolean isConnected(); 

 /**
  * Accepts an initialization command to use whenever the system is reset
  * @param command String
  */
 public void setInitCommand (String command);

 /**
  * Gets the initialization command
  * @return String the initial command
  */ 
 public String getInitCommand();

//--------------------------------------------------------------
//Setting and getting values
//--------------------------------------------------------------

 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value String the desired value
  */
 public void setValue(String variable, String value);
 
 /**
  * Gets the value of a String variable of the application
  * @param variable String the variable name
  * @return String the value
  */
 public String getString (String variable);
 
 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value double the desired value
  */ 
 public void setValue(String variable, double value);
 
 /**
  * Gets the value of a double variable of the application
  * @param variable String the variable name
  * @return double the value
  */
 public double getDouble (String variable);

 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value double[] the desired value
  */
 public void setValue(String variable, double[] value);

 /**
  * Gets the value of a double[] variable of the application
  * @param variable String the variable name
  * @return double the value
  */
 public double[] getDoubleArray (String variable);

 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value double[][] the desired value
  */
 public void setValue(String variable, double[][] value);

 /**
  * Gets the value of a double[][] variable of the application
  * @param variable String the variable name
  * @return double the value
  */
 public double[][] getDoubleArray2D (String variable); 

//--------------------------------------------------------------
//Control commands
//--------------------------------------------------------------
  
 /**
  * Evaluates a given command in the external application
  * @param command String to be executed
  */
 public void eval (String command);

 /**
  * Resets the application
  */
 public void reset();

 /**
  * The result of last action can be read using
  * this method.   
  * @return String the result of last action
  */  
 public String getActionResult(); 
  
//--------------------------------------------------------------
//Other low level methods
//-------------------------------------------------------------- 
 
 
/*********************
  High level protocol
**********************/
  
//--------------------------------------------------------------
//Linking variables
//--------------------------------------------------------------
 
 /**
  * Sets the client application
  * @param clientApp Object the client application. Reflection is used to access the variables in the client.
  * @return boolean false if the client is null
  */
 public boolean setClient(Object clientApp);

 /**
  * Links a client variable with a variable of the external application
  * @param clientVariable String the client variable
  * @param externalVariable String the external variable
  */
 public boolean linkVariables(String clientVariable, String externalVariable);

 /**
  * Clears all linking between variables
  */
 public void clearLinks();
 
//--------------------------------------------------------------
//Control commands
//--------------------------------------------------------------
 
 /**
  * Some external applications, such as MATLAB, can continuously execute  
  * a command after every step().
  * @param command a String to be executed
  */
 public void setCommand(String command);

 /**
  * Gets the command to be executed by the external application.  
  * @return String the command
  */ 
 public String getCommand();

 /**
  * Steps the application a given step or a number of times.
  * If getCommand() is non-null the command is executed that number of times.
  * If getCommand() is null, the dt parameter is passed down to the externall application,
  * and the actual meaning of this parameter dt will depend on the implementing class
  * @param dt double
  */
 public void step (double dt);

 /**
  * Synchronizes client and external applications
  */
 public void synchronize();
 
 
//--------------------------------------------------------------
//Remote methods
//--------------------------------------------------------------
 
 /**
  * Sets the package size used to group valus of the external variables
  */
 public void setPackageSize(int size);

 /**
  * Empty the buffer
  */
 public void flush();
  
//--------------------------------------------------------------
//Other high level methods
//--------------------------------------------------------------

 /**
  * Forces or not to the client to wait until an external variable is read. 
  * @param wait boolean 
  */
 public void setWaitForEver(boolean wait);

}
