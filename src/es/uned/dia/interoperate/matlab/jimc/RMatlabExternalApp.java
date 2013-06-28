package es.uned.dia.interoperate.matlab.jimc;

import es.uned.dia.interoperate.ExternalApp;
import java.io.*;

import java.lang.reflect.*;
import java.net.Socket;

/**
 * Class to implement the communication protocol with a remote MATLAB
 * @author <a href="mailto:gfarias@bec.uned.es">Gonzalo Farias</a>
 * @author <a href="mailto:fem@um.es">Francisco Esquembre</a> 
 */
public class RMatlabExternalApp implements ExternalApp {

  //protected boolean needsToExtract=false; // true if any file is not in place. Then it will work in Simulations.getTemporaryDir()
  protected long id=-1;   // The integer id for the Matlab engine opened  
  protected String initCommand = null; // An optional initialization command required for the correct reset

  boolean startRequired = true; // Whether the simulation needs to be started
  boolean waitForEverFlag=false; //Flag to wait for a variable
  protected String userDir=null; // The users working directory for this instance
  static String pathOfJimc=null;
  static String pathOfJmatlink=null;
  static String temporalDirectory=null;
  public String commandUpdate=" "; 
  protected String resultOfLastAction="";// A String with the result of the last method executed 

  protected Object varContextObject=null; // The client of the external application
  protected java.util.Vector<String[]> linkVector=null;// A Vector of Strings to keep information of the connected variables
  protected Field[] varContextFields=null; //An array of fields of the client
  public Field varContextField;
  public int[] linkIndex;
  public int[] linkType;
  static final int DOUBLE=0,ARRAYDOUBLE=1,ARRAYDOUBLE2D=2,STRING=3;
  
  protected Socket jimTCP; //The socket to connect with the remote application
  protected DataInputStream bufferInputTCP; // TCP buffer input
  protected DataOutputStream bufferOutputTCP; // TCP buffer input
  protected int SERVICE_PORT; // TCP port
  protected String SERVICE_IP; // TCP remote IP
  protected int EjsId; // EJS identifier
  protected boolean removeBuffer=true; // Whether the buffer has to be cleaned
  protected boolean asynchronousSimulation=false; // Flag to indicate the asynchronous mode
  protected boolean modeAS=false; //Asynchronous mode
  protected int packageSize=1; // Size of the package
  protected boolean connected=false; // Flag to indicate if the connection was successful
  protected boolean flushNow=true;
  
  
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
  public boolean connect (){
    return connect("",""); 
  }

 /**
  * Finishes the connection with the external application     
  */         
  public void disconnect (){
    connected=false;
    if (jimTCP!=null){    
      try {
        jimTCP.close();
      }catch (IOException ioe){System.out.println("Error " + ioe);
      }catch (Exception e2){System.out.println("Error " + e2);
      }
    }
  }
 
 /**
  * Checks if the connection was successful
  * @return boolean true if the connection was successful
  */
  public boolean isConnected(){ 
    return connected;
  }

 /**
  * Accepts an initialization command to use whenever the system is reset
  * @param command String
  */
 public void setInitCommand (String command){
   //process the command    
   initCommand = command.trim();
   initialize();     
 }

 /**
  * Gets the initialization command
  * @return String the initial command
  */ 
 public String getInitCommand(){
   return initCommand;
 }

//--------------------------------------------------------------
//Setting and getting values
//--------------------------------------------------------------

 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value String the desired value
  */
 public void setValue(String variable, String value){
   try {  
     if (!asynchronousSimulation){
       bufferOutputTCP.writeInt(EjsId);
       bufferOutputTCP.writeUTF("setValueString");
       bufferOutputTCP.writeUTF(variable);
       bufferOutputTCP.writeUTF(value);
       if (flushNow)
         bufferOutputTCP.flush();
     }
   }catch(Exception e){
     System.out.println("Error closing Remote Matlab "+e);
   }
 }
  
 /**
  * Gets the value of a String variable of the application
  * @param variable String the variable name
  * @return String the value
  */
 public String getString (String variable){ 
   if (asynchronousSimulation) return getStringAS();
   if (modeAS){
     haltUpdate(true);
   }
   String charArray = null;
   try{
     bufferOutputTCP.writeInt(EjsId);
     bufferOutputTCP.writeUTF("getString");
     bufferOutputTCP.writeUTF(variable);
     bufferOutputTCP.flush();
     charArray = bufferInputTCP.readUTF();
   }catch(Exception e){
     System.out.println("Error getString Remote Matlab "+e);
   }
   return (charArray);
 }
  
 
 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value double the desired value
  */ 
 public void setValue(String variable, double value){
   try{  
     if (!asynchronousSimulation){
       bufferOutputTCP.writeInt(EjsId);
       bufferOutputTCP.writeUTF("setValueDouble");
       bufferOutputTCP.writeUTF(variable);
       bufferOutputTCP.writeDouble(value);
       if (flushNow)
         bufferOutputTCP.flush();
     }
   }catch(Exception e){
     System.out.println("Error setValue Remote Matlab "+e);
   }
 }
  
 /**
  * Gets the value of a double variable of the application
  * @param variable String the variable name
  * @return double the value
  */
 public double getDouble (String variable){
   if (asynchronousSimulation) return getDoubleAS();
   if (modeAS){
     haltUpdate(true);
   }
   double valueDouble = 0.0;
   try{
     bufferOutputTCP.writeInt(EjsId);
     bufferOutputTCP.writeUTF("getDouble");
     bufferOutputTCP.writeUTF(variable);
     bufferOutputTCP.flush();
     valueDouble = bufferInputTCP.readDouble();
   }catch(Exception e){
     System.out.println("Error getDouble Remote Matlab "+e);
   }   
   return (valueDouble);
 }

 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value double[] the desired value
  */
 public void setValue(String variable, double[] value){   
   try{
     if (!asynchronousSimulation){
       bufferOutputTCP.writeInt(EjsId);
       bufferOutputTCP.writeUTF("setValueD[]");
       bufferOutputTCP.writeUTF(variable);
       bufferOutputTCP.writeInt(value.length);
       for (int i = 0; i < value.length; i++) {
         bufferOutputTCP.writeDouble(value[i]);
       }
       if (flushNow)
         bufferOutputTCP.flush();
     }
   }catch(Exception e){
     System.out.println("Error setValue Remote Matlab "+e);
   }   
 }

 /**
  * Gets the value of a double[] variable of the application
  * @param variable String the variable name
  * @return double the value
  */
 public double[] getDoubleArray (String variable){
   if (asynchronousSimulation) return getDoubleArrayAS();
   if (modeAS){
     haltUpdate(true);
   }
   int _dim1;
   double[] vectorDouble=null;
   try{
     bufferOutputTCP.writeInt (EjsId);
     bufferOutputTCP.writeUTF("getDoubleArray");
     bufferOutputTCP.writeUTF (variable);
     bufferOutputTCP.flush();
     _dim1=bufferInputTCP.readInt();
     vectorDouble=new double[_dim1];
     for(int i=0;i<_dim1;i++) { vectorDouble[i]=bufferInputTCP.readDouble();}
   }catch(Exception e){
     System.out.println("Error getDoubleArray Remote Matlab "+e);
   }   
   return(vectorDouble);
}

 /**
  * Sets the value of the given variable of the application
  * @param variable String the variable name
  * @param value double[][] the desired value
  */
 public void setValue(String variable, double[][] value){
   try{
     if (!asynchronousSimulation){
       bufferOutputTCP.writeInt(EjsId);
       bufferOutputTCP.writeUTF("setValueD[][]");
       bufferOutputTCP.writeUTF(variable);
       bufferOutputTCP.writeInt(value.length);
       bufferOutputTCP.writeInt(value[0].length);
       for (int i = 0; i < value.length; i++) {
         for (int j = 0; j < value[0].length; j++) {
           bufferOutputTCP.writeDouble(value[i][j]);
         }
       }
       if (flushNow)
         bufferOutputTCP.flush();
     }
   }catch(Exception e){
     System.out.println("Error setValue Remote Matlab "+e);
   }
 }

 /**
  * Gets the value of a double[][] variable of the application
  * @param variable String the variable name
  * @return double the value
  */
 public double[][] getDoubleArray2D (String variable){
   if (asynchronousSimulation) return getDoubleArray2DAS();
   if (modeAS){
     haltUpdate(true);
   }
   int _dim1,_dim2;
   double[][] arrayDouble=null;
   try{
     bufferOutputTCP.writeInt (EjsId);
     bufferOutputTCP.writeUTF ("getDoubleArray2D");
     bufferOutputTCP.writeUTF (variable);
     bufferOutputTCP.flush();
     _dim1=bufferInputTCP.readInt();
     _dim2=bufferInputTCP.readInt();
     arrayDouble= new double[_dim1][_dim2] ;
     for(int i=0;i<_dim1;i++) {
       for(int j=0;j<_dim2;j++){ arrayDouble[i][j]=bufferInputTCP.readDouble();}
     }
   }catch(Exception e){
     System.out.println("Error getDoubleArray2D Remote Matlab "+e);
   }   
   return(arrayDouble);
 } 

//--------------------------------------------------------------
//Control commands
//--------------------------------------------------------------
  
 /**
  * Evaluates a given command in the external application
  * @param command String to be executed
  */
 public void eval (String command){
   //synchronize(false);   
   removeBuffer=false;
   asynchronousSimulation=false;
   
   try {
     bufferOutputTCP.writeInt(EjsId);
     bufferOutputTCP.writeUTF("eval");
     bufferOutputTCP.writeUTF(command);
     if (flushNow)
       bufferOutputTCP.flush();
   }
   catch (Exception e) {
     System.out.println(" eval Remote Exception:" + e);
   }
 }

 /**
  * Resets the application
  */
 public void reset(){
   eval("bdclose ('all')");
   eval("clear all");
   initialize();
 }
 
 /**
  * The result of last action can be read using
  * this method.   
  * @return String the result of last action
  */  
 public String getActionResult(){
   return resultOfLastAction;
 }
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
  */
 public boolean setClient(Object clientApp){
   if (clientApp==null) return (false);
   varContextObject=clientApp; 
   varContextFields=clientApp.getClass().getFields();
   return (true);
 }

 /**
  * Links a client variable with a variable of the external application
  * @param clientVariable String the client variable
  * @param externalVariable String the external variable
  */
 public boolean linkVariables(String clientVariable, String externalVariable){   
   if (varContextObject==null) return (false);               
   int type;
   //Search if the ivar exists
   for (int i=0; i < varContextFields.length; i++) {
       if (clientVariable.equals((varContextFields[i]).getName())) {
         //Detect type
         if (varContextFields[i].getType().getName().equals("double")) type=DOUBLE;
           else if (varContextFields[i].getType().getName().equals("[D")) type=ARRAYDOUBLE;
         else if (varContextFields[i].getType().getName().equals("[[D")) type=ARRAYDOUBLE2D;
         else if (varContextFields[i].getType().getName().equals("java.lang.String")) type=STRING;
         else return (false);
                                            
        if (linkVector==null) {
         linkVector=new java.util.Vector<String[]>();
         linkIndex= new int[1];
         linkIndex[0]=i;
         linkType = new int[1]; 
         linkType[0]=type;         
       }else{
         int[] _linkIndex=new int[linkIndex.length+1];
           System.arraycopy(linkIndex,0,_linkIndex,0,linkIndex.length);
           _linkIndex[linkIndex.length]=i; 
         linkIndex=_linkIndex;
         int[] _linkType=new int[linkType.length+1];
           System.arraycopy(linkType,0,_linkType,0,linkType.length);
           _linkType[linkType.length]=type;  
         linkType=_linkType;         
       }        
        String[] _element={clientVariable,externalVariable};                         
        linkVector.addElement(_element);
       return (true);        
     }
   }       
   return(false);
 }

 /**
  * Clears all linking between variables
  */
 public void clearLinks(){
   varContextObject=null; 
   varContextFields=null;   
 }
 
//--------------------------------------------------------------
//Control commands
//--------------------------------------------------------------
 
 /**
  * Some external applications, such as MATLAB, can continuously execute  
  * a command after every step().
  * @param command a String to be executed
  */
 public void setCommand(String command){
   commandUpdate=command;
 }

 /**
  * Gets the command to be executed by the external application.  
  * @return String the command
  */ 
 public String getCommand(){
   return commandUpdate;
 }

 /**
  * Steps the application a given step or a number of times.
  * If getCommand() is non-null the command is executed that number of times.
  * If getCommand() is null, the dt parameter is passed down to the externall application,
  * and the actual meaning of this parameter dt will depend on the implementing class
  * @param dt double
  */
 public void step (double dt){
   if (modeAS)
     asynchronousStep(dt);
   else{
     synchronousStep(dt);
   }
 }

 /**
  * Synchronizes client and external applications
  */
 public void synchronize(){  
   removeBuffer=true;
   asynchronousSimulation=false;
 }
 
//--------------------------------------------------------------
//Remote methods
//-------------------------------------------------------------- 
 
 /**
  * Sets the package size used to group valus of the external variables
  */
 public void setPackageSize(int size){
   if (modeAS){
     synchronize();
     packageSize = (int) size;
   }
 }

 /**
  * Empty the buffer
  */
 public void flush(){
   try{
     bufferOutputTCP.flush();
    }catch(Exception e){
       System.out.println("Error flush Remote Matlab "+e);
    }
   flushNow=true;
 }
  
//--------------------------------------------------------------
//Other high level methods
//--------------------------------------------------------------
 
 /**
  * Forces or not to the client to wait until an external variable is read. 
  * @param set boolean 
  */
 public void setWaitForEver(boolean set){
   waitForEverFlag=set;
 }
 
 
//--------------------------------------------------------------
//Other public methods
//-------------------------------------------------------------- 
  
 /**
  * sets all external variables with client values
  */
 public void setValues(){
   int k=0;
   String[] var;
   String evar;
   
   //Set Values   
   for (int i=0; i<linkVector.size(); i++){
      var= (String[]) linkVector.elementAt(i);
      evar=var[1];
      try {       
        varContextField= varContextFields[linkIndex[k]];      
        switch (linkType[k++]){
        case DOUBLE: setValue(evar,varContextField.getDouble(varContextObject)); break;
        case ARRAYDOUBLE: setValue(evar,(double[])varContextField.get(varContextObject)); break;
        case ARRAYDOUBLE2D: setValue(evar,(double[][])varContextField.get(varContextObject)); break;
        case STRING: setValue(evar,(String)varContextField.get(varContextObject)); break; 
        }
        
      } catch (java.lang.IllegalAccessException e) {
        System.out.println("Error Step: setting a value " + e);
      }           
   }   
 }
  
 /**
  * gets all external values
  */
 public void getValues(){
   //Get Values   
   int k=0;
   String[] var;
   String evar;
   
   for (int i=0; i<linkVector.size(); i++){
      var= (String[]) linkVector.elementAt(i);
      evar=var[1];
      try {       
        varContextField= varContextFields[linkIndex[k]];      
        switch (linkType[k++]){
        case DOUBLE: varContextField.setDouble(varContextObject,getDouble(evar)); break;
        case ARRAYDOUBLE: varContextField.set(varContextObject,getDoubleArray(evar));  break;
        case ARRAYDOUBLE2D: varContextField.set(varContextObject,getDoubleArray2D(evar)); break;
        case STRING: varContextField.set(varContextObject,getString(evar)); break;        
        }
        
      } catch (java.lang.IllegalAccessException e) {
        System.out.println("Error Step: getting a value " + e);
      }           
   }      
 }
 

 /**
  * Package-internal constructor
  */
 public RMatlabExternalApp () {    
 }
 
 /**
  * Package-internal constructor with configuration String
  * @param remoteConfiguration a String to set the remote server
  */
 public RMatlabExternalApp (String remoteConfiguration) {
   EjsId = 2005;
   remoteConfiguration=remoteConfiguration.trim().toLowerCase().replace("<","").replace(">","");
   String[] params=remoteConfiguration.split(":");
   if (params.length!=3) {
     System.out.println("Error of Input Arguments");
     System.out.println("------------------------");
     System.out.println("Arguments for synchronous mode = \n <matlab:ipAddress:ipPort>");
     System.out.println("Arguments for asynchronous mode = \n <matlabas:ipAddress:ipPort>");
     return;
   }
   if (params[0].equals("matlabas")) modeAS=true;
   String sAdress=params[1];
   String sPort=params[2];
   SERVICE_IP = sAdress;
   try {
     SERVICE_PORT = Integer.parseInt(sPort);
   }
   catch (NumberFormatException nfe) {
     System.out.println("Error in Port:" + nfe);
   }
 }
 
 /**
  * Starts an authorized connection with the remote external application
  * @param user The user
  * @param pwd The password
  * @return boolean true if the the connection was successful
  */    
 public boolean connect (String user, String pwd){        
   asynchronousSimulation=false;      
   int result=-1;

   if (jimTCP!=null){
     if (!jimTCP.isClosed()) {resultOfLastAction=CONNECTION_OK;return true;} //user ok
   }
   try {
     jimTCP = new java.net.Socket(SERVICE_IP, SERVICE_PORT); 
     bufferInputTCP = new DataInputStream(new BufferedInputStream(jimTCP.getInputStream()));
     bufferOutputTCP = new DataOutputStream(new BufferedOutputStream(jimTCP.getOutputStream()));
     jimTCP.setSoTimeout(connectionTimeOut);
     jimTCP.setTcpNoDelay(true);

     //Configure buffer sizes
     int buffin=bufferInputTCP.readInt();
     int buffout=bufferInputTCP.readInt();
     jimTCP.setSendBufferSize(buffout);
     jimTCP.setReceiveBufferSize(buffin);
     
     //Is any authorization requried?
     Boolean aut=bufferInputTCP.readBoolean();

     long rtime=-1;

     //Authentication       
     if (!aut){
       result=2;
     }else{
       bufferOutputTCP.writeUTF(user);
       bufferOutputTCP.writeUTF(pwd);
       bufferOutputTCP.flush();
       result=bufferInputTCP.readInt();
       rtime=bufferInputTCP.readLong();
     }

     if (result==2) {
       System.out.println("Successful Connection");
       if (rtime>0)
         System.out.println("Time available:"+(rtime/(1000*60))+" minutes");
       connected = true;
       resultOfLastAction=CONNECTION_OK;
       return true;
     }      

     System.out.println("Authentication Error");        
     disconnect();
     if (result==0) {resultOfLastAction=connectionAuthenticationFail;return false;}
     resultOfLastAction=connectionSlotFail;
     return false;        
   }
   catch (IOException ioe) {
     System.out.println("Error " + ioe);
     connected=false;       
     resultOfLastAction=connectionNoServer;
     return false;
   }
   catch (Exception e2) {
     System.out.println("Error " + e2);
     connected=false;
     resultOfLastAction=connectionNoServer;
     return false;
   }
 }
 
 /**
  * Returns the remaining time
  * @return
  */
 public  long getRemainingTime(){
   long result=0;
   try {
     bufferOutputTCP.writeInt(EjsId);
     bufferOutputTCP.writeUTF("getRemainingTime");
     bufferOutputTCP.flush();
     result=bufferInputTCP.readLong();
   }
   catch (Exception e) {
     System.out.println(" getRemainingTime Exception:" + e);
   }
   return result;
 }
  
 /**
  * Synchronizes with the option to remove or not the data on buffer
  * @param _remove boolean parameter
  */
 public void synchronize(boolean _remove){
   if (modeAS){
     removeBuffer = _remove;
     asynchronousSimulation = false;
   }
 }
  
//--------------------------------------------------------------
//Protected method
//-------------------------------------------------------------- 
 
protected void setFlush(boolean flag){
  flushNow=flag;
}
 /**
  * Initializes a Matlab session. 
  */
 protected void initialize () {
   if (initCommand!=null) eval(initCommand);
 }
 
 /**
  * Stops the execution of the remote external application
  * @param _remove boolean true to delete all data in the buffer
  */
 protected void haltUpdate(boolean _remove){
   byte outFirma1=(byte)((int)(Math.random()*255)),outFirma2=(byte)((int)(Math.random()*255)),outFirma3=(byte)((int)(Math.random()*255));
   byte[] inFirma=new byte[3];
   try{
     bufferOutputTCP.writeInt (EjsId);
     bufferOutputTCP.writeUTF ("haltupdate");
     bufferOutputTCP.writeBoolean (_remove);
     bufferOutputTCP.flush();
   }
   catch (Exception e) {
     System.out.println(" halt update Remote Exception:"+e);
   }
   //remove old data
   if (_remove){
     try{
       int buffsizein= bufferInputTCP.available();
       while (buffsizein>0){
         bufferInputTCP.skip(buffsizein);
         buffsizein= bufferInputTCP.available();
       }
       bufferOutputTCP.writeByte(outFirma1);
       bufferOutputTCP.writeByte(outFirma2);
       bufferOutputTCP.writeByte(outFirma3);
       bufferOutputTCP.flush();
       inFirma[0]=bufferInputTCP.readByte();
       inFirma[1]= bufferInputTCP.readByte();
       inFirma[2]= bufferInputTCP.readByte();
       boolean synOK=false;
       while (!synOK){
         if ((inFirma[0]==outFirma1) && (inFirma[1]==outFirma2) && (inFirma[2]==outFirma3))
           synOK=true;
         else {
           inFirma[0]=inFirma[1];
           inFirma[1]=inFirma[2];
           inFirma[2]= bufferInputTCP.readByte();
         }
       }
     }catch (Exception e) {
       System.out.println(" halt update Remote Exception:"+e);
     }
   }
 }

 /**
  * Gets the value of a String variable of the application from buffer
  * @return String the value
  */
 protected String getStringAS() {
   String charArray = null;
   try{
     charArray = bufferInputTCP.readUTF();
   }catch(Exception e){
     System.out.println("Error getStringAS Remote Matlab "+e);
   }   
   return (charArray);
 }

 /**
  * Gets the value of a double variable of the application from buffer
  * @return double the value
  */
 protected double getDoubleAS() {
   double valueDouble=0.0;
   try{
     valueDouble= bufferInputTCP.readDouble();
   }catch(Exception e){
     System.out.println("Error getDoubleAS Remote Matlab "+e);
   }   
   return(valueDouble);
 }

 /**
  * Gets the value of a double[] variable of the application from buffer
  * @return double[] the value
  */
 protected double[] getDoubleArrayAS () {

   int _dim1;
   double[] vectorDouble=null;
   try{
     _dim1=bufferInputTCP.readInt();
     vectorDouble=new double[_dim1];
     for(int i=0;i<_dim1;i++) { vectorDouble[i]=bufferInputTCP.readDouble();}
   }catch(Exception e){
     System.out.println("Error getDoubleArrayAS Remote Matlab "+e);
   }    
   return(vectorDouble);
 }

 /**
  * Gets the value of a double[][] variable of the application from buffer
  * @return double[][] the value
  */
 protected double[][] getDoubleArray2DAS ()  {
   int _dim1,_dim2;
   double[][] arrayDouble=null;
   try{
     _dim1=bufferInputTCP.readInt();
     _dim2=bufferInputTCP.readInt();
     arrayDouble= new double[_dim1][_dim2] ;
     for(int i=0;i<_dim1;i++) {
       for(int j=0;j<_dim2;j++){
         arrayDouble[i][j]=bufferInputTCP.readDouble();
       }
     }
   }catch(Exception e){
     System.out.println("Error getDoubleArray2DAS Remote Matlab "+e);
   } 
   return(arrayDouble);
 }
 
 /**
  * Synchronous step
  */
 protected void synchronousStep (double dt) {  
   //Set Values 
   setValues();  
   //Do Step  
   int steps=(int)dt;
   for (int i=0;i<steps;i++){
      setFlush(false);
      eval(commandUpdate);
   }
   //Get Values   
   getValues();        
 }

 /**
  * Asynchronous step
  */
 protected  void asynchronousStep (double dt){  
   if (!asynchronousSimulation){     
     //Set Values  
     setValues();  
     int steps=(int)dt;
     haltUpdate(removeBuffer);
     try{
       //EJS identifier
       bufferOutputTCP.writeInt (EjsId);
       //remote function
       bufferOutputTCP.writeUTF ("stepMatlabAS");
       //send external variables to link
       String externalVars="";
       for (int i=0; i<linkVector.size(); i++){
         String[] _var = (String[]) linkVector.elementAt(i);
         if (externalVars.equals("")) externalVars=_var[1];
         else externalVars=externalVars+","+_var[1];
       }
       bufferOutputTCP.writeUTF (externalVars);      
       //command to execute
       bufferOutputTCP.writeUTF (commandUpdate);
       //number of command executions
       bufferOutputTCP.writeInt (steps);
       //size of the package
       bufferOutputTCP.writeInt(packageSize);
       //send
       bufferOutputTCP.flush();
     }catch (Exception e) {
       System.out.println("Remote Step Asynchronous Exception:"+e);
     }
     asynchronousSimulation=true;
   }  
   //Get Values
   getValues();
 }
 
//--------------------------------------------------------------
//private methods
//-------------------------------------------------------------- 

}
