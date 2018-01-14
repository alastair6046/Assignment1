import processing.serial.*;
import g4p_controls.*;

Serial xbee;
String potVal;

GTextArea MessagesReceivedBox;
GLabel LblTextBox;

GTextArea SendMessageBox;
GLabel LblSendMessage;

GButton BtnSend; 

void setup() {
  size(800,600);
  println(Serial.list());
  xbee = new Serial(this, "COM16",9600);
  
  MessagesReceivedBox = new GTextArea(this, 50,300,700,300);
  LblTextBox = new GLabel(this, 50, 280, 560, 20, "Log Tray");
  LblTextBox.setTextAlign(GAlign.LEFT, null);
  
  SendMessageBox = new GTextArea(this, 50,100,700,100);
  LblSendMessage = new GLabel(this, 50, 80, 560, 20, "Send Tray");
  LblSendMessage.setTextAlign(GAlign.LEFT, null);
  
  BtnSend = new GButton(this, 650, 200, 100, 50, "Send");
}

void handleButtonEvents(GButton button, GEvent event)
{
  if (button == BtnSend)
  {
    xbee.write(SendMessageBox.getText(0));
  }
  // println("Button Clicked");
}


void handleTextEvents(GEditableTextControl textcontrol, GEvent event)
{
}

void draw()
{
  background(220, 220, 255);
}

void serialEvent(Serial xbee)
{
  potVal = xbee.readStringUntil(10);
  if (potVal != null) {
     MessagesReceivedBox.appendText(potVal);
    println("Incoming data: " + potVal);
  }
}