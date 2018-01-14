import processing.serial.*;

Serial xbee;

String potVal;

void setup() {
  size(300,300);
  println(Serial.list());
  xbee = new Serial(this, Serial.list()[1],9600);
}

void draw()
{
  
}

void serialEvent(Serial xbee)
{
  potVal = xbee.readStringUntil(10);
  if (potVal != null) {
    println("Incoming data: " + potVal);
  }
}