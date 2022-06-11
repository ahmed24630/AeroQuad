class Sonar
{
    public:
      int range;
      int addr;
      
      Sonar()
      {
      }
      
      void initialize(int address)
      {
          //addr=ADDRESS&(0x7F); //Wire only wants 7-bit addresses, chop off the highest bit
          addr=address;
      //    Serial.print("Initing Sonar at 0x");
      //    Serial.println(addr, HEX);
          requestRead();
          delay(70);
          
      //    Serial.println("Done initing sonar");
      }
      
      int getValue()
      {
          return range;
      }
      
      void measure()
      {
          int temp = readData();
          range = temp==0 ? range:temp;
          requestRead();
      }
          
        
    private:
      void requestRead();      
      int readData();
};

int Sonar::readData()
{
//    Serial.print("readData() called, addr: 0x");
//    Serial.println(addr, HEX);
    Wire.beginTransmission(addr);
      Wire.send(0x02);
    Wire.endTransmission();
    
    Wire.requestFrom(addr, 2);
     //while(Wire.available() < 2){};
     //Serial.println("Stopped Failing");
    int result = (Wire.receive()<<8);
    result += Wire.receive();
    
    return result;
}

void Sonar::requestRead()
{
  Wire.beginTransmission(addr);
    Wire.send(0x00);
    Wire.send(0x50);
  Wire.endTransmission();
}
