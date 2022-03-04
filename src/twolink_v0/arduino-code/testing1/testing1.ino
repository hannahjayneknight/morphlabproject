int x;

void setup() {
  // initialize both serial ports:
  Serial.begin(115200);
  ///Serial1.begin(9600);
  int x = 0;
}



void loop() {
  // read from port 1, send to port 0:
  Serial.println(x);
  delay(10);
  //Serial.println();
  //Serial.write(x);
  x = x + 1;
  if ( x > 100) {
      x=0;
      delay(30);
    }

}
