/* FILE: sickdt50_reader
 * AUTHOR: William Willie Wells
 * DATE: May 2016
 *
 * Initialization time ≤ 250 ms
 * Response time ≤ 15ms.
 */
 
// begin serial output:
void setup(){ delay(250); Serial.begin(9600); }

// 10-bit analog stored
unsigned int q_analog;

// read from Sick DT50 and output via serial port:
void loop(){
  // read input: 6 bits of resolution lost
  q_analog = analogRead(A4);
  
  // write output
  Serial.print("\r\n");
  Serial.print(q_analog);
  Serial.print(", ");
  
  // set loop frequency
  delay(200);
}
