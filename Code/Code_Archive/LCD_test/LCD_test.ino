
#define D7 10
#define D6 9
#define D5 8
#define D4 7


#define rs   7   
#define en   6

void setup() {
  // put your setup code here, to run once:

}


void enable() {
  digitalWrite(en,HIGH);
  digitalWrite(en,LOW);
}

void port(int dec) {
  
  if (dec%2 == 1)
  digitalWrite(D4,HIGH);
  else
  digitalWrite(D4,LOW);
  dec = dec/2;
  if (dec%2 == 1)
  digitalWrite(D5,HIGH);
  else
  digitalWrite(D5,LOW);
  dec = dec/2;
  if (dec%2 == 1)
  digitalWrite(D6,HIGH);
  else
  digitalWrite(D6,LOW);
  dec = dec/2;
  if (dec%2 == 1)
  digitalWrite(D4,HIGH);
  else
  digitalWrite(D4,LOW);

  enable();
}

void busy() {

  digitalWrite(rs,LOW);
  while(D7)
  {
    enable();
    digitalRead(D7);
  }
}

void out( int hex) {

  int ubit = hex/100;
  int lbit = hex%100;
  port(ubit);
  port(lbit);
  busy();
}

void init() {

  port(15);
  delay(20*0.001);
  port(3);
  delay(10*0.001);
  port(3);
  delay(1*0.001);
  port(3);
  delay(1*0.001);
  port(2);
  delay(1*0.001);
  
}

void command(int numb) {

  digitalWrite(rs,LOW);
  out(numb);
}

void data(int numb) {

  digitalWrite(rs,HIGH);
  out(numb);
}

void loop() {
  // put your main code here, to run repeatedly:
  init();
  command(208);
  command(6);
  command(14);
  command(1);

  data(41);
  data(62);
  


  delay(100);
}


/*
 * //The pins used are same as explained earlier
#define lcd_port    P3
 
//LCD Registers addresses
#define LCD_EN      0x80
#define LCD_RS      0x20
 
void lcd_reset()
{
 lcd_port = 0xFF;
  delayms(20);
  lcd_port = 0x03+LCD_EN;
  lcd_port = 0x03;
  delayms(10);
  lcd_port = 0x03+LCD_EN;
  lcd_port = 0x03;
  delayms(1);
  lcd_port = 0x03+LCD_EN;
  lcd_port = 0x03;
  delayms(1);
  lcd_port = 0x02+LCD_EN;
  lcd_port = 0x02;
  delayms(1);
}
 
void lcd_init ()
{
  lcd_reset();         // Call LCD reset
  lcd_cmd(0x28);       // 4-bit mode - 2 line - 5x7 font. 
  lcd_cmd(0x0C);       // Display no cursor - no blink.
  lcd_cmd(0x06);       // Automatic Increment - No Display shift.
  lcd_cmd(0x80);       // Address DDRAM with 0 offset 80h.
 }
 */
