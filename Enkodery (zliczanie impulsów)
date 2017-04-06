#include <RedBot.h>
RedBotMotors motors;

RedBotEncoder encoder = RedBotEncoder(A2, 10);
int buttonPin = 12;
int countsPerRev = 192;   // 4 pairs of N-S x 48:1 gearbox = 192 ticks per wheel rev

int lCount;
int rCount;

void setup()
{
  pinMode(buttonPin, INPUT_PULLUP);
  Serial.begin(9600);
  Serial.println("left    right");
  Serial.println("================");
}

void loop(void)
{
  lCount = encoder.getTicks(LEFT);
  rCount = encoder.getTicks(RIGHT);

  Serial.print(lCount);
  Serial.print("\t");
  Serial.println(rCount);
}
