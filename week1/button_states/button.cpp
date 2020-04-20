# include  "button.h"

Button::Button(uint8_t pin, uint32_t db)
{
  buttonPin = pin;
  debouncePeriod = db;
}
void Button::Init(bool usePullup = true)
{
  if (usePullup) pinMode(buttonPin, INPUT_PULLUP);
  else pinMode (buttonPin, INPUT);
}
bool Button::CheckButtonPress(void)
{
  bool returnValue;

  switch (state) {

    case BUTTON_STABLE:
      if (buttonPosition != digitalRead(buttonPin)) {
        returnValue = false;
        tempButtonPos = digitalRead(buttonPin);
        lastBounceTime = millis();
        state = BUTTON_UNSTABLE;
      }
      break;

    case BUTTON_UNSTABLE:
      if (tempButtonPos != digitalRead(buttonPin)) {
        lastBounceTime = millis();
        tempButtonPos = digitalRead(buttonPin);
        returnValue = false;
      }
      else if (millis() - lastBounceTime >= debouncePeriod) {
        if (tempButtonPos == LOW && buttonPosition == HIGH) {
          state = BUTTON_STABLE;
          buttonPosition = digitalRead(buttonPin);
          returnValue = true;
        }
        else {
          state = BUTTON_STABLE;
          buttonPosition = digitalRead(buttonPin);
          returnValue = false;
        }
      }
      break;

    default:
      Serial.print("Error!");
  }

  return returnValue;
}
