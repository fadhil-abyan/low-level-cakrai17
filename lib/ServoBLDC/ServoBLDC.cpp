// ServoBLDC.cpp
#include "ServoBLDC.h"

ServoBLDC::ServoBLDC(int throttlePin, int directionPin, int throttleMinUs, int throttleMaxUs)
    : _throttlePin(throttlePin),
      _directionPin(directionPin),
      _minPulseWidth(throttleMinUs),
      _maxPulseWidth(throttleMaxUs) {
}

void ServoBLDC::attach() {
    _throttleServo.attach(_throttlePin, _minPulseWidth, _maxPulseWidth);
    _directionServo.attach(_directionPin, 1000, 2000); // Direction servo usually uses 1000-2000
}

// Sets the motor speed and direction based on a percentage input.
// percentage: -100 (full reverse) to 100 (full forward). 0 is stop.
// dirCwUs: The microsecond value for CW direction (e.g., 1250 us)
// dirCcwUs: The microsecond value for CCW direction (e.g., 1750 us)
void ServoBLDC::setMotorPercentage(float percentage, int dirCwUs, int dirCcwUs) {
    int actualThrottleUs;
    int actualDirectionUs;

    // Clamp percentage to valid range -100 to 100
    percentage = constrain(percentage, -100, 100);

    if (percentage == 0) {
        // Stop the motor: send min throttle and any neutral/default direction
        actualThrottleUs = _minPulseWidth; // Assuming 1000us is stop for throttle ESC
        actualDirectionUs = dirCwUs; // Or any default, as throttle will be 1000us anyway
                                     // (it shouldn't matter much when throttle is 1000us)
    } else if (percentage > 0) { // Forward (CW)
        actualDirectionUs = dirCwUs;
        // Map 1% to 100% to a safe operating range for your ESC's throttle input.
        // Start from _minPulseWidth (1000) for 0% and go up to _maxPulseWidth (2000) for 100%.
        // We'll use 1000 + (1000 * percentage/100) or similar.
        // A common pattern is to map 0-100% to 1000-2000us
        // or a slightly adjusted range like 1000-1940 (manual default for throttle)
        // Let's use 1000-2000 for simplicity as your attach uses it.
        actualThrottleUs = map(percentage, 1, 100, _minPulseWidth, _maxPulseWidth);
    } else { // Reverse (CCW) - percentage is negative
        actualDirectionUs = dirCcwUs;
        // Map 1% to 100% (absolute value) to 1000-2000us range
        actualThrottleUs = map(abs(percentage), 1, 100, _minPulseWidth, _maxPulseWidth);
    }

    // Write the calculated microsecond values to the servos
    _throttleServo.writeMicroseconds(actualThrottleUs);
    _directionServo.writeMicroseconds(actualDirectionUs);
}