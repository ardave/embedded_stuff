import micropython
from machine import Pin

# Allocate emergency exception buffer for ISR debugging
micropython.alloc_emergency_exception_buf(100)

# GPIO configuration (same as C code)
BLINK_GPIO = 8   # LED output
SWITCH_GPIO = 4  # Button input

# Initialize pins
led = Pin(BLINK_GPIO, Pin.OUT)
switch = Pin(SWITCH_GPIO, Pin.IN, Pin.PULL_UP)

def handle_switch_change(pin):
    """Called from main context via schedule() - safe to do complex operations."""
    switch_pressed = switch.value() == 0  # Active low (pull-up)
    led.value(switch_pressed)
    print(f"Switch {'pressed' if switch_pressed else 'released'}, LED {'on' if switch_pressed else 'off'}")

def switch_isr(pin):
    """ISR - keep minimal, defer work via schedule()."""
    micropython.schedule(handle_switch_change, pin)

# Configure interrupt on both edges (like GPIO_INTR_ANYEDGE in C)
switch.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, handler=switch_isr)

print("=" * 40)
print("Interrupt-based button->LED control ready")
print(f"LED on GPIO {BLINK_GPIO}, Switch on GPIO {SWITCH_GPIO}")
print("Press the button to toggle LED")
print("=" * 40)

# Keep the program running
while True:
    pass
