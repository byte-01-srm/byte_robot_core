import pygame
import sys
import time

def main():
    # Initialize the pygame library and joystick module
    pygame.init()
    pygame.joystick.init()

    # Check if any joysticks are connected
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No controllers found. Please connect your Red Gear controller.")
        pygame.quit()
        sys.exit()

    # Initialize the first connected joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()

    print(f"Detected Controller: {joystick.get_name()}")
    print(f"Number of Axes: {joystick.get_numaxes()}")
    print(f"Number of Buttons: {joystick.get_numbuttons()}")
    print(f"Number of Hats (D-Pads): {joystick.get_numhats()}")
    print("-" * 40)
    print("Listening for input... (Press Ctrl+C to stop)")

    try:
        while True:
            # Process pygame events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    break
                    
                # Button events (A, B, X, Y, Bumpers, etc.)
                elif event.type == pygame.JOYBUTTONDOWN:
                    print(f"Button Pressed: {event.button}")
                elif event.type == pygame.JOYBUTTONUP:
                    print(f"Button Released: {event.button}")
                
                # Analog stick and trigger events
                elif event.type == pygame.JOYAXISMOTION:
                    # Filter out small deadzone fluctuations
                    if abs(event.value) > 0.1:
                        print(f"Axis {event.axis} value: {event.value:.2f}")
                
                # D-Pad events
                elif event.type == pygame.JOYHATMOTION:
                    print(f"D-Pad (Hat) {event.hat} moved to: {event.value}")
            
            # Short sleep to prevent 100% CPU usage
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nExiting...")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        # Clean up
        pygame.quit()

if __name__ == "__main__":
    main()
